#!/usr/bin/python3

import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import TransformException

from nav2_msgs.action import NavigateToPose

import spice_mapf_msgs.msg as spice_mapf_msgs
import spice_mapf_msgs.srv as spice_mapf_srvs
import spice_mapf_msgs.action as spice_mapf_actions
import spice_msgs.msg as spice_msgs
import spice_msgs.srv as spice_srvs

AT_GOAL_THRESHOLD = 0.15

class MAPFNavigator(Node):
    
    def __init__(self) -> None:
        super().__init__('mapf_navigator_node')

        robot_ns = self.get_namespace()[1:]
        if robot_ns is None:
            self.get_logger().error('Could not get robot namespace')
            raise Exception()
        
        self.id = spice_msgs.Id(id=robot_ns, robot_type=spice_msgs.RobotType(type=spice_msgs.RobotType.CARRIER_ROBOT))
        
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.change_planner_type_client = self.create_client(spice_srvs.SetPlannerType, "set_planner_type")
        self.join_planner_client = self.create_client(spice_mapf_srvs.JoinPlanner, "/join_planner")
        self.request_goal_client = self.create_client(spice_mapf_srvs.RequestGoal, "/request_goal")

        self.robot_pos_publisher = self.create_publisher(spice_mapf_msgs.RobotPose, "/robot_pos", 10)
        self.paths_subscriber = self.create_subscription(spice_mapf_msgs.RobotPoses, "/mapf_paths", self.paths_cb, 10)

        self.navigate_mapf_server = ActionServer(self, 
                                                 spice_mapf_actions.NavigateMapf,
                                                 "navigate_mapf",
                                                 execute_callback=self.navigate_mapf_cb,
                                                 goal_callback=self.navigate_mapf_goal_cb
                                                 )

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.current_transform = None
        self.current_nav_goal: spice_mapf_msgs.RobotPose = None
        self.current_nav_step_goal: spice_mapf_msgs.RobotPose = None
        self.nav_client_handle = None

        self.joined_planner = False
        self.join_planner_future = None
        self.planner_type_is_set = False
        self.planner_type_future = None

        self.make_ready_timer = self.create_timer(0.2, self.make_ready_timer_cb)

    def make_ready_timer_cb(self):
        self.make_ready()
        self.publish_current_pos()

    def navigate_mapf_goal_cb(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.goal_pose}')
        if not self.make_ready() or self.current_nav_goal is not None:
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT
        
    def req_goal(self, request_goal, goal_handle):
            request_goal_future = self.request_goal_client.call_async(request_goal)
            rate = self.create_rate(10, self.get_clock())
            TIMEOUT = 5.0
            self.get_logger().info(f'Requesting goal')
            start_time = self.get_clock().now()
            while not request_goal_future.done():
                rate.sleep()
                wait_time = self.get_clock().now() - start_time
                if wait_time.to_msg().sec > TIMEOUT:
                    self.get_logger().warn(f'Timeout on request_goal')
                    self.current_nav_goal = None
                    goal_handle.abort()
                    return None            
                
            result: spice_mapf_srvs.RequestGoal.Response = request_goal_future.result()
            return result
    
    def navigate_mapf_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing reqest')
            
        goal: spice_mapf_actions.NavigateMapf.Goal = goal_handle.request
        self.current_nav_goal = spice_mapf_msgs.RobotPose()
        self.current_nav_goal.position.x = goal.goal_pose.pose.position.x
        self.current_nav_goal.position.y = goal.goal_pose.pose.position.y
        self.current_nav_goal.id = self.id
        request_goal = spice_mapf_srvs.RequestGoal.Request()
        request_goal.workcell_id = goal.workcell_id
        request_goal.robot_pose = self.current_nav_goal
        
        can_go = False
        while not can_go:
            result = self.req_goal(request_goal, goal_handle)
            if result is None:
                return spice_mapf_actions.NavigateMapf.Result(success=False)
            elif not result.success and result.currently_occupied:
                self.get_logger().info(f'Requested goal is currently blocked by other robot. Waiting...')
                time.sleep(1)
            elif not result.success:
                self.get_logger().info(f'Requested goal was denied')
                goal_handle.abort()
                self.current_nav_goal = None
                return spice_mapf_actions.NavigateMapf.Result(success=False)
            elif result.success and not result.currently_occupied:
                can_go = True

        self.get_logger().info(f'Requested goal: {self.current_nav_goal.position}, going to: {result.goal_position}, robot is starting at: {self.current_transform.transform.translation}')
        self.current_nav_goal.position = result.goal_position
        self.get_logger().info(f'Accepted goal, starting navigation')
        self.navigate_to_goal()
        rate = self.create_rate(10, self.get_clock())
        while not self.at_goal():
            rate.sleep()
            if self.nav_client_handle is not None: # currently nagivating
                if self.nav_client_handle.status == GoalStatus.STATUS_SUCCEEDED or \
                    self.nav_client_handle.status == GoalStatus.STATUS_ABORTED or \
                    self.nav_client_handle.status == GoalStatus.STATUS_CANCELED:

                    self.nav_client_handle = None

                    if self.current_nav_step_goal is not None: # ensure we are at goal
                        if not self.at_step_goal():
                            self.get_logger().info(f'Navigating to step_goal again')
                            self.navigate_to_goal()

                        x_diff_goal = self.current_nav_goal.position.x - self.current_transform.transform.translation.x
                        y_diff_goal = self.current_nav_goal.position.y - self.current_transform.transform.translation.y
                        feedback = spice_mapf_actions.NavigateMapf.Feedback()
                        feedback.distance_to_goal = abs(x_diff_goal) + abs(y_diff_goal)
                        goal_handle.publish_feedback(feedback)
            else:
                if not self.at_step_goal():
                    self.navigate_to_goal()

        goal_handle.succeed()
        self.current_nav_goal = None
        self.current_nav_step_goal = None
        self.get_logger().info(f'Reached navigation goal successfully')
        return spice_mapf_actions.NavigateMapf.Result(success=True)
        

    def make_ready(self) -> bool:
        if not self.get_robot_transform():
            return False
        
        if not self.planner_type_is_set:
            self.set_planner_type()
            return False
        
        if not self.joined_planner:
            self.try_join_planner()
            return False
        
        return True

    def at_goal(self) -> bool:
        if self.current_nav_goal is None: # no goal, assume we are there
            return True
        goal_position = self.current_nav_goal.position
        current_position = self.current_transform.transform.translation
        x_diff = goal_position.x - current_position.x
        y_diff = goal_position.y - current_position.y
        dist = np.linalg.norm([x_diff, y_diff])
        return dist < AT_GOAL_THRESHOLD
    
    def at_step_goal(self) -> bool:
        if self.current_nav_step_goal is None:
            return True # assume we are there, if no goal is present
        x_diff = self.current_nav_step_goal.position.x - self.current_transform.transform.translation.x
        y_diff = self.current_nav_step_goal.position.y - self.current_transform.transform.translation.y
        dist = np.linalg.norm([x_diff, y_diff])
        return dist < AT_GOAL_THRESHOLD
    
    def set_planner_type(self):
        if self.planner_type_future is None:
            msg = spice_srvs.SetPlannerType.Request()
            msg.planner_type.type = spice_msgs.PlannerType.PLANNER_STRAIGHT_LINE
            while not self.change_planner_type_client.wait_for_service(1.0):
                self.get_logger().info(f'timeout on wait for service: "set_planner_type"')
            self.planner_type_future = self.change_planner_type_client.call_async(msg)
            self.planner_type_future.add_done_callback(self.planner_type_cb)

    def planner_type_cb(self, future: Future):
        self.planner_type_future = None
        result: spice_srvs.SetPlannerType.Response = future.result()
        if result.success:
            self.planner_type_is_set = True
            self.get_logger().info(f'Agent {self.id.id} set planner type')
        else:
            self.get_logger().error('Failed to set planner type!')

    def try_join_planner(self):
        if self.join_planner_future is not None:
            return
        join_msg = spice_mapf_srvs.JoinPlanner.Request()
        join_msg.robot_pose.id = self.id
        join_msg.robot_pose.position.x = self.current_transform.transform.translation.x
        join_msg.robot_pose.position.y = self.current_transform.transform.translation.y
        while not self.join_planner_client.wait_for_service(1.0):
            self.get_logger().info(f'timeout on wait for service: "/join_planner"')
        self.join_planner_future = self.join_planner_client.call_async(join_msg)
        self.join_planner_future.add_done_callback(self.join_planner_cb)

    def join_planner_cb(self, future: Future):
        self.join_planner_future = None
        result: spice_mapf_srvs.JoinPlanner.Response = future.result()
        if result.success:
            self.joined_planner = True
            self.get_logger().info(f'Agent {self.id.id} joined mapf planner')
        else:
            self.get_logger().warn(f'Failed to join planner, waiting 1s and retrying')
            time.sleep(1)
            self.try_join_planner()
    
    def publish_current_pos(self):
        if not self.joined_planner:
            return
        if self.current_transform is not None:
            pose_msg = spice_mapf_msgs.RobotPose()
            pose_msg.position.x = self.current_transform.transform.translation.x
            pose_msg.position.y = self.current_transform.transform.translation.y
            pose_msg.heading = self.current_transform.transform.rotation
            pose_msg.id = self.id
            self.robot_pos_publisher.publish(pose_msg)

    def get_robot_transform(self) -> bool:
        try:
            from_frame_rel = self.id.id + "_base_link"
            to_frame_rel = "map"
            self.current_transform = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, Time())
            return True
        except TransformException as e:
            self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}', once=True)
            return False
        
    def navigate_to_goal(self):
        if self.current_nav_step_goal is None:
            return
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.pose.position.x = self.current_nav_step_goal.position.x
        nav_goal.pose.pose.position.y = self.current_nav_step_goal.position.y
        nav_goal.pose.pose.orientation = self.current_nav_step_goal.heading
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.nav_reponse_future = self.navigation_client.send_goal_async(
            nav_goal)
        self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)
        self.get_logger().info(f'Agent {self.id.id} trying to navigate to {nav_goal.pose.pose.position}')

    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav 2 goal was rejected!')

        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.on_nav_done)

        self.nav_client_handle = goal_handle
        
    def paths_cb(self, msg: spice_mapf_msgs.RobotPoses):
        poses: list[spice_mapf_msgs.RobotPose] = msg.poses
        for robot_pose in poses:
            if robot_pose.id == self.id:
                if self.current_nav_step_goal != robot_pose:
                    current_position = self.current_transform.transform.translation
                    x_diff = robot_pose.position.x - current_position.x
                    y_diff = robot_pose.position.y - current_position.y
                    dist = np.linalg.norm([x_diff, y_diff])
                    # if dist > AT_GOAL_THRESHOLD:
                    self.get_logger().info(f'Got new pose from path: {robot_pose.position}')
                    self.current_nav_step_goal = robot_pose
                    self.navigate_to_goal()
                return
    
    def on_nav_done(self, future: Future):
        nav_result = future.result()
        nav_goal_result: GoalStatus = nav_result.status
        self.get_logger().info('Navigation result: ' + str(nav_goal_result))


if __name__ == '__main__':
    rclpy.init()
    test_node = MAPFNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(test_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass