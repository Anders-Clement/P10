#!/usr/bin/python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time, Duration
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
from tf2_ros import TransformException
import tf_transformations

from nav2_msgs.action import NavigateToPose

import spice_mapf_msgs.msg as spice_mapf_msgs
import spice_mapf_msgs.srv as spice_mapf_srvs
import spice_mapf_msgs.action as spice_mapf_actions
import spice_msgs.msg as spice_msgs
import spice_msgs.srv as spice_srvs
import mapf_controller

AT_GOAL_THRESHOLD = 0.1

class MAPFNavigator(Node):
    
    def __init__(self) -> None:
        super().__init__('mapf_navigator_node')

        robot_ns = self.get_namespace()[1:]
        if robot_ns is None:
            self.get_logger().error('Could not get robot namespace')
            raise Exception()
        
        self.id = spice_msgs.Id(id=robot_ns, robot_type=spice_msgs.RobotType(type=spice_msgs.RobotType.CARRIER_ROBOT))
        
        self.join_planner_client = self.create_client(spice_mapf_srvs.JoinPlanner, "/join_planner")

        self.robot_pos_publisher = self.create_publisher(spice_mapf_msgs.RobotPose, "/robot_pos", 10)
        self.paths_subscriber = self.create_subscription(spice_mapf_msgs.RobotPoses, "/mapf_paths", self.paths_cb, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.controller = mapf_controller.MAPFController(self, self.tf_buffer)
        self.navigation_action_server = MAPFNavigatorActionServer(self, self)

        self.current_transform = None
        self.current_nav_step_goal: spice_mapf_msgs.RobotPose = None
        self.next_nav_step_goal: spice_mapf_msgs.RobotPose = None
        self.has_published_feedback = False

        self.joined_planner = False
        self.join_planner_future = None

        self.join_planner_timer = self.create_timer(1.0, self.try_join_planner)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if not self.get_robot_transform():
            return
        self.publish_current_pos()
        if self.at_step_goal():
            if self.next_nav_step_goal is not None:
                self.current_nav_step_goal = self.next_nav_step_goal
                self.next_nav_step_goal = None

        if self.current_nav_step_goal is not None:
            nav_goal = self.current_nav_step_goal
        else:
            nav_goal = spice_mapf_msgs.RobotPose()
            nav_goal.heading = self.current_transform.transform.rotation
            nav_goal.position.x = self.current_transform.transform.translation.x
            nav_goal.position.y = self.current_transform.transform.translation.y

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.current_transform.header.stamp
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = nav_goal.position.x
        goal_pose.pose.position.y = nav_goal.position.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = nav_goal.heading
        cmd_vel = self.controller.compute_cmd_vel(self.current_transform, goal_pose)
        if cmd_vel is not None:
            self.cmd_vel_publisher.publish(cmd_vel)
        else:
            self.cmd_vel_publisher.publish(Twist()) # empty if controller cannot run
    
    def at_step_goal(self) -> bool:
        if self.current_nav_step_goal is None:
            return True # assume we are there, if no goal is present
        x_diff = self.current_nav_step_goal.position.x - self.current_transform.transform.translation.x
        y_diff = self.current_nav_step_goal.position.y - self.current_transform.transform.translation.y
        distance = math.sqrt(x_diff**2 + y_diff**2)
        return distance < AT_GOAL_THRESHOLD
    
    def is_available_for_navigation(self) -> bool:
        got_transform = self.get_robot_transform()
        return self.joined_planner and got_transform

    def try_join_planner(self):
        if self.join_planner_future is not None:
            return
        if not self.get_robot_transform():
            return
        self.join_planner_timer.cancel()
        while not self.join_planner_client.wait_for_service(1.0):
            self.get_logger().info(f'timeout on wait for service: "/join_planner"')

        self.get_robot_transform() # to ensure it is up to date, as robot can move during the above wait
        join_msg = spice_mapf_srvs.JoinPlanner.Request()
        join_msg.robot_pose.id = self.id

        if self.current_nav_step_goal is None:
            join_msg.robot_pose.position.x = self.current_transform.transform.translation.x
            join_msg.robot_pose.position.y = self.current_transform.transform.translation.y
        else:
            join_msg.robot_pose.position.x = self.current_nav_step_goal.position.x
            join_msg.robot_pose.position.y = self.current_nav_step_goal.position.y

        if self.current_nav_step_goal is None:
            self.current_nav_step_goal = join_msg.robot_pose # control will now go to this point
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
            self.join_planner_timer.reset()
    
    def publish_current_pos(self):
        if not self.joined_planner:
            return
        pose_msg = spice_mapf_msgs.RobotPose()
        pose_msg.position.x = self.current_transform.transform.translation.x
        pose_msg.position.y = self.current_transform.transform.translation.y
        pose_msg.heading = self.current_transform.transform.rotation
        pose_msg.id = self.id
        self.robot_pos_publisher.publish(pose_msg)

    def get_robot_transform(self) -> bool:
        try:
            from_frame_rel = "base_link"
            to_frame_rel = "map"
            self.current_transform = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, Time())
            return True
        except TransformException as e:
            self.get_logger().info(
                        f'Failed to get robot transform map->base_link: {e}', once=False)
            return False
        
    def paths_cb(self, msg: spice_mapf_msgs.RobotPoses):
        poses: list[spice_mapf_msgs.RobotPose] = msg.poses
        for robot_pose in poses:
            if robot_pose.id == self.id:
                if robot_pose.rejoin:
                    self.join_planner_timer.reset()
                    self.try_join_planner()
                    self.join_planner_future = None
                    self.planner_type_is_set = False
                    self.planner_type_future = None
                    self.navigation_action_server.current_nav_goal = None
                    self.next_nav_step_goal = None
                    return

                if self.current_nav_step_goal != robot_pose:
                    self.get_logger().info(f'Got new pose from path: {robot_pose.position}')
                    if self.at_step_goal():
                        self.current_nav_step_goal = robot_pose
                    else:
                        self.next_nav_step_goal = robot_pose
                    self.has_published_feedback = False
                return
    

class MAPFNavigatorActionServer():
    def __init__(self, nodehandle: Node, navigator: MAPFNavigator) -> None:
        self.nodehandle = nodehandle
        self.navigator = navigator
        self.request_goal_client = self.nodehandle.create_client(spice_mapf_srvs.RequestGoal, "/request_goal")
        self.navigate_mapf_server = ActionServer(self.nodehandle, 
                                                 spice_mapf_actions.NavigateMapf,
                                                 "navigate_mapf",
                                                 execute_callback=self.navigate_mapf_cb,
                                                 goal_callback=self.navigate_mapf_goal_cb
                                                 )
        self.current_nav_goal = None

    def req_goal(self, request_goal, goal_handle):
        request_goal_future = self.request_goal_client.call_async(request_goal)
        rate = self.nodehandle.create_rate(10, self.nodehandle.get_clock())
        TIMEOUT = 5.0
        self.nodehandle.get_logger().info(f'Requesting goal')
        start_time = self.nodehandle.get_clock().now()
        while not request_goal_future.done():
            rate.sleep()
            wait_time = self.nodehandle.get_clock().now() - start_time
            if wait_time.to_msg().sec > TIMEOUT:
                self.nodehandle.get_logger().warn(f'Timeout on request_goal')
                self.current_nav_goal = None
                goal_handle.abort()
                return None            
            
        result: spice_mapf_srvs.RequestGoal.Response = request_goal_future.result()
        return result

    def navigate_mapf_goal_cb(self, goal_request: spice_mapf_actions.NavigateMapf.Goal):
        self.nodehandle.get_logger().info(f'Received goal request: {goal_request.goal_pose}')
        if self.navigator.is_available_for_navigation() and self.current_nav_goal is None:
            return GoalResponse.ACCEPT
        else:
            self.nodehandle.get_logger().info(f'Rejecting goal, already navigating, or have not joined planner yet')
            return GoalResponse.REJECT            
    
    def navigate_mapf_cb(self, goal_handle: ServerGoalHandle):
        self.nodehandle.get_logger().info(f'Executing request')
            
        goal: spice_mapf_actions.NavigateMapf.Goal = goal_handle.request
        self.current_nav_goal = spice_mapf_msgs.RobotPose()
        self.current_nav_goal.position.x = goal.goal_pose.pose.position.x
        self.current_nav_goal.position.y = goal.goal_pose.pose.position.y
        self.current_nav_goal.id = self.navigator.id

        request_goal = spice_mapf_srvs.RequestGoal.Request()
        request_goal.workcell_id = goal.workcell_id
        request_goal.robot_pose = self.current_nav_goal
        
        can_go = False
        while not can_go:
            result = self.req_goal(request_goal, goal_handle)
            if result is None:
                return spice_mapf_actions.NavigateMapf.Result(success=False)
            elif not result.success and result.currently_occupied:
                self.nodehandle.get_logger().info(f'Requested goal is currently blocked by other robot. Waiting...')
                time.sleep(1)
                continue
            elif not result.success:
                self.nodehandle.get_logger().info(f'Requested goal was denied')
                goal_handle.abort()
                self.current_nav_goal = None
                return spice_mapf_actions.NavigateMapf.Result(success=False)
            elif result.success and not result.currently_occupied:
                break

        self.nodehandle.get_logger().info(
            f'Requested goal: {self.current_nav_goal.position}, \
                going to: {result.goal_position}, \
                robot is starting at: {self.navigator.current_transform.transform.translation}')
        self.current_nav_goal.position = result.goal_position
        rate = self.nodehandle.create_rate(5, self.nodehandle.get_clock())

        # wait until goal is reached
        while not self.at_goal():
            rate.sleep()

        goal_handle.succeed()
        self.current_nav_goal = None
        self.nodehandle.get_logger().info(f'Reached navigation goal successfully')
        return spice_mapf_actions.NavigateMapf.Result(success=True)

    def at_goal(self) -> bool:
        if self.current_nav_goal is None: # no goal, assume we are there
            return True
        
        x_diff = self.current_nav_goal.position.x - self.navigator.current_transform.transform.translation.x
        y_diff = self.current_nav_goal.position.y - self.navigator.current_transform.transform.translation.y
        distance = math.sqrt(x_diff**2 + y_diff**2)

        q_nav = self.current_nav_goal.heading
        goal_rpy = tf_transformations.euler_from_quaternion([q_nav.w, q_nav.x, q_nav.y, q_nav.z])
        q_robot = self.navigator.current_transform.transform.rotation
        robot_rpy = tf_transformations.euler_from_quaternion([q_robot.w, q_robot.x, q_robot.y, q_robot.z])
        goal_yaw = goal_rpy[2]
        robot_yaw = robot_rpy[2]

        return distance < AT_GOAL_THRESHOLD and goal_yaw-robot_yaw < AT_GOAL_THRESHOLD


if __name__ == '__main__':
    rclpy.init()
    test_node = MAPFNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(test_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass