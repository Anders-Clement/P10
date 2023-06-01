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

AT_GOAL_THRESHOLD = 0.15

class MAPFNavigator(Node):
    
    def __init__(self) -> None:
        super().__init__('mapf_navigator_node')

        robot_ns = self.get_namespace()[1:]
        if robot_ns is None:
            self.get_logger().error('Could not get robot namespace')
            raise Exception()
        
        self.id = spice_msgs.Id(id=robot_ns, robot_type=spice_msgs.RobotType(type=spice_msgs.RobotType.CARRIER_ROBOT))
        
        self.join_planner_client = self.create_client(spice_mapf_srvs.JoinPlanner, "/join_planner")
        self.request_goal_client = self.create_client(spice_mapf_srvs.RequestGoal, "/request_goal")

        self.robot_pos_publisher = self.create_publisher(spice_mapf_msgs.RobotPose, "/robot_pos", 10)
        self.paths_subscriber = self.create_subscription(spice_mapf_msgs.RobotPoses, "/mapf_paths", self.paths_cb, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.navigate_mapf_server = ActionServer(self, 
                                                 spice_mapf_actions.NavigateMapf,
                                                 "navigate_mapf",
                                                 execute_callback=self.navigate_mapf_cb,
                                                 goal_callback=self.navigate_mapf_goal_cb
                                                 )

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.controller = mapf_controller.MAPFController(self, self.tf_buffer)

        self.current_transform = None
        self.current_nav_goal: spice_mapf_msgs.RobotPose = None
        self.current_nav_step_goal: spice_mapf_msgs.RobotPose = None
        self.has_published_feedback = False

        self.joined_planner = False
        self.join_planner_future = None

        self.join_planner_timer = self.create_timer(1.0, self.try_join_planner)
        self.publish_pos_timer = self.create_timer(1.0, self.publish_current_pos)

    def navigate_mapf_goal_cb(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.goal_pose}')
        got_transform = self.get_robot_transform()
        if not (self.joined_planner or got_transform) or self.current_nav_goal is not None:
            self.get_logger().info(f'Rejecting goal, ready: {self.joined_planner}, self.current_nav_goal: {self.current_nav_goal}')
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
        self.get_logger().info(f'Executing request')
            
        goal: spice_mapf_actions.NavigateMapf.Goal = goal_handle.request
        self.current_nav_goal = spice_mapf_msgs.RobotPose()
        self.current_nav_goal.position.x = goal.goal_pose.pose.position.x
        self.current_nav_goal.position.y = goal.goal_pose.pose.position.y
        self.current_nav_goal.id = self.id
        request_goal = spice_mapf_srvs.RequestGoal.Request()
        request_goal.workcell_id = goal.workcell_id
        request_goal.robot_pose = self.current_nav_goal

        self.get_robot_transform()
        
        can_go = False
        while not can_go:
            result = self.req_goal(request_goal, goal_handle)
            if result is None:
                return spice_mapf_actions.NavigateMapf.Result(success=False)
            elif not result.success and result.currently_occupied:
                self.get_logger().info(f'Requested goal is currently blocked by other robot. Waiting...')
                time.sleep(1)
                continue
            elif not result.success:
                self.get_logger().info(f'Requested goal was denied')
                goal_handle.abort()
                self.current_nav_goal = None
                return spice_mapf_actions.NavigateMapf.Result(success=False)
            elif result.success and not result.currently_occupied:
                self.current_nav_step_goal = spice_mapf_msgs.RobotPose()
                self.current_nav_step_goal.heading = self.current_transform.transform.rotation
                self.current_nav_step_goal.position.x = self.current_transform.transform.translation.x
                self.current_nav_step_goal.position.y = self.current_transform.transform.translation.y
                break

        self.get_logger().info(f'Requested goal: {self.current_nav_goal.position}, going to: {result.goal_position}, robot is starting at: {self.current_transform.transform.translation}')
        self.current_nav_goal.position = result.goal_position
        self.get_logger().info(f'Accepted goal, starting navigation')
        rate = self.create_rate(10, self.get_clock())
        while not self.at_goal():
            rate.sleep()
            self.get_robot_transform()
            self.publish_current_pos()
            if self.at_step_goal():
                if not self.has_published_feedback:
                    self.has_published_feedback = True
                    x_diff_goal = self.current_nav_goal.position.x - self.current_transform.transform.translation.x
                    y_diff_goal = self.current_nav_goal.position.y - self.current_transform.transform.translation.y
                    feedback = spice_mapf_actions.NavigateMapf.Feedback()
                    feedback.distance_to_goal = abs(x_diff_goal) + abs(y_diff_goal)
                    goal_handle.publish_feedback(feedback)
            
            robot_pose = PoseStamped()
            robot_pose.header = self.current_transform.header
            robot_pose.pose.position.x = self.current_transform.transform.translation.x
            robot_pose.pose.position.y = self.current_transform.transform.translation.y
            robot_pose.pose.position.z = self.current_transform.transform.translation.z
            robot_pose.pose.orientation = self.current_transform.transform.rotation

            goal_pose = PoseStamped()
            goal_pose.header.stamp = robot_pose.header.stamp
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = self.current_nav_step_goal.position.x
            goal_pose.pose.position.y = self.current_nav_step_goal.position.y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation = self.current_nav_step_goal.heading
            cmd_vel = self.controller.compute_cmd_vel(robot_pose, goal_pose)
            if cmd_vel is not None:
                self.cmd_vel_publisher.publish(cmd_vel)

        # ensure last cmd_vel is zero:
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

        goal_handle.succeed()
        self.current_nav_goal = None
        self.current_nav_step_goal = None
        self.get_logger().info(f'Reached navigation goal successfully')
        return spice_mapf_actions.NavigateMapf.Result(success=True)

    def at_goal(self) -> bool:
        if self.current_nav_goal is None: # no goal, assume we are there
            return True
        
        x_diff = self.current_nav_goal.position.x - self.current_transform.transform.translation.x
        y_diff = self.current_nav_goal.position.y - self.current_transform.transform.translation.y
        dist = np.linalg.norm([x_diff, y_diff])

        q_nav = self.current_nav_goal.heading
        goal_rpy = tf_transformations.euler_from_quaternion([q_nav.w, q_nav.x, q_nav.y, q_nav.z])
        q_robot = self.current_transform.transform.rotation
        robot_rpy = tf_transformations.euler_from_quaternion([q_robot.w, q_robot.x, q_robot.y, q_robot.z])
        goal_yaw = goal_rpy[2]
        robot_yaw = robot_rpy[2]

        return dist < AT_GOAL_THRESHOLD and goal_yaw-robot_yaw < AT_GOAL_THRESHOLD
    
    def at_step_goal(self) -> bool:
        if self.current_nav_step_goal is None:
            return True # assume we are there, if no goal is present
        x_diff = self.current_nav_step_goal.position.x - self.current_transform.transform.translation.x
        y_diff = self.current_nav_step_goal.position.y - self.current_transform.transform.translation.y
        dist = np.linalg.norm([x_diff, y_diff])
        return dist < AT_GOAL_THRESHOLD

    def try_join_planner(self):
        if self.join_planner_future is not None:
            return
        if not self.get_robot_transform():
            return
        self.join_planner_timer.cancel()
        while not self.join_planner_client.wait_for_service(1.0):
            self.get_logger().info(f'timeout on wait for service: "/join_planner"')
            self.get_robot_transform() # to ensure it is up to date, as robot can move during this wait
        join_msg = spice_mapf_srvs.JoinPlanner.Request()
        join_msg.robot_pose.id = self.id
        join_msg.robot_pose.position.x = self.current_transform.transform.translation.x
        join_msg.robot_pose.position.y = self.current_transform.transform.translation.y
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
        if self.get_robot_transform():
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
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}', once=False)
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
                    self.current_nav_goal = None
                    return

                if self.current_nav_step_goal != robot_pose:
                    self.get_logger().info(f'Got new pose from path: {robot_pose.position}')
                    self.current_nav_step_goal = robot_pose
                    self.has_published_feedback = False
                return
    

if __name__ == '__main__':
    rclpy.init()
    test_node = MAPFNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(test_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass