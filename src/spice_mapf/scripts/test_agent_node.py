#!/usr/bin/python3

import numpy as np
import random

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import TransformException

from nav2_msgs.action import NavigateToPose

import spice_mapf_msgs.msg as spice_mapf_msgs
import spice_mapf_msgs.srv as spice_mapf_srvs
import spice_msgs.msg as spice_msgs
import spice_msgs.srv as spice_srvs

GOALS = [(2,2), (3,3), (4,4), (5,5)]

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

        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        if not self.get_robot_transform():
            return
        
        if not self.planner_type_is_set:
            self.set_planner_type()
            return
        
        if not self.joined_planner:
            self.try_join_planner()
            return
        
        self.publish_current_pos()    

        if self.nav_client_handle is not None: 
            if self.nav_client_handle.status == GoalStatus.STATUS_SUCCEEDED or \
                self.nav_client_handle.status == GoalStatus.STATUS_ABORTED or \
                self.nav_client_handle.status == GoalStatus.STATUS_CANCELED:

                self.nav_client_handle = None

                if self.current_nav_goal is not None: # ensure we are at goal
                    x_diff = self.current_nav_goal.position.x - self.current_transform.transform.translation.x
                    y_diff = self.current_nav_goal.position.y - self.current_transform.transform.translation.y
                    dist = np.linalg.norm([x_diff, y_diff])
                    if dist > 0.25:
                        self.navigate_to_goal()
                    else:
                        self.current_nav_goal = None

        else: # we have not started navigating yet
            if self.at_goal():
                self.select_random_goal()

    def at_goal(self) -> bool:
        if self.current_nav_goal is None: # no goal, assume we are there
            return True
        goal_position = self.current_nav_goal.position
        current_position = self.current_transform.transform.translation
        x_diff = goal_position.x - current_position.x
        y_diff = goal_position.y - current_position.y
        dist = np.linalg.norm([x_diff, y_diff])
        return dist < 0.25

    def select_random_goal(self):
        rand_goal = GOALS[int(random.random()*len(GOALS))]
        self.current_nav_goal = spice_mapf_msgs.RobotPose()

        self.current_nav_goal.position.x = float(rand_goal[0])
        self.current_nav_goal.position.y = float(rand_goal[1])
        self.current_nav_goal.id = self.id
        request_goal = spice_mapf_srvs.RequestGoal.Request()
        request_goal.robot_pose = self.current_nav_goal
        request_goal_future = self.request_goal_client.call_async(request_goal)
        request_goal_future.add_done_callback(self.request_goal_cb)

    def request_goal_cb(self, future: Future):
        result: spice_mapf_srvs.RequestGoal.Response = future.result()
        if not result.success:
            self.current_nav_goal = None
        else:
            self.current_nav_goal.position = result.goal_position

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
            self.get_logger().warn(f'Failed to join planner, retrying')
            self.try_join_planner()
    
    def publish_current_pos(self):
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
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
            return False
        
    def navigate_to_goal(self):

        x_diff = self.current_nav_step_goal.position.x - self.current_transform.transform.translation.x
        y_diff = self.current_nav_step_goal.position.y - self.current_transform.transform.translation.y
        dist = np.linalg.norm([x_diff,y_diff])
        if dist < 0.1:
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.pose.position.x = self.current_nav_step_goal.position.x
        nav_goal.pose.pose.position.y = self.current_nav_step_goal.position.y
        nav_goal.pose.pose.orientation = self.current_nav_step_goal.heading
        # TODO: add rotation
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.nav_reponse_future = self.navigation_client.send_goal_async(
            nav_goal) #, self.on_nav_feedback)
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
        for robot_pose in msg.poses:
            if robot_pose.id == self.id:
                if self.current_nav_step_goal != robot_pose:
                    self.get_logger().info(f'Got new pose from path: {robot_pose}')
                    self.current_nav_step_goal = robot_pose
                    self.navigate_to_goal()
                return

    # def on_nav_feedback(self, msg):
    #     pass
    
    def on_nav_done(self, future: Future):
        nav_result = future.result()
        nav_goal_result: GoalStatus = nav_result.status
        self.get_logger().info('Navigation result: ' + str(nav_goal_result))


if __name__ == '__main__':
    rclpy.init()
    test_node = MAPFNavigator()
    rclpy.spin(test_node)