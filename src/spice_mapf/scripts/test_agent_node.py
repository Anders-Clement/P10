#!/usr/bin/python3

import os
import enum
import sys

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.action import ActionClient
import tf2_ros
from tf2_ros import TransformException

from nav2_msgs.action import NavigateToPose

import spice_mapf_msgs.msg as spice_mapf_msgs
import spice_mapf_msgs.srv as spice_mapf_srvs
import spice_msgs.msg as spice_msgs
import spice_msgs.srv as spice_srvs


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

        self.robot_pos_publisher = self.create_publisher(spice_mapf_msgs.Position, "/robot_post", 10)
        self.paths_subscriber = self.create_subscription(spice_mapf_msgs.Paths, "mapf_paths", self.paths_cb, 10)

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.current_transform = None
        self.current_nav_goal: spice_mapf_msgs.Location = None

        self.joined_planner = False
        self.join_planner_future = None

        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        if not self.get_robot_transform():
            return
        
        if not self.joined_planner:
            self.try_join_planner()
            return
        
        self.publish_current_pos()    

    def try_join_planner(self):
        if self.join_planner_future is not None:
            return
        join_msg = spice_mapf_srvs.JoinPlanner.Request()
        join_msg.id = self.id
        x = int(round(self.current_transform.transform.translation.x, 0))
        y = int(round(self.current_transform.transform.translation.y, 0))
        join_msg.location.x = x
        join_msg.location.y = y
        while not self.join_planner_client.wait_for_service(1.0):
            self.get_logger().info(f'timeout on wait for service: "/join_planner"')
        self.join_planner_future = self.join_planner_client.call_async(join_msg)
        self.join_planner_future.add_done_callback(self.join_planner_cb)

    def join_planner_cb(self, future: Future):
        self.join_planner_future = None
        result: spice_mapf_srvs.JoinPlanner.Response = future.result()
        if result.success:
            self.joined_planner = True
        else:
            self.get_logger().warn(f'Failed to join planner, retrying')
            self.try_join_planner()
    
    def publish_current_pos(self):
        if self.current_transform is not None:
            pos_msg = spice_mapf_msgs.Position()
            pos_msg.x = self.current_transform.transform.translation.x
            pos_msg.y = self.current_transform.transform.translation.y
            pos_msg.id = self.id
            self.robot_pos_publisher.publish(pos_msg)

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
        # TODO: navigate to self.current_nav_goal using straight line planner
        pass
        
    def paths_cb(self, msg: spice_mapf_msgs.Paths):
        for path in msg.paths:
            if path.id == self.id:
                if self.current_nav_goal != path.location:
                    self.navigate_to_goal()
                    self.current_nav_goal = path.location
                return

    def on_nav_feedback(self, msg):
        pass
    
    def on_nav_done(self, msg):
        pass


if __name__ == '__main__':
    rclpy.init()
    test_node = MAPFNavigator()
    rclpy.spin(test_node)