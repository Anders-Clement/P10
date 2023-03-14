#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan
from spice_msgs.msg import RobotType
from spice_msgs.srv import GetRobotsByType


class DynamicObstacleAvoidance(Node):

    obstacles = []

    def __init__(self):
        super().__init__('dynamic_obstacle_avoidance')
        self.ns = os.getenv("ROBOT_NAMESPACE")
        self.to_frame_rel = self.ns + '_base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cli_getRobots= self.create_client(GetRobotsByType,'/get_robots_by_type')
        self.pub_obstacle = self.create_publisher(LaserScan, 'scan', 1)

        self.timer_getRobots = self.create_timer(1.0, self.getRobots_timer)
        self.timer_updateObstacles = self.create_timer(0.1, self.updateObstacles_timer)

    def getRobots_timer(self):
        cli_request = GetRobotsByType.Request()
        cli_request.type = RobotType(type=RobotType.CARRIER_ROBOT)

        cli_response = self.cli_getRobots.call_async(cli_request) # get list of active robots as type GetRobots.srv
        cli_response.add_done_callback(self.add_obstacle)

    def add_obstacle(self, cli_response: Future):
        self.obstacles = cli_response.result().robots

    def updateObstacles_timer(self):
        for robot in self.obstacles:
            if robot.id.id == self.ns:
                continue
            #self.get_logger().info(f"Doing: {robot.id.id}")
            from_frame_rel = robot.id.id+'_base_link'
            
            try:
                t = self.tf_buffer.lookup_transform(self.to_frame_rel, from_frame_rel, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {self.to_frame_rel} to {from_frame_rel}: {ex}')
                continue
            
            
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            angle_to_robot = math.atan2(robot_y,robot_x)
            distance_to_robot = math.sqrt(robot_x ** 2 + robot_y ** 2)
            ROBOT_RADIUS = 0.15
            # to get points at the sides of the other robot
            angle_increment = math.atan2(0.01, distance_to_robot)

            laser_msg = LaserScan()
            frame_id = 'base_link'
            laser_msg.header.frame_id = frame_id
            laser_msg.header.stamp = self.get_clock().now().to_msg()
            laser_msg.angle_min = angle_to_robot-angle_increment
            laser_msg.angle_max = angle_to_robot+angle_increment
            laser_msg.angle_increment = angle_increment
            laser_msg.time_increment = 0.01
            laser_msg.scan_time = 1.0
            laser_msg.range_min = 0.0
            laser_msg.range_max = 10.0
            laser_msg.ranges = [distance_to_robot-0.25,distance_to_robot,distance_to_robot-0.50]
            laser_msg.intensities = []

            self.pub_obstacle.publish(laser_msg)
    

def main(args=None):
    rclpy.init(args=args)
    dynamic_obstacle_avoidance = DynamicObstacleAvoidance()
    rclpy.spin(dynamic_obstacle_avoidance)
    rclpy.shutdown()


if __name__ == '__main__':
    main()