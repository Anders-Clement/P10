#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
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
        self.pub_obstacle = self.create_publisher(PoseArray, 'local_costmap/dynamic_obstacle', 1)

        self.timer_getRobots = self.create_timer(1.0, self.getRobots_timer)
        self.timer_updateObstacles = self.create_timer(1.0, self.updateObstacles_timer)

    def getRobots_timer(self):
        cli_request = GetRobotsByType.Request()
        cli_request.type = RobotType(type=RobotType.CARRIER_ROBOT)

        cli_response = self.cli_getRobots.call_async(cli_request) # get list of active robots as type GetRobots.srv
        while not self.cli_getRobots.wait_for_service(1.0):
            self.get_logger().info('Timeout waiting for service /get_robots')
        cli_response.add_done_callback(self.add_obstacle)

    def add_obstacle(self, cli_response: Future):
        self.obstacles = cli_response.result().robots

    def updateObstacles_timer(self):
        for robot in self.obstacles:
            if robot.id.id == self.ns:
                continue
            # self.get_logger().info(f"Adding obstacle on robot: {robot.id.id}")
            from_frame_rel = robot.id.id+'_base_link'
            
            try:
                t = self.tf_buffer.lookup_transform(self.to_frame_rel, from_frame_rel, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {self.to_frame_rel} to {from_frame_rel}: {ex}')
                continue
            
            robot_pose_array_msg = PoseArray()
            robot_pose_array_msg.header.stamp = self.get_clock().now().to_msg()
            robot_pose_array_msg.header.frame_id = from_frame_rel + 'base_link'

            robot_pose_msg = Pose()

            robot_pose_msg.position.x = t.transform.translation.x
            robot_pose_msg.position.y = t.transform.translation.y
            robot_pose_msg.position.z = t.transform.translation.z
            robot_pose_msg.orientation.w = t.transform.rotation.w
            robot_pose_msg.orientation.x = t.transform.rotation.x
            robot_pose_msg.orientation.y = t.transform.rotation.y
            robot_pose_msg.orientation.z = t.transform.rotation.z

            robot_pose_array_msg.poses.append(robot_pose_msg)
            ROBOT_RADIUS = 0.15
            ANGLE_INCREMENT = 6.28/8
            
            for i in range(8):
                robot_pose_msg = Pose()
                robot_pose_msg.position.x = t.transform.translation.x + (ROBOT_RADIUS*math.cos(ANGLE_INCREMENT*i))
                robot_pose_msg.position.y = t.transform.translation.y + (ROBOT_RADIUS*math.sin(ANGLE_INCREMENT*i))
                robot_pose_msg.position.z = t.transform.translation.z
                robot_pose_msg.orientation.w = t.transform.rotation.w
                robot_pose_msg.orientation.x = t.transform.rotation.x
                robot_pose_msg.orientation.y = t.transform.rotation.y
                robot_pose_msg.orientation.z = t.transform.rotation.z
                robot_pose_array_msg.poses.append(robot_pose_msg)

            self.pub_obstacle.publish(robot_pose_array_msg)

    

def main(args=None):
    rclpy.init(args=args)
    dynamic_obstacle_avoidance = DynamicObstacleAvoidance()
    rclpy.spin(dynamic_obstacle_avoidance)
    rclpy.shutdown()


if __name__ == '__main__':
    main()