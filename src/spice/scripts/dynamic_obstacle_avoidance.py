#!/usr/bin/env python3
...

import math

import os
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan
from spice_msgs.srv import GetRobots




class DynamicObstacleAvoidance(Node):


    def __init__(self):
        super().__init__('dynamic_obstacle_avoidance')
        self.ns = os.getenv("ROBOT_NAMESPACE")
        self.ns = 'polybot12' #debug

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cli_getRobots= self.create_client(GetRobots,'get_robots')
        #self.get_logger().info(f"{self.ns} : ns")
        topic = self.ns+'/scan'
        self.pub_obstacle = self.create_publisher(LaserScan,topic ,1)

        self.timer = self.create_timer(1.0, self.on_timer)



    def on_timer(self):
        cli_request = GetRobots.Request()

        cli_response = self.cli_getRobots.call_async(cli_request) # get list of active robots as type GetRobots.srv
        cli_response.add_done_callback(self.add_obstacle)


    
    def add_obstacle(self, cli_response: Future):
        #rclpy.spin_until_future_complete(self, cli_response)#,timeout_sec=0.9)
        to_frame_rel = self.ns + '_base_link'

        self.get_logger().info(f"responce: {cli_response.result().robots}")

        if cli_response.result().robots is None:
            self.get_logger().info(f"responce: No robots found")
            return

        for robot in cli_response.result().robots:
            self.get_logger().info(f"responce: found: {robot.id.id}")

            from_frame_rel = robot.id.id+'_base_link'


            
            try:
                t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())

            except TransformException as ex:
                self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                continue
        
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            deg = math.atan2(y,x)/math.pi*180

            laser_msg = LaserScan()
            laser_msg.header.frame_id = self.ns
            laser_msg.header.stamp = self.get_clock().now().to_msg()
            laser_msg.angle_min = deg-1
            laser_msg.angle_max = deg+1
            laser_msg.angle_increment = 1
            dist = math.sqrt(x ** 2 + y ** 2)
            laser_msg.ranges = [dist,dist,dist]
            laser_msg.scan_time = 0

            self.pub_obstacle.publish(laser_msg)

    



def main(args=None):
    rclpy.init(args=args)
    dynamic_obstacle_avoidance = DynamicObstacleAvoidance()
    rclpy.spin(dynamic_obstacle_avoidance)
    rclpy.shutdown()



if __name__ == '__main__':
    main()