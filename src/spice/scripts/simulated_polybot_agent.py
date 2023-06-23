#!/usr/bin/python3

import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Transform
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler

class Robot():
    def __init__(self, x = 0, y = 0, theta = 0, max_simulation_dt=0.01) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.max_simulation_dt = max_simulation_dt
        self.time = 0

    def simulate(self, cmd_vel: Twist, dt: float):
        start_time = self.time
        running = True
        while running:
            remaining_sim_time = start_time+dt - self.time
            if remaining_sim_time > self.max_simulation_dt:
                remaining_sim_time = self.max_simulation_dt
            else:
                running = False

            self.x += remaining_sim_time*cmd_vel.linear.x*math.cos(self.theta)
            self.y += remaining_sim_time*cmd_vel.linear.x*math.sin(self.theta)
            self.theta += remaining_sim_time*cmd_vel.angular.z
            self.time += remaining_sim_time

        return self.get_transform()

    def get_transform(self) -> Transform:
        transform = Transform()

        transform.translation.x = self.x
        transform.translation.y = self.y
        transform.translation.z = 0.0
        
        x,y,z,w = quaternion_from_euler(0,0,self.theta)
        transform.rotation.x = x
        transform.rotation.y = y
        transform.rotation.z = z
        transform.rotation.w = w
        
        return transform

class PolybotSimulator(Node):
    def __init__(self):
        super().__init__('polybot_simulator_node')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.UPDATE_RATE = 0.1
        self.robot = Robot(
            x=self.get_parameter('initial_x').value,
            y=self.get_parameter('initial_y').value,
            theta=self.get_parameter('initial_theta').value,
        )
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, qos_best_effort)
        self.tf_publisher = self.create_publisher(TFMessage, 'tf', 1)
        self.pub_tf_timer = self.create_timer(self.UPDATE_RATE, self.publish_data)

        self.current_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()


    def publish_data(self):
        time_since_last_cmd_vel = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds*1e-9  
        if time_since_last_cmd_vel > 0.5:
            self.current_cmd_vel = Twist()
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform = self.robot.simulate(self.current_cmd_vel, self.UPDATE_RATE)

        tf_msg = TFMessage()
        tf_msg.transforms.append(transform)
        self.tf_publisher.publish(tf_msg)

    def cmd_vel_cb(self, msg: Twist):
        self.current_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

if __name__ == "__main__":
    rclpy.init()
    node = PolybotSimulator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown() 