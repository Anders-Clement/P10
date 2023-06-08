#!/usr/bin/python3

import serial
import time
import threading
import dataclasses
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

@dataclasses.dataclass
class RobotMeasurementPacket:
    imu_yaw_rate: float
    wheel_lin_vel: float
    wheel_angular_vel: float
    odom_x_pos: float
    odom_y_pos: float
    odom_yaw: float

    def __init__(self, msg: bytes):
        self.imu_yaw_rate, self.wheel_lin_vel, self.wheel_angular_vel, \
            self.odom_x_pos, self.odom_y_pos, self.odom_yaw = struct.unpack('ffffff', msg[1:25])       

class ArduinoSerial:
    def __init__(self, usb_port: str, message_callback) -> None:
        # open serial port with 1ms read timeout
        self.serial_port = serial.Serial(usb_port, 115200, timeout=0.001)
        self.buffer = bytes()
        self.stopped = False
        self.read_thread = threading.Thread(target=self.read_msgs, daemon=True)
        self.read_thread.start()
        self.message_callback = message_callback

    def stop(self):
        self.stopped = True

    def read_msgs(self):
        while not self.stopped:
            msg = self.check_for_msg()
            if msg is not None:
                self.message_callback(msg)

    def check_for_msg(self) -> None | RobotMeasurementPacket:
        INCOMING_PACKET_SIZE = 28
        # read up to a packet size
        incoming = self.serial_port.read(INCOMING_PACKET_SIZE)

        if len(incoming) > 0:
            self.buffer += incoming
        
        while len(self.buffer) >= INCOMING_PACKET_SIZE:
            if not (self.buffer[0] == 255 and self.buffer[INCOMING_PACKET_SIZE-3] == 2 \
                     and self.buffer[INCOMING_PACKET_SIZE-2] == 1 and self.buffer[INCOMING_PACKET_SIZE-1] == 0):
                self.buffer = self.buffer[1:]
                continue

            # we got a message
            message = RobotMeasurementPacket(self.buffer[:INCOMING_PACKET_SIZE])
            self.buffer = self.buffer[INCOMING_PACKET_SIZE:]
            return message
        
    def send_cmd_vel(self, cmd_vel: Twist):
        # start of packet
        cmd_msg = bytes([255])
        # cmd_vel_x, cmd_vel_yaw_rate
        cmd_msg += struct.pack('ff', cmd_vel.linear.x, cmd_vel.angular.z)
        # end of packet
        cmd_msg += bytes([2,1,0])
        self.serial_port.write(cmd_msg)

class PolybotBaseNode(Node):
    def __init__(self):
        super().__init__('polybot_base_node')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.arduino = ArduinoSerial('/dev/ttyACM0', self.incoming_msg_cb)

    def incoming_msg_cb(self, message: RobotMeasurementPacket):
        msg = Odometry()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.twist.twist.linear.x = message.wheel_lin_vel
        msg.twist.twist.angular.z = message.imu_yaw_rate
        msg.pose.pose.position.x = message.odom_x_pos
        msg.pose.pose.position.y = message.odom_y_pos
        q = quaternion_from_euler(0,0, message.odom_yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        self.odom_publisher.publish(msg)

    def cmd_vel_cb(self, msg: Twist):
        self.arduino.send_cmd_vel(msg)

if __name__ == "__main__":
    rclpy.init()
    node = PolybotBaseNode()
    rclpy.spin(node)
    node.arduino.stop()
    rclpy.shutdown() 