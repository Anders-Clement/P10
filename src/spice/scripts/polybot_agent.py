#!/usr/bin/python3

import serial
import time
import threading
import dataclasses
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

@dataclasses.dataclass
class RobotMeasurementPacket:
    yaw_rate: float
    linear_vel: float
    angular_vel: float

    def __init__(self, msg: bytes):
        self.yaw_rate, self.linear_vel, self.angular_vel = struct.unpack('fff', msg[1:13])       

class ArduinoSerial:
    def __init__(self, usb_port: str, message_callback) -> None:
        # open serial port with 1ms read timeout
        self.serial_port = serial.Serial(usb_port, 115200, timeout=1.0)
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
        # read up to a packet size
        incoming = self.serial_port.read(16)

        if len(incoming) > 0:
            self.buffer += incoming
        
        while len(self.buffer) >= 16:
            if not (self.buffer[0] == 255 and self.buffer[13] == 2 \
                     and self.buffer[14] == 1 and self.buffer[15] == 0):
                self.buffer = self.buffer[1:]
                continue

            # we got a message
            message = RobotMeasurementPacket(self.buffer[:16])
            self.buffer = self.buffer[16:]
            return message
        
    def send_cmd_vel(self, cmd_vel: Twist):
        # start of packet
        cmd_msg = bytes([255])
        # cmd_vel_x, cmd_vel_yaw_rate, unused
        cmd_msg += struct.pack('fff', cmd_vel.linear.x, cmd_vel.angular.z, 0.0)
        # end of packet
        cmd_msg += bytes([2,1,0])
        self.serial_port.write(cmd_msg)


class PolybotBaseNode(Node):
    def __init__(self):
        super().__init__('polybot_base_node')
        self.arduino = ArduinoSerial('/dev/ttyACM0', self.incoming_msg_cb)
        self.odom_publisher = self.create_publisher(TwistStamped, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

    def incoming_msg_cb(self, message: RobotMeasurementPacket):
        msg = TwistStamped()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.twist.linear.x = message.linear_vel
        msg.twist.angular.z = message.angular_vel
        self.odom_publisher.publish(msg)

    def check_for_msg(self):
        message = self.arduino.check_for_msg()
        if message is not None:
            msg = TwistStamped()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.twist.linear.x = message.linear_vel
            msg.twist.angular.z = message.angular_vel
            self.odom_publisher.publish(msg)

    def cmd_vel_cb(self, msg: Twist):
        self.arduino.send_cmd_vel(msg)

if __name__ == "__main__":
    rclpy.init()
    node = PolybotBaseNode()
    rclpy.spin(node)
    node.arduino.stop()
    rclpy.shutdown() 