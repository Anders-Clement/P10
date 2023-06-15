#!/usr/bin/python3

import serial
import time
import threading
import dataclasses
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler

@dataclasses.dataclass
class RobotMeasurementPacket:
    imu_yaw_rate: float
    wheel_lin_vel: float
    wheel_angular_vel: float
    odom_x_pos: float
    odom_y_pos: float
    odom_yaw: float
    cmd_vel_x: float
    cmd_vel_z: float
    BYTES_PER_FLOAT = 4
    NUM_FLOATS = 8

    def __init__(self, msg: bytes):
        
        self.imu_yaw_rate, self.wheel_lin_vel, self.wheel_angular_vel, \
            self.odom_x_pos, self.odom_y_pos, self.odom_yaw, self.cmd_vel_x, self.cmd_vel_z = struct.unpack(
            'f'*self.NUM_FLOATS, msg[1:self.BYTES_PER_FLOAT*self.NUM_FLOATS+1])       

class ArduinoSerial:
    def __init__(self, usb_port: str, message_callback, logger) -> None:
        # open serial port with 1ms read timeout
        self.serial_port = serial.Serial(usb_port, 0, timeout=0.01)
        self.buffer = bytes()
        self.stopped = False
        self.read_thread = threading.Thread(target=self.read_msgs, daemon=True)
        self.read_thread.start()
        self.message_callback = message_callback
        self.logger = logger

    def stop(self):
        self.stopped = True

    def read_msgs(self):
        while not self.stopped:
            msg = self.check_for_msg()
            if msg is not None:
                # self.logger.info(f'Msg: {msg}')
                self.message_callback(msg)

    def check_for_msg(self) -> None | RobotMeasurementPacket:
        INCOMING_PACKET_SIZE = 4 + RobotMeasurementPacket.NUM_FLOATS*RobotMeasurementPacket.BYTES_PER_FLOAT
        # read up to a packet size
        incoming = ""
        try:
            incoming = self.serial_port.read(INCOMING_PACKET_SIZE)
        except serial.SerialException as e:
            # something maybe in OS is reading/writing to serial port sometimes
            # catch exception instead of crashing. 
            self.logger.warn(f'Got serial exception: {e}')
            pass

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
        # self.logger.info(f'Sending bytes: {cmd_msg}')
        self.serial_port.write(cmd_msg)

class PolybotBaseNode(Node):
    def __init__(self):
        super().__init__('polybot_base_node')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.tf_publisher = self.create_publisher(TFMessage, 'tf', 10)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').value
        self.get_logger().info(f'Connecting to serial port: {port}')

        self.arduino = ArduinoSerial(port, self.incoming_msg_cb, self.get_logger())

        self.num_incoming_msgs = -1

    def incoming_msg_cb(self, message: RobotMeasurementPacket):
        self.num_incoming_msgs += 1
        # only publish odom and tf at 10 Hz
        # if self.num_incoming_msgs % 5 != 0:
        #     return
        odom_msg = Odometry()
        now = self.get_clock().now()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.twist.twist.linear.x = message.wheel_lin_vel
        odom_msg.twist.twist.angular.z = message.imu_yaw_rate
        odom_msg.pose.pose.position.x = message.odom_x_pos
        odom_msg.pose.pose.position.y = message.odom_y_pos
        q = quaternion_from_euler(0,0, message.odom_yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        self.odom_publisher.publish(odom_msg)
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header = odom_msg.header
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = message.odom_x_pos
        transform.transform.translation.y = message.odom_y_pos
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        tf_msg.transforms.append(transform)
        self.tf_publisher.publish(tf_msg)

    def cmd_vel_cb(self, msg: Twist):
        self.arduino.send_cmd_vel(msg)

if __name__ == "__main__":
    rclpy.init()
    node = PolybotBaseNode()
    rclpy.spin(node)
    node.arduino.stop()
    rclpy.shutdown() 