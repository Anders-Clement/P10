#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from robot_state import RobotStateManager

class RobotStateManagerNode(Node):

    def __init__(self):
        super().__init__('robot_state_manager_node')
        self.sm = RobotStateManager(self)

def main(args=None):
    rclpy.init(args=args)

    robot_state_manager_node = RobotStateManagerNode()

    rclpy.spin(robot_state_manager_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_state_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()