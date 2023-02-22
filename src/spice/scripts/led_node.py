#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from spice_msgs.msg import LedOutput, RobotStateTransition, RobotState, LedOutput

import pigpio

class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.led_sub = self.create_subscription(LedOutput, 'led_output', self.led_cb, 10)
        self.robot_state_transition_event_sub = self.create_subscription(
            RobotStateTransition,
            'robot_state_transition_event',
            self.robot_state_transition_event_cb,
            10)
        self.ledPins = {'r': 26,
                        'g': 13,
                        'b': 19}
        self.pi = pigpio.pi()
        for pin in self.ledPins.values():
            self.pi.set_PWM_frequency(pin, 8000)
        self.led_cb(LedOutput(red=True, blue=False, green=False))

    def robot_state_transition_event_cb(self, msg: RobotStateTransition) -> None:
        if msg.new_state.state == RobotState.STARTUP:
            self.led_cb(LedOutput(red=True, blue=False, green=True))
        elif msg.new_state.state == RobotState.READY_FOR_JOB:
            self.led_cb(LedOutput(red=False, blue=False, green=True))
        elif msg.new_state.state == RobotState.MOVING:
            self.led_cb(LedOutput(red=False, blue=True, green=True))
        elif msg.new_state.state == RobotState.PROCESSING:
            self.led_cb(LedOutput(red=True, blue=True, green=True))
        elif msg.new_state.state == RobotState.ERROR:
            self.led_cb(LedOutput(red=True, blue=False, green=False))
        

    def led_cb(self, msg: LedOutput):
        self.get_logger().info(f'Setting LEDs: \n{msg}')
        self.pi.set_PWM_dutycycle(self.ledPins['r'], msg.red*25)
        self.pi.set_PWM_dutycycle(self.ledPins['g'], msg.green*25)
        self.pi.set_PWM_dutycycle(self.ledPins['b'], msg.blue*25)


def main(args=None):
    rclpy.init(args=args)

    led_node = LedNode()

    rclpy.spin(led_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    led_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()