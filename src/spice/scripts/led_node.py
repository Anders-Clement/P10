#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from spice_msgs.msg import LedOutput

import pigpio

class gpio():
    def __init__(self) -> None:
        self.ledPins = {'r': 26,
                        'g': 13,
                        'b': 19}
        self.pi = pigpio.pi()
        for pin in self.ledPins.values():
            self.pi.set_PWM_frequency(pin, 8000)
            self.pi.set_PWM_dutycycle(pin, 0)

    def set_rgb(self, output: LedOutput):
        self.pi.set_PWM_dutycycle(self.ledPins['r'], output.red*25)
        self.pi.set_PWM_dutycycle(self.ledPins['g'], output.green*25)
        self.pi.set_PWM_dutycycle(self.ledPins['b'], output.blue*25)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('led_node')
        self.led_sub = self.create_subscription(LedOutput, 'led_output', self.led_cb, 10)
        self.gpio = gpio()

    def led_cb(self, msg: LedOutput):
        self.get_logger().info(f'Setting LEDs: \n{msg}')
        self.gpio.set_rgb(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()