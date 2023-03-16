#!/usr/bin/python3

import enum

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
import dataclasses

import spice_msgs.srv as sm_srv
import spice_msgs.msg as sm_msg


@dataclasses.dataclass
class RobotInfo:
    name: str
    state: str

class RobotManager:
    def __init__(self) -> None:
        self.robots: list[sm_msg.Robot] = list()

    def check_new_robots(self, robots: list[sm_msg.Robot]):
        self.robots = robots
        return True

    def get_robot_infos(self) -> list[RobotInfo]:
        return [RobotInfo(robot.id.id, robot.robot_state.internal_state) for robot in self.robots]


class Ui(Node):
    def __init__(self) -> None:
        super().__init__('gui_node')
        self.init_gui()
        self.init_ros()
        self.robot_manager = RobotManager()
        self.labels: list[tk.Label] = list()

    def init_ros(self):
        self.get_robots_client = self.create_client(sm_srv.GetRobots, '/get_robots')
        while not self.get_robots_client.wait_for_service(1.0):
            self.get_logger().info('Timeout waiting for service /get_robots')
        self.get_robots_timer = self.create_timer(1.0, self.get_robots_timer_cb)

    def init_gui(self):
        self.root = tk.Tk()
        self.root.geometry('600x800')
        self.root.title('Robot State Viewer')
        self.frame = None
        self.reset_gui_frame()

    def reset_gui_frame(self):
        if self.frame:
            self.frame.destroy()
        self.frame = ttk.Frame(self.root, padding=10)
        self.frame.grid()
        self.num_robots_label = ttk.Label(self.frame, text="Number of robots: ?", padding=5)
        self.num_robots_label.grid(column=0, row=0, rowspan=2)

    def loop(self):
        self.root.after(50, self.spin)
        self.root.mainloop()

    def spin(self):
        self.root.after(50, self.spin)
        rclpy.spin_once(self, timeout_sec=0.05)

    def get_robots_timer_cb(self):
        future = self.get_robots_client.call_async(sm_srv.GetRobots.Request())
        future.add_done_callback(self.get_robots_cb)

    def get_robots_cb(self, future: rclpy.Future):
        result: sm_srv.GetRobots.Response = future.result()
        if self.robot_manager.check_new_robots(result.robots):
            robot_info = self.robot_manager.get_robot_infos()
            self.reset_gui_frame()
            self.num_robots_label['text'] = 'Number of robots: ' + str(len(robot_info))
            for i, info in enumerate(robot_info):
                ttk.Label(self.frame, text=info.name + ":", padding=5).grid(column=0, row=i)
                ttk.Label(self.frame, text=info.state, padding=5).grid(column=1, row=i)

        
if __name__ == '__main__':
    rclpy.init()
    ui = Ui()
    ui.loop()
