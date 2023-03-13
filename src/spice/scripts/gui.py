#!/usr/bin/python3

import enum

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
import dataclasses

import spice_msgs.srv as sm_srv
import spice_msgs.msg as sm_msg

class ROBOT_STATE(enum.IntEnum):
    MR_READY_FOR_JOB = sm_msg.RobotState.MR_READY_FOR_JOB
    MR_PROCESSING_JOB = sm_msg.RobotState.MR_PROCESSING_JOB
    WC_READY_FOR_ROBOTS = sm_msg.RobotState.WC_READY_FOR_ROBOTS
    STARTUP = sm_msg.RobotState.STARTUP
    ERROR = sm_msg.RobotState.ERROR

class CARRIER_ROBOT_STATE(enum.IntEnum):
    STARTUP = 0
    READY_FOR_JOB = 1
    FIND_WORKCELL = 2
    MOVING = 3
    REGISTER_WORK = 4
    WAIT_IN_QUEUE = 5
    READY_FOR_PROCESS = 6
    PROCESS_DONE =7
    EXIT_WORKCELL = 8
    ERROR = 9

class WORK_CELL_ROBOT_STATE(enum.IntEnum):
    STARTUP = 0
    READY_FOR_ROBOT = 1
    ROBOT_ENTERING = 2
    PROCESSING = 3
    ROBOT_EXITING = 4
    NUM_STATES = 5

@dataclasses.dataclass
class RobotInfo:
    name: str
    state: str

class RobotManager:
    def __init__(self) -> None:
        self.robots: list[sm_msg.Robot] = list()

    def check_new_robots(self, robots: list[sm_msg.Robot]):
        # check if robots have been deregistered:
        new_robot_ids = [robot.id.id for robot in robots]
        robots_to_delete = [robot for robot in self.robots if robot.id.id not in new_robot_ids]
        self.robots = [robot for robot in self.robots if robot not in robots_to_delete]

        # add new robots:
        known_ids = [robot.id.id for robot in self.robots]
        new_robots = [robot for robot in robots if robot.id.id not in known_ids]
        self.robots += new_robots
        
        return len(robots_to_delete) > 0 or len(new_robots) > 0

    def get_robot_infos(self) -> list[RobotInfo]:
        infos = list()
        for robot in self.robots:
            if robot.id.robot_type.type == sm_msg.RobotType.CARRIER_ROBOT:
                internal_state = CARRIER_ROBOT_STATE(robot.robot_state.internal_state)
            else:
                internal_state = WORK_CELL_ROBOT_STATE(robot.robot_state.internal_state)
            infos.append(RobotInfo(robot.id.id, internal_state))
        return infos


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
            self.get_logger().debug('Got a change in robots or their states')
            robot_info = self.robot_manager.get_robot_infos()
            self.reset_gui_frame()

            for i, info in enumerate(robot_info):
                name_label = ttk.Label(self.frame, text=info.name + ":", padding=5).grid(column=0, row=i)
                state_label = ttk.Label(self.frame, text=info.state, padding=5).grid(column=1, row=i)
                self.labels.append(name_label)
                self.labels.append(state_label)

        
if __name__ == '__main__':
    rclpy.init()
    ui = Ui()
    ui.loop()
