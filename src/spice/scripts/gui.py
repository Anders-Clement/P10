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
        self.root.geometry('800x600')
        self.root.title('Robot State Viewer')
        self.tab_control = ttk.Notebook(self.root)
        self.robot_state_tab = ttk.Frame(self.tab_control)
        self.queue_param_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.robot_state_tab, text="Robot state")
        self.tab_control.add(self.queue_param_tab, text="Queue costmap Params")
        self.tab_control.pack(expand=1, fill='both')
        self.reset_robot_state_tab()
        self.reset_queue_param_tab()

    def reset_robot_state_tab(self):
        if self.robot_state_tab:
           for child in self.robot_state_tab.winfo_children():
               child.destroy()
        else:
            self.robot_state_tab = ttk.Frame(self.tab_control)
            self.tab_control.add(self.robot_state_tab, text="Robot state")
            self.tab_control.pack(expand=1, fill='both')
        
        self.robot_state_label = ttk.Label(self.robot_state_tab, text="Number of robots: ?", padding=5).grid(column=0, row=0, rowspan=2)
    
    def reset_queue_param_tab(self):
        if self.queue_param_tab:
            pass
        else:
            self.queue_param_tab = ttk.Frame(self.tab_control)
            self.tab_control.add(self.queue_param_tab, text="Queue costmap Params")
            self.tab_control.pack(expand=1, fill='both')
        
        
        self.ypadding = 20
        self.coloumn_offset = 45
        self.cloumn_start = 0
        #self.robot_state_tab.grid
        ttk.Label(self.queue_param_tab, text="work_cell_rep_slope",padding=10).grid(column=self.cloumn_start,row=0,rowspan=2, pady=self.ypadding)
        self.work_cell_rep_slope = tk.Scale(self.queue_param_tab, from_=0, to=1, resolution=0.01, orient='horizontal').grid(column=self.cloumn_start, row=1, rowspan=2, pady=10)

        ttk.Label(self.queue_param_tab, text="carrier_bot_rep_slope",padding=10).grid(column=self.cloumn_start,row=2,rowspan=2, pady=self.ypadding)
        self.carrier_bot_rep_slope = tk.Scale(self.queue_param_tab, from_=0, to=1,resolution=0.01, orient='horizontal').grid(column=self.cloumn_start, row=3, rowspan=2, pady=10)

        ttk.Label(self.queue_param_tab, text="wall_rep_slope",padding=10).grid(column=self.cloumn_start,row=4,rowspan=2, pady=self.ypadding)
        self.wall_rep_slope = tk.Scale(self.queue_param_tab, from_=0, to=1,resolution=0.01, orient='horizontal').grid(column=self.cloumn_start, row=5, rowspan=2, pady=10)


        ttk.Label(self.queue_param_tab, text="work_cell_att_slope",padding=10).grid(column=self.cloumn_start+self.coloumn_offset,row=0,rowspan=2, pady=self.ypadding)
        self.work_cell_att_slope = tk.Scale(self.queue_param_tab, from_=0, to=1, resolution=0.01,orient='horizontal').grid(column=self.cloumn_start+self.coloumn_offset, row=1, rowspan=2, pady=10)

        ttk.Label(self.queue_param_tab, text="queue_att_slope",padding=10).grid(column=self.cloumn_start+self.coloumn_offset,row=2,rowspan=2, pady=self.ypadding)
        self.queue_att_slope = tk.Scale(self.queue_param_tab, from_=0, to=1, resolution=0.01,orient='horizontal').grid(column=self.cloumn_start+self.coloumn_offset, row=3, rowspan=2, pady=10)

        ttk.Label(self.queue_param_tab, text="queue_rep_slope",padding=10).grid(column=self.cloumn_start+self.coloumn_offset,row=4,rowspan=2, pady=self.ypadding)
        self.queue_rep_slope = tk.Scale(self.queue_param_tab, from_=0, to=1, resolution=0.01,orient='horizontal').grid(column=self.cloumn_start+self.coloumn_offset, row=5, rowspan=2, pady=10)
        

        
        
        ttk.Label(self.queue_param_tab, text="plan_rep_slope",padding=10).grid(column=self.cloumn_start+2*self.coloumn_offset,row=0,rowspan=2, pady=self.ypadding)
        self.plan_rep_slope = tk.Scale(self.queue_param_tab, from_=0, to=1,resolution=0.01, orient='horizontal').grid(column=self.cloumn_start+2*self.coloumn_offset, row=1, rowspan=2, pady=10)

        ttk.Label(self.queue_param_tab, text="q_max_vel",padding=10).grid(column=self.cloumn_start+2*self.coloumn_offset,row=2,rowspan=2, pady=self.ypadding)
        self.q_max_vel = tk.Scale(self.queue_param_tab, from_=0, to=5,resolution=0.1, orient='horizontal').grid(column=self.cloumn_start+2*self.coloumn_offset, row=3, rowspan=2, pady=10)

        ttk.Label(self.queue_param_tab, text="min_move_dist",padding=10).grid(column=self.cloumn_start+2*self.coloumn_offset,row=4,rowspan=2, pady=self.ypadding)
        self.min_move_dist = tk.Scale(self.queue_param_tab, from_=0, to=50,resolution=1, orient='horizontal').grid(column=self.cloumn_start+2*self.coloumn_offset, row=5, rowspan=2, pady=10)


        ttk.Label(self.queue_param_tab, text="",padding=10).grid(column=self.cloumn_start+self.coloumn_offset,row=6,rowspan=2, pady=self.ypadding)

    def loop(self):
        #self.root.after(10, self.spin)
        self.root.mainloop()

    def spin(self):
        self.root.after(10, self.spin)
        rclpy.spin_once(self, timeout_sec=0.05)

    def get_robots_timer_cb(self):
        future = self.get_robots_client.call_async(sm_srv.GetRobots.Request())
        future.add_done_callback(self.get_robots_cb)

    def get_robots_cb(self, future: rclpy.Future):
        result: sm_srv.GetRobots.Response = future.result()
        if self.robot_manager.check_new_robots(result.robots):
            robot_info = self.robot_manager.get_robot_infos()
            self.reset_robot_state_tab()
            self.robot_state_label = ttk.Label(self.robot_state_tab, text='Number of robots: ' + str(len(robot_info)), padding=5).grid(column=0, row=0, rowspan=2)
            #+ str(len(robot_info)))
            for i, info in enumerate(robot_info):
                ttk.Label(self.robot_state_tab, text=info.name + ":", padding=5).grid(column=3, row=i)
                ttk.Label(self.robot_state_tab, text=info.state, padding=5).grid(column=4, row=i)

        
if __name__ == '__main__':
    rclpy.init()
    ui = Ui()
    ui.loop()
