#!/usr/bin/python3

import enum

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

import spice_msgs.srv as sm_srv
import spice_msgs.msg as sm_msg
import geometry_msgs.msg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


import time


class Ui(Node):
    def __init__(self) -> None:
        super().__init__('workcell_editor_node')
        self.init_gui()
        self.labels: list[tk.Label] = list()
        self.update_ros()
       
       

    def init_ros(self):

        self.selected_workcell = ""
        self.selected_workcell_pos = geometry_msgs.msg.TransformStamped()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.delete_workcell_cli = self.create_client(sm_srv.DeleteWorkCell, 'delete_workcell')
        self.create_workcell_cli = self.create_client(sm_srv.CreateWorkCell, 'create_workcell')
        
        self.get_robots_client = self.create_client(sm_srv.GetRobotsByType, '/get_robots_by_type')
        self.goal_pose_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        while not self.get_robots_client.wait_for_service(1.0):
            self.get_logger().info('Timeout waiting for service /get_robots')
        while not self.delete_workcell_cli.wait_for_service(timeout_sec=1.0):
             self.get_logger().info('service not available, waiting again...')
        while not self.create_workcell_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_workcells_timer_cb()
        self.get_robots_timer = self.create_timer(1.0, self.get_workcells_timer_cb)
        
        

    def init_gui(self):

        self.root = tk.Tk()
        self.root.geometry('800x600')
        self.root.title('Workcell editor')
        self.tab_control = ttk.Notebook(self.root)  
        self.workcell_tab = ttk.Frame(self.tab_control)
        self.create_workcell_text = "Create Workcell"
        self.workcells = {self.create_workcell_text : geometry_msgs.msg.TransformStamped()}
        self.deleted_workcells = []
        self.workcell_string_to_type = {"WORK_CELL_BACK_COVER" : sm_msg.RobotType.WORK_CELL_BACK_COVER ,"WORK_CELL_DRILL" : sm_msg.RobotType.WORK_CELL_DRILL,"WORK_CELL_FUSES" : sm_msg.RobotType.WORK_CELL_FUSES,"WORK_CELL_TOP" : sm_msg.RobotType.WORK_CELL_TOP}

        self.add_workcell_req = sm_srv.CreateWorkCell.Request()
        self.add_workcell_req.id.robot_type.type = sm_msg.RobotType.WORK_CELL_ANY
        
        self.vcmdxt = (self.workcell_tab.register(self.validate_entry))
        self.vcmdyt = (self.workcell_tab.register(self.validate_entry))
        self.vcmdzt = (self.workcell_tab.register(self.validate_entry))
        self.vcmdxq = (self.workcell_tab.register(self.validate_entry))
        self.vcmdyq = (self.workcell_tab.register(self.validate_entry))
        self.vcmdzq = (self.workcell_tab.register(self.validate_entry))
        self.vcmdwq = (self.workcell_tab.register(self.validate_entry))

        self.svxt = tk.StringVar()
        self.svyt = tk.StringVar()
        self.svzt = tk.StringVar()
        self.svxq = tk.StringVar()
        self.svyq = tk.StringVar()
        self.svzq = tk.StringVar()
        self.svwq = tk.StringVar()

        self.sv_add_workcell_id = tk.StringVar()

        self.svxt.trace_add("write", lambda *_, var="xt": self.entry_changed(var))
        self.svyt.trace_add("write", lambda *_, var="yt": self.entry_changed(var))
        self.svzt.trace_add("write", lambda *_, var="zt": self.entry_changed(var))
        self.svxq.trace_add("write", lambda *_, var="xq": self.entry_changed(var))
        self.svyq.trace_add("write", lambda *_, var="yq": self.entry_changed(var))
        self.svzq.trace_add("write", lambda *_, var="zq": self.entry_changed(var))
        self.svwq.trace_add("write", lambda *_, var="wq": self.entry_changed(var))
        self.sv_add_workcell_id.trace_add("write", lambda *_, var="": self.add_workcell_id_entry_changed())
        
        #self.sv_add_workcell_id.trace_add("write", self.add_workcell_id_entry_changed) ##########################
        
        tk.Label(self.workcell_tab, text="Translation").grid(row=1, column=8,sticky="ew", columnspan=7)
        tk.Label(self.workcell_tab, text="x").grid(row=2, column=0)
        self.xt = tk.Entry(self.workcell_tab, textvariable=self.svxt)
        self.xt.grid(row=2, column=1, columnspan=3)
        
        tk.Label(self.workcell_tab, text="y").grid(row=2, column=5)
        self.yt = tk.Entry(self.workcell_tab, textvariable=self.svyt)        
        self.yt.grid(row=2, column=6, columnspan=3)

        tk.Label(self.workcell_tab, text="z").grid(row=2, column=10)
        self.zt = tk.Entry(self.workcell_tab, textvariable=self.svzt)        
        self.zt.grid(row=2, column=11, columnspan=3)

        tk.Label(self.workcell_tab, text="Rotation").grid(row=3, column=8, sticky="ew", columnspan=7)
        tk.Label(self.workcell_tab, text="x").grid(row=4, column=0)
        self.xq = tk.Entry(self.workcell_tab, textvariable=self.svxq)
        self.xq.grid(row=4, column=1, columnspan=3)
        
        tk.Label(self.workcell_tab, text="y").grid(row=4, column=5)
        self.yq = tk.Entry(self.workcell_tab, textvariable=self.svyq)        
        self.yq.grid(row=4, column=6, columnspan=3)

        tk.Label(self.workcell_tab, text="z").grid(row=4, column=10)
        self.zq = tk.Entry(self.workcell_tab, textvariable=self.svzq)        
        self.zq.grid(row=4, column=11,columnspan=3)

        tk.Label(self.workcell_tab, text="w").grid(row=4, column=15)
        self.wq = tk.Entry(self.workcell_tab, textvariable=self.svwq)        
        self.wq.grid(row=4, column=16,columnspan=3)

        
        ###ADD workcell id entry
        self.add_workcell_label = tk.Label(self.workcell_tab, text="New Workcell Id")
        self.add_workcell_label.grid(row=6, column=7, columnspan=3)
        self.add_workcell_id = tk.Entry(self.workcell_tab, textvariable=self.sv_add_workcell_id)        
        self.add_workcell_id.grid(row=6, column=10,columnspan=4)

        self.publish_bt = tk.Button(self.workcell_tab, text="Publish", command= self.publish_position)
        self.publish_bt.grid(row=13, sticky="ew", column=13,columnspan=3)

        self.add_bt = tk.Button(self.workcell_tab, text="Add Workcell", command= self.add_workcell)
        self.add_bt.grid(row=13, sticky="ew", column=10,columnspan=3)

        self.delete_bt = tk.Button(self.workcell_tab, text="Delete Workcell", command= self.delete_workcell)
        self.delete_bt.grid(row=13, sticky="ew", column=7,columnspan=3)

        self.add_workcell_type_list = ["WORK_CELL_BACK_COVER","WORK_CELL_DRILL","WORK_CELL_FUSES","WORK_CELL_TOP"]
        self.add_workcell_type_value_inside = tk.StringVar(self.root)
        self.add_workcell_type_value_inside.trace("w", self.OptionMenu_addworkcelltype_cb)
        self.add_workcell_type_options = ttk.OptionMenu(self.workcell_tab, self.add_workcell_type_value_inside, self.add_workcell_type_list[0],*self.add_workcell_type_list)
        self.add_workcell_type_options.grid(row=8, column=8, sticky="nsew", columnspan=7)


        self.options_list = [self.create_workcell_text]
        self.value_inside = tk.StringVar(self.root)
        self.value_inside.trace("w", self.optionMenuCallback)
        self.value_inside.set(self.create_workcell_text)
        self.options = ttk.OptionMenu(self.workcell_tab, self.value_inside, *self.options_list)
        self.options.grid(row=0, column=8, sticky="nsew", columnspan=7)
        self.options.grid(row=0, column=8, sticky="nsew", columnspan=7)
                
        self.options.grid(row=0, column=8, sticky="nsew", columnspan=7) 
                
        self.tab_control.add(self.workcell_tab, text="workcell editor")
        self.tab_control.pack(expand=1, fill='both')

        self.update_optionslist = False

        self.init_ros()


    def validate_entry(self, P):
        print(P)
        if str.isdigit(P) or str(P) == "":
        #     if P == "":
        #         value = 0
        #     else:
        #         value = float(P)
        #     print("hello2")
        #     if(arg == "xt"):
        #         self.selected_workcell_pos.transform.translation.x = value
        #     elif(arg == "yt"):
        #         self.selected_workcell_pos.transform.translation.y = value
        #     elif(arg == "zt"):
        #         self.selected_workcell_pos.transform.translation.z = value
        #     elif(arg == "xq"):
        #         self.selected_workcell_pos.transform.rotation.x = value
        #     elif(arg == "yq"):
        #         self.selected_workcell_pos.transform.rotation.y = value
        #     elif(arg == "zq"):
        #         self.selected_workcell_pos.transform.rotation.z = value
        #     elif(arg == "wq"):
        #         self.selected_workcell_pos.transform.rotation.w = value

            return True
        else:
            print("ONLY FLOATs ALOOWED >:(")
            return False
        
    def entry_changed(self, arg):
        if(arg == "xt"):
            if self.svxt.get() == "":
                #self.selected_workcell_pos.transform.translation.x = 0.0
                return
            try:
                self.selected_workcell_pos.transform.translation.x = float(self.svxt.get())
            except:
                self.xt.delete(0, tk.END)
                self.xt.insert(0,float(self.selected_workcell_pos.transform.translation.x))
        
        elif(arg == "yt"):
            if self.svyt.get() == "":
                #self.selected_workcell_pos.transform.translation.y = 0.0
                return
            try:
                self.selected_workcell_pos.transform.translation.y = float(self.svyt.get())
            except:
                self.yt.delete(0, tk.END)
                self.yt.insert(0,float(self.selected_workcell_pos.transform.translation.y))
        
        elif(arg == "zt"):
            if self.svzt.get() == "":
                #self.selected_workcell_pos.transform.translation.z = 0.0
                return
            try:
                self.selected_workcell_pos.transform.translation.z = float(self.svzt.get())
            except:
                self.zt.delete(0, tk.END)
                self.zt.insert(0,float(self.selected_workcell_pos.transform.translation.z))
        
        elif(arg == "xq"):
            if self.svxq.get() == "":
                #self.selected_workcell_pos.transform.rotation.x = 0.0
                return
            try:
                self.selected_workcell_pos.transform.rotation.x = float(self.svxq.get())
            except:
                self.xq.delete(0, tk.END)
                self.xq.insert(0,float(self.selected_workcell_pos.transform.rotation.x))
        
        elif(arg == "yq"):
            if self.svyq.get() == "":
                #self.selected_workcell_pos.transform.rotation.y = 0.0
                return
            try:
                self.selected_workcell_pos.transform.rotation.y = float(self.svyq.get())
            except:
                self.yq.delete(0, tk.END)
                self.yq.insert(0,float(self.selected_workcell_pos.transform.rotation.y))
        
        elif(arg == "zq"):
            if self.svzq.get() == "":
                #self.selected_workcell_pos.transform.rotation.z = 0.0
                return
            try:
                self.selected_workcell_pos.transform.rotation.z = float(self.svzq.get())
            except:
                self.zq.delete(0, tk.END)
                self.zq.insert(0,float(self.selected_workcell_pos.transform.rotation.z))
        
        elif(arg == "wq"):
            if self.svwq.get() == "":
                #self.selected_workcell_pos.transform.rotation.w = 0.0
                return
            try:
                self.selected_workcell_pos.transform.rotation.w = float(self.svwq.get())
            except:
                self.wq.delete(0, tk.END)
                self.wq.insert(0,float(self.selected_workcell_pos.transform.rotation.w))
    
    def goal_pose_callback(self, msg : geometry_msgs.msg.PoseStamped):
        self.selected_workcell_pos.transform.translation.x = msg.pose.position.x
        self.selected_workcell_pos.transform.translation.y = msg.pose.position.y
        self.selected_workcell_pos.transform.translation.z = msg.pose.position.z

        self.selected_workcell_pos.transform.rotation.x = msg.pose.orientation.x
        self.selected_workcell_pos.transform.rotation.y = msg.pose.orientation.y
        self.selected_workcell_pos.transform.rotation.z = msg.pose.orientation.z
        self.selected_workcell_pos.transform.rotation.w = msg.pose.orientation.w

        self.set_text_values()

    def set_text_values(self):
        position = self.selected_workcell_pos
        self.xt.delete(0, tk.END)
        self.xt.insert(0,position.transform.translation.x)

        self.yt.delete(0, tk.END)
        self.yt.insert(0,position.transform.translation.y)
        self.zt.delete(0, tk.END)
        self.zt.insert(0,position.transform.translation.z)

        self.xq.delete(0, tk.END)
        self.xq.insert(0,position.transform.rotation.x)
        self.yq.delete(0, tk.END)
        self.yq.insert(0,position.transform.rotation.y)
        self.zq.delete(0, tk.END)
        self.zq.insert(0,position.transform.rotation.z)
        self.wq.delete(0, tk.END)
        self.wq.insert(0,position.transform.rotation.w)
        return
    
    def optionMenuCallback(self,*args):
        if(self.value_inside.get() == ""):
            return
        
        self.selected_workcell = self.value_inside.get()
        self.selected_workcell_pos = self.workcells[self.value_inside.get()]
        self.set_text_values()
        if self.selected_workcell == self.create_workcell_text:
            self.AddWorkcellGui()
        else:
            self.RemoveWorkcellGui()
    
    def OptionMenu_addworkcelltype_cb(self, *args):
        if not self.add_workcell_type_value_inside.get() in self.workcell_string_to_type:
            self.add_workcell_req.id.robot_type.type = sm_msg.RobotType.WORK_CELL_ANY
            return
        self.add_workcell_req.id.robot_type.type = self.workcell_string_to_type[self.add_workcell_type_value_inside.get()]

    def AddWorkcellGui(self):
        self.publish_bt.grid_remove()
        self.add_bt.grid()
        self.add_workcell_label.grid()
        self.add_workcell_id.grid()
        self.add_workcell_type_options.grid()
        self.delete_bt.grid_remove()


    def RemoveWorkcellGui(self):
        self.publish_bt.grid()
        self.add_bt.grid_remove()
        self.add_workcell_label.grid_remove()
        self.add_workcell_id.grid_remove()
        self.add_workcell_type_options.grid_remove()
        self.delete_bt.grid()

    def add_workcell_id_entry_changed(self):
        self.add_workcell_req.id.id = self.sv_add_workcell_id.get()

    def refresh(self):
        if(self.update_optionslist):
    # Reset var and delete all old options
            self.value_inside.set(self.create_workcell_text)
            self.options['menu'].delete(0, 'end')

            # Insert list of new options (tk._setit hooks them up to var)
        
            for choice in self.workcells:
                self.options['menu'].add_command(label=choice, command=tk._setit(self.value_inside, choice))
            self.update_optionslist = False

        
    def get_workcells_timer_cb(self):     
        for workcell in self.workcells:
            if workcell == self.create_workcell_text:
                continue
            try:
                    t = self.tf_buffer.lookup_transform(
                        "map",
                        workcell,
                        rclpy.time.Time())
            except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform map to {workcell}: {ex}')
                    continue
            self.workcells[workcell] = t
        request = sm_srv.GetRobotsByType.Request()
        request.type.type = sm_msg.RobotType.WORK_CELL_ANY
        future = self.get_robots_client.call_async(request)
        future.add_done_callback(self.get_workcells_cb)

    def publish_position(self):
        print("prepare to publish")
        if self.selected_workcell is None or self.selected_workcell == "" or self.selected_workcell == self.create_workcell_text:
            return
        publisher = self.create_publisher(geometry_msgs.msg.PoseStamped, "/"+ self.selected_workcell + "/goal_pose", 10)
        msg = geometry_msgs.msg.PoseStamped()
        msg.header = self.selected_workcell_pos.header
        msg.pose.position.x = self.selected_workcell_pos.transform.translation.x
        msg.pose.position.y = self.selected_workcell_pos.transform.translation.y
        msg.pose.position.z = self.selected_workcell_pos.transform.translation.z
        msg.pose.orientation.x = self.selected_workcell_pos.transform.rotation.x
        msg.pose.orientation.y = self.selected_workcell_pos.transform.rotation.y
        msg.pose.orientation.z = self.selected_workcell_pos.transform.rotation.z
        msg.pose.orientation.w = self.selected_workcell_pos.transform.rotation.w
        publisher.publish(msg)

    def add_workcell(self):
        if self.selected_workcell is None or self.add_workcell_req.id.id == "" or self.add_workcell_req.id.robot_type == sm_msg.RobotType.WORK_CELL_ANY:
            print("remember to set type and add unique workcell id")
            return
        self.add_workcell_req.position = self.selected_workcell_pos.transform
        
        future = self.create_workcell_cli.call_async(self.add_workcell_req)
        future.add_done_callback(self.add_workcell_done)
        
        
    def add_workcell_done(self, future: rclpy.Future):
        result: sm_srv.CreateWorkCell.Response = future.result()
        if result.succes:   
            print("succesfully created workcell")
        else:
            print("Oh no it didnt work creating workcell, does a workcell with identical id exist??")


    def delete_workcell(self):
        request = sm_srv.DeleteWorkCell.Request()
        request.id = self.selected_workcell
        future = self.delete_workcell_cli.call_async(request)
        future.add_done_callback(self.delete_workcell_done)
        del self.workcells[request.id]
        self.deleted_workcells.append(request.id)
        self.update_optionslist = True
        self.refresh()
        return
    
    def delete_workcell_done(self, future: rclpy.Future):
        result: sm_srv.DeleteWorkCell.Response = future.result()
        if result.succes:   
            print("succesfully deleted workcell")
        else:
            print("Oh no it didnt work deleting workcell, was it already deleted??")


    def update_ros(self):
        rclpy.spin_once(self)
        self.root.after(100, self.update_ros)

    def get_workcells_cb(self, future: rclpy.Future):
        result: sm_srv.GetRobotsByType.Response = future.result()
        new_workcells = result.robots                
        for new_workcell in new_workcells:
           if new_workcell.id.id in self.deleted_workcells:
               continue
           if self.workcells.get(new_workcell.id.id) is None:
                try:
                    t = self.tf_buffer.lookup_transform(
                        "map",
                        new_workcell.id.id,
                        rclpy.time.Time())
                    print(new_workcell.id.id)
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform map to {new_workcell.id.id}: {ex}')
                    continue
                self.workcells[new_workcell.id.id] = t
                self.update_optionslist = True
        self.refresh()
    

if __name__ == '__main__':
    rclpy.init()
    ui = Ui()
    ui.root.mainloop()


        

