#!/usr/bin/python3

import rclpy
import csv

from rclpy.node import Node
from spice_msgs.msg import RobotStateTransition, RobotType, TaskData
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy




class data_processing(Node):

    def __init__(self):
        super().__init__('data_processing')
        msg_queue_size = 1000000
        self.sub_lid_states = self.create_subscription(RobotStateTransition,'/lid_cell/robot_state_transition_event',self.lid_rste_cb,msg_queue_size)
        self.sub_drill_states = self.create_subscription(RobotStateTransition,'/drill_cell/robot_state_transition_event',self.drill_rste_cb,msg_queue_size)
        self.sub_fuses_states = self.create_subscription(RobotStateTransition,'/fuses_cell/robot_state_transition_event',self.fuses_rste_cb,msg_queue_size)
        self.sub_cover_states = self.create_subscription(RobotStateTransition,'/back_cover_cell/robot_state_transition_event',self.cover_rste_cb,msg_queue_size)
        self.sub_task_data = self.create_subscription(TaskData,'/task_data',self.task_data_cb, msg_queue_size)
        self.sub_write_to_csv = self.create_subscription(String, '/do_write_csv',self.write_to_csv, msg_queue_size)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_bag_clock = self.create_subscription(Clock,'/clock',self.clock_cb,qos_profile)

        
        self.path = "/media/jonas/BIG_USB/bag_files_testing_P10/csv/data.csv"
        self.reset_variables()
        self.initialize_csv()


    def clock_cb(self, msg : Clock):
        self.current_time = msg


    def task_data_cb(self, msg : TaskData):
        if msg.task_state == TaskData.TASKDONE:
            self.total_task_time += msg.total_time
            self.task_done_counter += 1

        elif msg.task_state == TaskData.ENTERWORKCELL:
            self.total_queue_time += msg.total_time
            self.queue_counter += 1

        elif msg.task_state == TaskData.ERROR:
            self.error_counter += 1


    def rste_cb(self, msg : RobotStateTransition):

        # cycle time of work cell from robot entering to it have exited
        if "ROBOT_ENTERING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP:
                self.start_lid_cycle_time = self.current_time.clock.sec
            elif msg.id.robot_type.type == RobotType.WORK_CELL_DRILL:
                self.start_drill_cycle_time = self.current_time.clock.sec
            elif msg.id.robot_type.type == RobotType.WORK_CELL_FUSES:
                self.start_fuses_cycle_time = self.current_time.clock.sec
            elif msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER:
                self.start_cover_cycle_time = self.current_time.clock.sec
            
        elif "ROBOT_EXITING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP and self.start_lid_cycle_time != 0:
                self.total_lid_cycle_time += self.current_time.clock.sec - self.start_lid_cycle_time
                self.start_lid_cycle_time = 0
                self.lid_cycle_counter += 1

            elif msg.id.robot_type.type == RobotType.WORK_CELL_DRILL and self.start_drill_cycle_time != 0:
                self.drill_cycle_counter += 1
                self.total_drill_cycle_time += self.current_time.clock.sec - self.start_drill_cycle_time
                # self.get_logger().info(f'cycle nr: {self.drill_cycle_counter}, adding {self.current_time.clock.sec - self.start_drill_cycle_time} so new total {self.total_drill_cycle_time}')
                self.start_drill_cycle_time = 0

            elif msg.id.robot_type.type == RobotType.WORK_CELL_FUSES and self.start_fuses_cycle_time != 0:
                self.total_fuses_cycle_time += self.current_time.clock.sec - self.start_fuses_cycle_time
                self.start_fuses_cycle_time = 0
                self.fuses_cycle_counter += 1

            elif msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER and self.start_cover_cycle_time != 0:
                self.total_cover_cycle_time += self.current_time.clock.sec - self.start_cover_cycle_time
                self.start_cover_cycle_time = 0
                self.cover_cycle_counter += 1
                
        # processing time of work cell from starting process on robot to end process (this should always be 5sec)
        if "PROCESSING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP:
                self.start_lid_processing_time = self.current_time.clock.sec
            elif msg.id.robot_type.type == RobotType.WORK_CELL_DRILL:
                self.start_drill_processing_time = self.current_time.clock.sec
            elif msg.id.robot_type.type == RobotType.WORK_CELL_FUSES:
                self.start_fuses_processing_time = self.current_time.clock.sec
            elif msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER:
                self.start_cover_processing_time = self.current_time.clock.sec
            
        elif "PROCESSING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP and self.start_lid_processing_time != 0:
                self.total_lid_processing_time += self.current_time.clock.sec - self.start_lid_processing_time
                self.start_lid_processing_time = 0

            elif msg.id.robot_type.type == RobotType.WORK_CELL_DRILL and self.start_drill_processing_time != 0:
                self.total_drill_processing_time += self.current_time.clock.sec - self.start_drill_processing_time
                # self.get_logger().info(f'processing, adding {self.current_time.clock.sec-self.start_drill_processing_time} so new total is {self.total_drill_processing_time}')
                self.start_drill_processing_time = 0

            elif msg.id.robot_type.type == RobotType.WORK_CELL_FUSES and self.start_fuses_processing_time != 0:
                self.total_fuses_processing_time += self.current_time.clock.sec - self.start_fuses_processing_time
                self.start_fuses_processing_time = 0

            elif msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER and self.start_cover_processing_time != 0:
                self.total_cover_processing_time += self.current_time.clock.sec - self.start_cover_processing_time
                self.start_cover_processing_time = 0


    def lid_rste_cb(self, msg : RobotStateTransition):

        # cycle time of work cell from robot entering to it have exited
        if "ROBOT_ENTERING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP:
                self.start_lid_cycle_time = self.current_time.clock.sec
            
        elif "ROBOT_EXITING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP and self.start_lid_cycle_time != 0:
                self.total_lid_cycle_time += self.current_time.clock.sec - self.start_lid_cycle_time
                self.start_lid_cycle_time = 0
                self.lid_cycle_counter += 1
                
        # processing time of work cell from starting process on robot to end process (this should always be 5sec)
        if "PROCESSING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP:
                self.start_lid_processing_time = self.current_time.clock.sec
            
        elif "PROCESSING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_TOP and self.start_lid_processing_time != 0:
                self.total_lid_processing_time += self.current_time.clock.sec - self.start_lid_processing_time
                self.start_lid_processing_time = 0


    def drill_rste_cb(self, msg : RobotStateTransition):

        # cycle time of work cell from robot entering to it have exited
        if "ROBOT_ENTERING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_DRILL:
                self.start_drill_cycle_time = self.current_time.clock.sec
            
        elif "ROBOT_EXITING" in msg.old_state.internal_state:

            if msg.id.robot_type.type == RobotType.WORK_CELL_DRILL and self.start_drill_cycle_time != 0:
                self.drill_cycle_counter += 1
                self.total_drill_cycle_time += self.current_time.clock.sec - self.start_drill_cycle_time
                # self.get_logger().info(f'cycle nr: {self.drill_cycle_counter}, adding {self.current_time.clock.sec - self.start_drill_cycle_time} so new total {self.total_drill_cycle_time}')
                self.start_drill_cycle_time = 0
                
        # processing time of work cell from starting process on robot to end process (this should always be 5sec)
        if "PROCESSING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_DRILL:
                self.start_drill_processing_time = self.current_time.clock.sec
            
        elif "PROCESSING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_DRILL and self.start_drill_processing_time != 0:
                self.total_drill_processing_time += self.current_time.clock.sec - self.start_drill_processing_time
                self.start_drill_processing_time = 0


    def fuses_rste_cb(self, msg : RobotStateTransition):

        # cycle time of work cell from robot entering to it have exited
        if "ROBOT_ENTERING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_FUSES:
                self.start_fuses_cycle_time = self.current_time.clock.sec
                # self.get_logger().info(f'starting fuse cycle nr: {self.fuses_cycle_counter+1}')
    
            
        elif "ROBOT_EXITING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_FUSES and self.start_fuses_cycle_time != 0:
                self.fuses_cycle_counter += 1
                self.total_fuses_cycle_time += self.current_time.clock.sec - self.start_fuses_cycle_time
                # self.get_logger().info(f'cycle nr: {self.fuses_cycle_counter}, adding {self.current_time.clock.sec - self.start_fuses_cycle_time} so new total {self.total_fuses_cycle_time}')
                self.start_fuses_cycle_time = 0
                
        # processing time of work cell from starting process on robot to end process (this should always be 5sec)
        if "PROCESSING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_FUSES:
                self.start_fuses_processing_time = self.current_time.clock.sec
            
        elif "PROCESSING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_FUSES and self.start_fuses_processing_time != 0:
                self.total_fuses_processing_time += self.current_time.clock.sec - self.start_fuses_processing_time
                # self.get_logger().info(f'Fuse processing complete adding {self.current_time.clock.sec - self.start_fuses_processing_time} so new total {self.total_fuses_processing_time}')
                self.start_fuses_processing_time = 0


    def cover_rste_cb(self, msg : RobotStateTransition):

        # cycle time of work cell from robot entering to it have exited
        if "ROBOT_ENTERING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER:
                self.start_cover_cycle_time = self.current_time.clock.sec
            
        elif "ROBOT_EXITING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER and self.start_cover_cycle_time != 0:
                self.total_cover_cycle_time += self.current_time.clock.sec - self.start_cover_cycle_time
                self.start_cover_cycle_time = 0
                self.cover_cycle_counter += 1
                
        # processing time of work cell from starting process on robot to end process (this should always be 5sec)
        if "PROCESSING" in msg.new_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER:
                self.start_cover_processing_time = self.current_time.clock.sec
            
        elif "PROCESSING" in msg.old_state.internal_state:
            if msg.id.robot_type.type == RobotType.WORK_CELL_BACK_COVER and self.start_cover_processing_time != 0:
                self.total_cover_processing_time += self.current_time.clock.sec - self.start_cover_processing_time
                self.start_cover_processing_time = 0


    def initialize_csv(self):
        self.get_logger().info(f'Creating file: {self.path}')

        with open(self.path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "Timestamp", "Group","Tasks completed", "Total Time for tasks","Average task time",
                "Queue counter", "Total time enqueued", "Average queue time", "Error state",
                "Total cycle counter", "Total cycle time", "Average cycle time", "Total processing time", "Average utilisation",
                "Lid cell processing", "Lid cell cycle", "Lid cycle counter", "Average lid cell cycle time",
                "Drill cell processing", "Drill cell cycle", "Drill cycle counter", "Average drill cell cycle time",
                "Fuses cell processing", "Fuses cell cycle", "Fuses cycle counter", "Average fuses cell cycle time",
                "Back cover cell processing", "Back cover cell cycle", "Back cover cycle counter", "Average back cover cell cycle time"
            ])
    

    def write_to_csv(self, msg : String):
        
        msg_split= msg.data.rsplit("/",2)
        timestamp = msg_split[2][13:27]
        current_bag_type = msg_split[1]

        average_task_time = self.total_task_time / self.task_done_counter
        average_queue_time = self.total_queue_time / self.queue_counter
        totale_cycle_time = self.total_lid_cycle_time + self.total_drill_cycle_time + self.total_fuses_cycle_time + self.total_cover_cycle_time
        total_cycle_counter = self.lid_cycle_counter + self.drill_cycle_counter + self.fuses_cycle_counter + self.cover_cycle_counter
        average_cycle_time = totale_cycle_time/total_cycle_counter
        total_processing_time = self.total_lid_processing_time + self.total_drill_processing_time + self.total_fuses_processing_time + self.total_cover_processing_time
        average_lid_cycle_time = self.total_lid_cycle_time/self.lid_cycle_counter
        average_drill_cycle_time = self.total_drill_cycle_time/self.drill_cycle_counter
        average_fuses_cycle_time = self.total_fuses_cycle_time/self.fuses_cycle_counter
        average_cover_cycle_time = self.total_cover_cycle_time/self.cover_cycle_counter
        lid_cell_utilisation = self.total_lid_cycle_time/6 # /6 = /600*100 aka /10min *100 to get in %.
        drill_cell_utilisation = self.total_drill_cycle_time/6
        fuses_cell_utilisation = self.total_fuses_cycle_time/6
        cover_cell_utilisation = self.total_cover_cycle_time/6
        average_utilisation = (lid_cell_utilisation + drill_cell_utilisation + fuses_cell_utilisation + cover_cell_utilisation)/4


        with open(self.path, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp, current_bag_type, self.task_done_counter, self.total_task_time, average_task_time,
                self.queue_counter, self.total_queue_time, average_queue_time, self.error_counter,  
                total_cycle_counter, totale_cycle_time, average_cycle_time, total_processing_time, average_utilisation,
                self.total_lid_processing_time, self.total_lid_cycle_time, self.lid_cycle_counter, average_lid_cycle_time, lid_cell_utilisation,
                self.total_drill_processing_time, self.total_drill_cycle_time, self.drill_cycle_counter, average_drill_cycle_time, drill_cell_utilisation,
                self.total_fuses_processing_time, self.total_fuses_cycle_time, self.fuses_cycle_counter, average_fuses_cycle_time, fuses_cell_utilisation, 
                self.total_cover_processing_time, self.total_cover_cycle_time, self.cover_cycle_counter, average_cover_cycle_time, cover_cell_utilisation
            ])
        self.get_logger().info(f'saving bag {timestamp} of type {current_bag_type} to location {self.path}')
        self.reset_variables()


    def reset_variables(self):
        self.total_queue_time = 0
        self.queue_counter = 0
        self.error_counter = 0
        self.total_task_time = 0
        self.task_done_counter = 0

        self.total_lid_processing_time = 0
        self.start_lid_processing_time = 0

        self.total_drill_processing_time = 0
        self.start_drill_processing_time = 0

        self.total_fuses_processing_time = 0
        self.start_fuses_processing_time = 0

        self.total_cover_processing_time = 0
        self.start_cover_processing_time = 0

        self.total_lid_cycle_time = 0
        self.start_lid_cycle_time = 0
        self.lid_cycle_counter = 0

        self.total_drill_cycle_time = 0
        self.start_drill_cycle_time = 0
        self.drill_cycle_counter = 0

        self.total_fuses_cycle_time = 0
        self.start_fuses_cycle_time = 0
        self.fuses_cycle_counter = 0

        self.total_cover_cycle_time = 0
        self.start_cover_cycle_time = 0
        self.cover_cycle_counter = 0



def main(args=None):
    rclpy.init(args=args)
    data_processing_node = data_processing()

    rclpy.spin(data_processing_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_processing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()