#!/usr/bin/env python3
...

from spice_msgs.srv import *
from spice_msgs.msg import *
from datetime import *
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from dataclasses import dataclass
from typing import Dict

@dataclass
class RobotData():
    id: Id
    robot_state: RobotState
    heartbeat_time: datetime
    state_subscriber: Subscription

class SwarmManager(Node):

    robots_dict: Dict["str", "RobotData"] = dict() 

    def __init__(self):
        super().__init__('SwarmManager')
        self.srvRegisterRobot = self.create_service(RegisterRobot, 'register_robot', self.register_robot_callback)
        self.srvGetReadyRobots = self.create_service(GetReadyRobots, 'get_ready_robots', self.get_ready_robots_callback)
        self.srvGetRobots = self.create_service(GetRobots, 'get_robots', self.get_robots_callback)
        self.srvHeartbeat = self.create_service(Heartbeat, 'heartbeat', self.heartbeat_callback)
        self.timerHeartbeat = self.create_timer(1, self.heartbeatTimer_callback)



    def registerRobot(self, id: Id) -> bool: #Register new robots and subscribe to their state_transition_event
        for robot in self.robots_dict.values():
            if id.id == robot.id.id:
                return False
            
        topic = id.id+'/robot_state_transition_event'
        statesub = self.create_subscription(RobotStateTransition, topic, self.robot_state_transition_callback,10)
        robot = RobotData(id, RobotState(state=RobotState.STARTUP), datetime.now(),statesub)
        self.robots_dict[id.id] = robot
        self.get_logger().info(f"{id} has been registered")
        return True


    def deregisterRobot(self, id:Id) -> bool: # unregister robot and stop subcribing to the that state event topic 
        found = False
        for robot in self.robots_dict.values():
            if id.id == robot.id.id:
                found = True
                break

        if found:
            self.destroy_subscription(self.robots_dict[id.id].state_subscriber)
            del self.robots_dict[id.id]
            self.get_logger().info(f"{id} has been removed")
            return True
        return False

    
    def register_robot_callback(self, request:RegisterRobot.Request, response:RegisterRobot.Response) -> RegisterRobot.Response: #srv to register robots when they start
        if self.registerRobot(request.id):
            response.success = True
        else:
            response.success = False
        
        return response
    

    def get_robots_callback(self, request:GetRobots.Request, response:GetRobots.Response) -> GetRobots.Response: # send all registered robots
        for robot in self.robots_dict.values():
            robot_msg = Robot()
            robot_msg.id = robot.id
            robot_msg.robot_state = robot.robot_state
            response.robots.append(robot_msg)

        return response
    
    def get_ready_robots_callback(self, request:GetReadyRobots.Request, response:GetReadyRobots.Response) -> GetReadyRobots.Response: #send robots that are waiting for task
        for robot in self.robots_dict.values():
            if robot.robot_state.state == RobotState.READY_FOR_JOB:
                response.robots.append(robot.id)

        return response

    def heartbeat_callback(self, request:Heartbeat.Request, response:Heartbeat.Response) -> Heartbeat.Response:
        response.restart_robot = True
        for robot in self.robots_dict.values():
            if request.id.id == robot.id.id:
                response.restart_robot = False
                robot.heartbeat_time = datetime.now()
                break

        return response

    def heartbeatTimer_callback(self):
        idRobotsToDel = []
        for robot in self.robots_dict.values():
            if datetime.now() - robot.heartbeat_time > timedelta(seconds=10):
                idRobotsToDel.append(robot.id)

        for delId in idRobotsToDel:
            self.get_logger().info(f"Timeout of {delId}")
            self.deregisterRobot(delId)



    def robot_state_transition_callback(self, msg:RobotStateTransition):
        self.robots_dict[msg.id.id].robot_state = msg.new_state
        self.robots_dict[msg.id.id].heartbeat_time = datetime.now()



def main(args=None):
    rclpy.init(args=args)
    swarmmanager = SwarmManager()
    rclpy.spin(swarmmanager)
    rclpy.shutdown()



if __name__ == '__main__':
    main()