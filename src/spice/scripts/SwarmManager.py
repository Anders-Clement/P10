#!/usr/bin/env python3
...

from spice_msgs.srv import *
from spice_msgs.msg import *
from datetime import *
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
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
        self.srv_register_robot = self.create_service(RegisterRobot, 'register_robot', self.register_robot_callback)
        self.srv_get_ready_robots = self.create_service(GetReadyRobots, 'get_ready_robots', self.get_ready_robots_callback)
        self.srv_get_robots = self.create_service(GetRobots, 'get_robots', self.get_robots_callback)
        self.srv_get_robots_by_type = self.create_service(GetRobotsByType, 'get_robots_by_type', self.get_robots_by_type_callback)
        self.srv_get_robots_by_state = self.create_service(GetRobotsByState, 'get_robots_by_state', self.get_robots_by_state_callback)
        self.srv_heartbeat = self.create_service(Heartbeat, 'heartbeat', self.heartbeat_callback)
        self.timer_heartbeat = self.create_timer(1, self.heartbeat_timer_callback)

    def register_robot(self, id: Id) -> bool: #Register new robots and subscribe to their state_transition_event
        for robot in self.robots_dict.values():
            if id.id == robot.id.id:
                return False
            
        topic = id.id+'/robot_state_transition_event'
        qos = QoSProfile(
                history = QoSHistoryPolicy.KEEP_LAST, 
                reliability = QoSReliabilityPolicy.RELIABLE,
                durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth = 10
            )
        statesub = self.create_subscription(RobotStateTransition, topic, self.robot_state_transition_callback, qos)
        robot = RobotData(id, RobotState(state=RobotState.STARTUP), datetime.now(),statesub)
        self.robots_dict[id.id] = robot
        self.get_logger().info(f"{id} has been registered, subscribing to topic: {topic}")
        return True

    def deregister_robot(self, id:Id) -> bool: # unregister robot and stop subcribing to the that state event topic 
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
        if self.register_robot(request.id):
            response.success = True
        else:
            response.success = False
        
        return response
    
    def get_robots_callback(self, request:GetRobots.Request, response:GetRobots.Response) -> GetRobots.Response: # send all registered robots
        response.robots = [Robot(id=robot.id, robot_state = robot.robot_state) for robot in self.robots_dict.values()]

        return response
    
    def get_ready_robots_callback(self, request:GetReadyRobots.Request, response:GetReadyRobots.Response) -> GetReadyRobots.Response: #send robots that are waiting for task
        response.robots = [Robot(id=robot.id, robot_state=robot.robot_state) \
                           for robot in self.robots_dict.values() if robot.robot_state.state == RobotState.MR_READY_FOR_JOB]
        return response

    def heartbeat_callback(self, request:Heartbeat.Request, response:Heartbeat.Response) -> Heartbeat.Response:
        response.restart_robot = True
        for robot in self.robots_dict.values():
            if request.id.id == robot.id.id:
                response.restart_robot = False
                robot.heartbeat_time = datetime.now()
                break

        return response

    def heartbeat_timer_callback(self):
        idRobotsToDel = []
        for robot in self.robots_dict.values():
            if datetime.now() - robot.heartbeat_time > timedelta(seconds=10):
                idRobotsToDel.append(robot.id)

        for delId in idRobotsToDel:
            self.get_logger().info(f"Timeout of {delId}")
            self.deregister_robot(delId)

    def robot_state_transition_callback(self, msg:RobotStateTransition):
        old_state = self.robots_dict[msg.id.id].robot_state
        self.robots_dict[msg.id.id].robot_state = msg.new_state
        new_state = self.robots_dict[msg.id.id].robot_state
        self.robots_dict[msg.id.id].heartbeat_time = datetime.now()

        self.get_logger().info(f'Got state transition event {msg} \n old robot_state: {old_state}, new robot_state: {new_state}')

    def get_robots_by_type_callback(self, request: GetRobotsByType.Request, response: GetRobotsByType.Response):
        response.robots = [Robot(id=robot.id, robot_state=robot.robot_state) \
                           for robot in self.robots_dict.values() if robot.id.robot_type == request.type]
        return response
    
    def get_robots_by_state_callback(self, request: GetRobotsByState.Request, response: GetRobotsByState.Response):
        response.robots = [Robot(id=robot.id, robot_state=robot.robot_state) \
                           for robot in self.robots_dict.values() if robot.robot_state.state == request.state.state]
        return response



def main(args=None):
    rclpy.init(args=args)
    swarmmanager = SwarmManager()
    rclpy.spin(swarmmanager)
    rclpy.shutdown()



if __name__ == '__main__':
    main()