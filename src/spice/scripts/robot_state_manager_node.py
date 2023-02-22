#!/usr/bin/python3

import os
import enum

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose

from spice_msgs.msg import RobotState, RobotStateTransition, Id
from spice_msgs.srv import Heartbeat

import robot_state

class ROBOT_STATE(enum.IntEnum):
    STARTUP = 0
    READY_FOR_JOB = 1
    MOVING = 2
    PROCESSING = 3
    ERROR = 4

class RobotStateManager(Node):
    def __init__(self) -> None:
        super().__init__('robot_state_manager_node')
        robot_ns = os.environ.get('ROBOT_NAMESPACE')
        if robot_ns is None:
            print('Could not get robot namespace from the environment')
            raise Exception()
        self.id = robot_ns
        self.current_task = None

        self.state_transition_event_pub = self.create_publisher(RobotStateTransition, 'robot_state_transition_event', 10)

        self.heartbeat_client = self.create_client(Heartbeat, '/heartbeat')
        self.heartbeat_timer = self.create_timer(5, self.heartbeat_timer_cb)
        self.heartbeat_timer.cancel()
        self.heartbeat_future = None

        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        

        self.states: "[RobotState]" = [
            robot_state.StartUpState(self),
            robot_state.ReadyForJobState(self),
            robot_state.MovingState(self),
            robot_state.ProcessingState(self),
            robot_state.ErrorState(self)
        ]
        self.current_state = ROBOT_STATE.STARTUP
        self.state_transition_event_pub.publish(
            RobotStateTransition(new_state=RobotState(state=self.current_state),
                                 old_state=RobotState(state=ROBOT_STATE.STARTUP),
                                 id=Id(id=self.id)))
        self.states[self.current_state].init()

        self.get_logger().info('Finished intialization')

    def change_state(self, new_state: ROBOT_STATE):
        if self.current_state == new_state:
            self.get_logger().error(f'Tried tog to state transition to same state: {new_state}')
        else:
            self.get_logger().info(f'State transition from: {self.current_state.name} to {new_state.name}')

            event_msg = RobotStateTransition()
            event_msg.old_state = RobotState(state=self.current_state)
            event_msg.new_state = RobotState(state=new_state)
            self.state_transition_event_pub.publish(event_msg)

            self.states[self.current_state].deinit()
            self.current_state = new_state
            self.states[self.current_state].init()
        
    def heartbeat_timer_cb(self):
        if self.current_state == ROBOT_STATE.READY_FOR_JOB \
            or self.current_state == ROBOT_STATE.MOVING \
            or self.current_state == ROBOT_STATE.PROCESSING:
            if self.heartbeat_future is None:
                heartbeat = Heartbeat.Request(id=Id(id=self.id))
                self.heartbeat_future = self.heartbeat_client.call_async(heartbeat)
                self.heartbeat_future.add_done_callback(self.heartbeat_cb)
            else:
                self.get_logger().error('Did not receive answer to heartbeat in time, resetting!')
                self.change_state(ROBOT_STATE.STARTUP)

    def heartbeat_cb(self, future: Future):
        result: Heartbeat.Response = future.result()
        if result.restart_robot:
            self.change_state(ROBOT_STATE.STARTUP)
        self.heartbeat_future = None

    def on_nav_feedback(self, msg):
        self.states[self.current_state].on_nav_feedback(msg)
    
    def on_nav_done(self, msg):
        self.states[self.current_state].on_nav_done(msg)


def main(args=None):
    rclpy.init(args=args)

    robot_state_manager_node = RobotStateManager()

    rclpy.spin(robot_state_manager_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_state_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()