#!/usr/bin/python3

import os
import enum

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav2_msgs.action import NavigateToPose

from spice_msgs.msg import RobotState, RobotStateTransition, Id, RobotType
from spice_msgs.srv import Heartbeat, RobotTask
from work_tree import WorkTree, Vertex
import robot_state


class ROBOT_STATE(enum.IntEnum):
    STARTUP = 0
    READY_FOR_JOB = 1
    FIND_WORKCELL = 2
    MOVING = 3
    REGISTER_WORK = 4
    WAIT_IN_QUEUE = 5
    ENTER_WORKCELL = 6
    READY_FOR_PROCESS = 7
    PROCESS_DONE = 8
    EXIT_WORKCELL = 9
    ERROR = 10
   

class RobotStateManager(Node):
    task_tree: WorkTree
    
    def __init__(self) -> None:
        super().__init__('robot_state_manager_node')
        robot_ns = os.environ.get('ROBOT_NAMESPACE')
        if robot_ns is None:
            print('Could not get robot namespace from the environment')
            raise Exception()
        self.id = Id(id=robot_ns, robot_type=RobotType(type=RobotType.CARRIER_ROBOT))
        self.current_work = None
        self.current_work_cell_info = None
        self.task_tree = None

        qos = QoSProfile(
                history = QoSHistoryPolicy.KEEP_LAST, 
                reliability = QoSReliabilityPolicy.RELIABLE,
                durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth = 10
            )
        
        self.state_transition_event_pub = self.create_publisher(RobotStateTransition, 'robot_state_transition_event', qos)

        self.heartbeat_client = self.create_client(Heartbeat, '/heartbeat')
        self.heartbeat_timer = self.create_timer(5, self.heartbeat_timer_cb)
        self.heartbeat_timer.cancel()
        self.heartbeat_future = None

        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.allocate_task_server = self.create_service(
            RobotTask, 'allocate_task', self.allocate_task_cb)
        

        self.states: "[RobotState]" = [
            robot_state.StartUpState(self),
            robot_state.ReadyForJobState(self),
            robot_state.FindWorkCell(self),
            robot_state.MovingState(self),
            robot_state.ProcessRegisterWorkState(self),
            robot_state.ProcessWaitQueueState(self),
            robot_state.EnterWorkCellState(self),
            robot_state.ProcessReadyForProcessingState(self),
            robot_state.ProcessProcessingDoneState(self),
            robot_state.ProcessExitWorkCellState(self),
            robot_state.ErrorState(self)
        ]

        self.current_state = ROBOT_STATE.STARTUP
        self.state_transition_event_pub.publish(
            RobotStateTransition(new_state=RobotState(state=self.current_state),
                                 old_state=RobotState(state=ROBOT_STATE.STARTUP),
                                 id=self.id))
        self.states[self.current_state].init()

        self.get_logger().info('Finished intialization')

    def change_state(self, new_state: ROBOT_STATE):
        if self.current_state == new_state:
            self.get_logger().error(f'Tried to transition to same state: {new_state}')
        else:
            self.get_logger().info(f'State transition from: {self.current_state.name} to {new_state.name}')

            def internal_robot_state_to_robot_state_msg_state(internal_state: ROBOT_STATE) -> RobotState:
                robot_state_msg = RobotState(internal_state=internal_state.value)
                if internal_state == ROBOT_STATE.STARTUP:
                    robot_state_msg.state=RobotState.STARTUP
                elif internal_state == ROBOT_STATE.READY_FOR_JOB:
                    robot_state_msg.state=RobotState.MR_READY_FOR_JOB
                elif internal_state == ROBOT_STATE.MOVING or \
                        internal_state == ROBOT_STATE.REGISTER_WORK or \
                        internal_state == ROBOT_STATE.WAIT_IN_QUEUE or \
                        internal_state == ROBOT_STATE.READY_FOR_PROCESS or \
                        internal_state == ROBOT_STATE.PROCESS_DONE or \
                        internal_state == ROBOT_STATE.EXIT_WORKCELL:
                    robot_state_msg.state=RobotState.MR_PROCESSING_JOB
                elif internal_state == ROBOT_STATE.ERROR:
                    robot_state_msg.state=RobotState.ERROR
                else:
                    self.get_logger().error("Internal state is not published correctly")
                return robot_state_msg

            event_msg = RobotStateTransition()
            event_msg.old_state = internal_robot_state_to_robot_state_msg_state(self.current_state)
            event_msg.new_state = internal_robot_state_to_robot_state_msg_state(new_state)
            event_msg.id = self.id
            self.state_transition_event_pub.publish(event_msg)

            self.states[self.current_state].deinit()
            self.current_state = new_state
            self.states[self.current_state].init()
        
    def heartbeat_timer_cb(self):
        if self.current_state == ROBOT_STATE.READY_FOR_JOB \
            or self.current_state == ROBOT_STATE.MOVING \
            or self.current_state == ROBOT_STATE.FIND_WORKCELL\
            or self.current_state == ROBOT_STATE.REGISTER_WORK \
            or self.current_state == ROBOT_STATE.WAIT_IN_QUEUE \
            or self.current_state == ROBOT_STATE.READY_FOR_PROCESS\
            or self.current_state == ROBOT_STATE.PROCESS_DONE\
            or self.current_state == ROBOT_STATE.EXIT_WORKCELL\
            or self.current_state == ROBOT_STATE.ERROR:

            if self.heartbeat_future is None:
                heartbeat = Heartbeat.Request(id=self.id)
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

    def allocate_task_cb(self, request: RobotTask.Request, response: RobotTask.Response) -> RobotTask.Response:
        return self.states[self.current_state].on_allocate_task(request, response)


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