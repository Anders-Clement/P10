#!/usr/bin/python3

import os
import enum
import sys

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav2_msgs.action import NavigateToPose

from spice_msgs.msg import RobotState, RobotStateTransition, Id, RobotType, TaskData
from spice_msgs.srv import Heartbeat, RobotTask, SetPlannerType
from work_tree import WorkTree
import robot_state
from geometry_msgs.msg import PoseStamped


class ROBOT_STATE(enum.IntEnum):
    STARTUP = 0
    READY_FOR_JOB = 1
    FIND_WORKCELL = 2
    MOVING = 3
    REGISTER_WORK = 4
    ENQUEUED = 5
    ENTER_WORKCELL = 6
    READY_FOR_PROCESS = 7
    PROCESS_DONE = 8
    EXIT_WORKCELL = 9
    ERROR = 10


class HeartBeatHandler:
    def __init__(self, service_topic: str, period: float, robot_id: Id, heartbeat_failed_func, nodehandle: Node) -> None:
        self.robot_id = robot_id
        self.heartbeat_client = nodehandle.create_client(Heartbeat, service_topic)
        self.heartbeat_timer = nodehandle.create_timer(period, self.heartbeat_timer_cb)
        self.heartbeat_timer.cancel()
        self.heartbeat_failed_func = heartbeat_failed_func
        self.nodehandle = nodehandle
        self.heartbeat_topic = service_topic
        self.heartbeat_future = None

    def heartbeat_timer_cb(self):
        if self.heartbeat_future is not None:
            self.nodehandle.get_logger().warn(f'heartbeat failed to topic: {self.heartbeat_topic}')
            self.deactivate()
            self.heartbeat_failed_func(self.nodehandle)
        else:
            request = Heartbeat.Request()
            request.id = self.robot_id
            self.heartbeat_future = self.heartbeat_client.call_async(request)
            self.heartbeat_future.add_done_callback(self.heartbeat_cb)

    def heartbeat_cb(self, future: Future):
        result: Heartbeat.Response = future.result()
        self.heartbeat_future = None
        if result.restart_robot:
            self.deactivate()
            self.heartbeat_failed_func(self.nodehandle)

    def activate(self):
        self.heartbeat_timer.reset()

    def deactivate(self):
        self.heartbeat_timer.cancel()
        self.heartbeat_future = None
        # self.nodehandle.destroy_timer(self.heartbeat_timer)
        # self.nodehandle.destroy_client(self.heartbeat_client)
   

class RobotStateManager(Node):
    task_tree: WorkTree
    
    def __init__(self) -> None:
        super().__init__('robot_state_manager_node')
        #robot_ns = os.environ.get('ROBOT_NAMESPACE')
        robot_ns = self.get_namespace()[1:]
        if robot_ns is None:
            self.get_logger().error('Could not get robot namespace')
            raise Exception()
        self.id = Id(id=robot_ns, robot_type=RobotType(type=RobotType.CARRIER_ROBOT))
        self.current_work = None
        self.current_work_cell_info = None
        self.task_tree = None
        self.current_task_id = 0
        self.task_start_time = 0
        self.enqueud_start_time = 0
        self.state_data_pub = self.create_publisher(TaskData, "/task_data", 10)

        qos = QoSProfile(
                history = QoSHistoryPolicy.KEEP_LAST, 
                reliability = QoSReliabilityPolicy.RELIABLE,
                durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth = 10
            )
        
        

        self.state_transition_event_pub = self.create_publisher(RobotStateTransition, 'robot_state_transition_event', qos)

        self.heartbeat = HeartBeatHandler('/heartbeat', 2.5, self.id, lambda arg : arg.change_state(ROBOT_STATE.STARTUP), self)
        self.work_cell_heartbeat: Heartbeat = None

        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.change_planner_type_client = self.create_client(SetPlannerType, "set_planner_type")

        self.allocate_task_server = self.create_service(
            RobotTask, 'allocate_task', self.allocate_task_cb)
        

        self.states: "[RobotState]" = [
            robot_state.StartUpState(self),
            robot_state.ReadyForJobState(self),
            robot_state.FindWorkCell(self),
            robot_state.MovingState(self),
            robot_state.ProcessRegisterWorkState(self),
            robot_state.EnqueuedState(self),
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
                robot_state_msg = RobotState(internal_state=internal_state.name)
                if internal_state == ROBOT_STATE.STARTUP:
                    robot_state_msg.state=RobotState.STARTUP
                elif internal_state == ROBOT_STATE.READY_FOR_JOB:
                    robot_state_msg.state=RobotState.MR_READY_FOR_JOB
                elif internal_state == ROBOT_STATE.ERROR:
                    robot_state_msg.state=RobotState.ERROR
                else:
                    robot_state_msg.state=RobotState.MR_PROCESSING_JOB
                return robot_state_msg

            event_msg = RobotStateTransition()
            event_msg.old_state = internal_robot_state_to_robot_state_msg_state(self.current_state)
            event_msg.new_state = internal_robot_state_to_robot_state_msg_state(new_state)
            event_msg.id = self.id
            self.state_transition_event_pub.publish(event_msg)

            self.states[self.current_state].deinit()
            self.current_state = new_state
            self.states[self.current_state].init()
        

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