from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from spice_msgs.msg import Id
from spice_msgs.srv import RegisterRobot, RobotTask

from robot_state_manager_node import RobotStateManager, ROBOT_STATE


class RobotStateTemplate():
    def __init__(self) -> None:
        raise NotImplementedError()
    def init(self):
        raise NotImplementedError()
    def deinit(self):
        raise NotImplementedError()
    def on_nav_feedback(self, msg):
        pass
    def on_nav_done(self, msg):
        pass
    

class StartUpState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
        self.register_robot_client = self.sm.create_client(RegisterRobot, '/register_robot')
        self.navigation_is_active_client = self.sm.create_client(Trigger, 'lifecycle_manager_navigation/is_active')

    def init(self):
        # check that all robot state is good
        self.check_nav2_stack_status()

    def check_nav2_stack_status(self):
        while not self.navigation_is_active_client.wait_for_service(5):
            self.sm.get_logger().info('timeout on wait for service: lifecycle_manager_navigation/is_active')
        
        self.nav_stack_is_active_future = self.navigation_is_active_client.call_async(Trigger.Request())
        self.nav_stack_is_active_future.add_done_callback(self.nav_stack_is_active_cb)

    def nav_stack_is_active_cb(self, future: Future):
        result: Trigger.Response = future.result()
        if result.success:
            self.register_robot()
        else:
            self.check_nav2_stack_status()

    def register_robot(self):
        register_robot_request = RegisterRobot.Request()
        register_robot_request.id = Id(id=self.sm.id)
        while not self.register_robot_client.wait_for_service(5):
            self.sm.get_logger().info('Robot StartUpState timeout for /register_robot service')

        self.register_future = self.register_robot_client.call_async(register_robot_request)
        self.register_future.add_done_callback(self.register_robot_done_callback)

    def register_robot_done_callback(self, future: Future):
        response: RegisterRobot.Response = future.result()
        if response.success:
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)
        else:
            self.sm.get_logger().info('Failed to register robot, is it already registered?')
            self.sm.change_state(ROBOT_STATE.ERROR)

    def deinit(self):
        if self.register_future is not None:
            if not self.register_future.cancelled():
                self.register_future.cancel()
        if self.nav_stack_is_active_future is not None:
            if not self.nav_stack_is_active_future.cancelled():
                self.nav_stack_is_active_future.cancel()


class ReadyForJobState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
        self.allocate_task_server = self.sm.create_service(
            RobotTask, 'allocate_task', self.allocate_task_cb)
        
    def init(self):
        if self.sm.heartbeat_timer.is_canceled():
            self.sm.heartbeat_timer.reset()
    def deinit(self):
        pass

    def allocate_task_cb(self, request: RobotTask.Request, response: RobotTask.Response) -> RobotTask.Response:
        if self.sm.current_task is not None:
            response.job_accepted = False
            return response
        
        if not self.sm.navigation_client.wait_for_server(5):
            response.job_accepted = False
            return response
        
        self.sm.current_task = request
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = request.goal_pose
        nav_response_future = self.sm.navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback)
        nav_response_future.add_done_callback(self.nav_goal_response_cb)

        response.job_accepted = True
        return response
    
    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting task')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        nav_goal_done_future: Future = goal_handle.get_result_async()
        nav_goal_done_future.add_done_callback(self.sm.on_nav_done)
        self.sm.change_state(ROBOT_STATE.MOVING)
            


class MovingState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    def init(self):
        pass
    def deinit(self):
        pass
    def on_nav_done(self, future: Future):
        nav_goal_result: GoalStatus = future.result().status
        self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.sm.change_state(ROBOT_STATE.PROCESSING)
        else:
            self.sm.change_state(ROBOT_STATE.ERROR)

class ProcessingState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        if self.sm.current_task is None:
            self.sm.change_state(ROBOT_STATE.ERROR)
        self.timer = self.sm.create_timer(self.sm.current_task.process_time, self.timer_cb)

    def deinit(self):
        self.timer.cancel()
    
    def timer_cb(self):
        self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)

class ErrorState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    def init(self):
        self.recovery_timer = self.sm.create_timer(10, self.recovery_cb)
    def deinit(self):
        self.recovery_timer.cancel()

    def recovery_cb(self):
        self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)