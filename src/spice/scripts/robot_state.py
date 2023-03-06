from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage, NavigateToPose_Feedback

from spice_msgs.msg import Id, RobotType, Layer, Node, Work
from spice_msgs.srv import RegisterRobot, RobotTask, AllocWorkCell

from work_tree import WorkTree, Vertex
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
    def on_allocate_task(self, request: RobotTask.Request, response: RobotTask.Response) -> RobotTask.Response:
        response.job_accepted = False
        return response
    

class StartUpState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        self.sm.get_logger().info('init StartUpState')
        self.nav_stack_is_active = False
        self.registered_robot = False
        self.timer = self.sm.create_timer(1, self.try_initialize)

        self.register_robot_client = self.sm.create_client(RegisterRobot, '/register_robot')
        self.navigation_is_active_client = self.sm.create_client(Trigger, 'lifecycle_manager_navigation/is_active')

        self.register_future = None
        self.nav_stack_is_active_future = None

        # ensure no heartbeat in this state
        self.sm.heartbeat_timer.cancel()

    def try_initialize(self):
        if not self.nav_stack_is_active:
            self.check_nav2_stack_status()
        elif not self.registered_robot:
            self.register_robot()
        else: # nav_stack is good, and we are registered
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)

    def check_nav2_stack_status(self):
        self.sm.get_logger().info('wait for service: lifecycle_manager_navigation/is_active')
        while not self.navigation_is_active_client.wait_for_service(1):
            self.sm.get_logger().info('timeout on wait for service: lifecycle_manager_navigation/is_active')
        
        self.nav_stack_is_active_future = self.navigation_is_active_client.call_async(Trigger.Request())
        self.nav_stack_is_active_future.add_done_callback(self.nav_stack_is_active_cb)

    def nav_stack_is_active_cb(self, future: Future):
        result: Trigger.Response = future.result()
        if result.success:
            self.nav_stack_is_active = True

    def register_robot(self):
        register_robot_request = RegisterRobot.Request()
        register_robot_request.id = self.sm.id
        if not self.register_robot_client.wait_for_service(1):
            self.sm.get_logger().info('Robot StartUpState timeout for /register_robot service')
            return

        self.register_future = self.register_robot_client.call_async(register_robot_request)
        self.register_future.add_done_callback(self.register_robot_done_callback)


    def register_robot_done_callback(self, future: Future):
        response: RegisterRobot.Response = future.result()
        if response.success:
            self.registered_robot = True
        else:
            self.sm.get_logger().info('Failed to register robot, is it already registered?')
            

    def deinit(self):
        if self.register_future:
            if not self.register_future.cancelled():
                self.register_future.cancel()
        if self.nav_stack_is_active_future:
            if not self.nav_stack_is_active_future.cancelled():
                self.nav_stack_is_active_future.cancel()
        self.timer.destroy()
        self.register_robot_client.destroy()
        self.navigation_is_active_client.destroy()


class ReadyForJobState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
        
    def init(self):
        self.sm.heartbeat_timer.reset()
        self.sm.heartbeat_future = None

        self.nav_response_future = None
        self.nav_goal_done_future = None

    def deinit(self):
        pass

    def on_allocate_task(self, request: RobotTask.Request, response: RobotTask.Response) -> RobotTask.Response:
        if self.sm.current_task is not None:
            response.job_accepted = False
            self.sm.get_logger().warn("Got a new task, but a task is already allocated, \
                                       ignoring new task")
            return response
        
        if not self.sm.navigation_client.wait_for_server(5):
            response.job_accepted = False
            self.sm.get_logger().warn("Got a new task, but received timeout \
                                      on wait for navigation client server, \
                                      ignoring the task")
            return response
        
        self.sm.taskTree = WorkTree(request.task.layers) # create task tree of robot task
                   
        #self.sm.root_vertex = taskTree.get_root()       
        #self.sm.get_logger().info(f"root vertex: {self.sm.root_vertex}") #debug
        response.job_accepted = True
        self.sm.change_state(ROBOT_STATE.FIND_WORKCELL)
        return response
    
    def nav_goal_response_cb(self, future: Future): ## Remove?
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting task')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)
        self.sm.change_state(ROBOT_STATE.MOVING)


class FindWorkCell(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        self.current_work = self.sm.taskTree.next_task()
        if not self.current_work: # no more work
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)
            return

        self.work_cell_allocator_client = self.sm.create_client(AllocWorkCell, "/allocate_work_cell")
        self.alloc_workcell()
    
    def alloc_workcell(self):
        alloc_workcell_request = AllocWorkCell.Request()
        alloc_workcell_request.robot_id = self.sm.id
        
        for child in self.current_work.children_:
            alloc_workcell_request.robot_types.append(child.work_type)

        if not self.work_cell_allocator_client.wait_for_service(timeout_sec=1.0):
            self.sm.get_logger().info("workcell allocator not available")
            return

        self.register_future = self.work_cell_allocator_client.call_async(alloc_workcell_request)
        self.register_future.add_done_callback(self.alloc_workcell_done_callback)

    def alloc_workcell_done_callback(self, future: Future):
        response: AllocWorkCell.Response = future.result()
        
        if response.found_job:
            ## call nav go
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = response.goal_pose
            self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
                nav_goal, self.sm.on_nav_feedback)
            self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)
            self.sm.get_logger().info("trying to move to workcell")
            
        else:
            self.sm.get_logger().info('Failed to allocate workcell to robot, are they available?')
            

    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting task')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)
        self.sm.change_state(ROBOT_STATE.MOVING)

    def deinit(self):
        self.work_cell_allocator_client.destroy()




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
            self.sm.current_task = None
            self.sm.change_state(ROBOT_STATE.ERROR)
    def on_nav_feedback(self, msg: NavigateToPose_FeedbackMessage):
        # feedback: NavigateToPose_Feedback = msg.feedback
        # self.sm.get_logger().info(str(feedback))
        pass


class ProcessingState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        if self.sm.current_task is None:
            self.sm.get_logger().warn('Entered ProcessingState with no task available!')
            self.sm.change_state(ROBOT_STATE.ERROR)
        self.timer = self.sm.create_timer(self.sm.current_task.process_time, self.timer_cb)

    def deinit(self):
        if self.timer:
            self.timer.cancel()
    
    def timer_cb(self):
        self.sm.current_task = None
        self.sm.change_state(ROBOT_STATE.FIND_WORKCELL)

class ErrorState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
        self.recovery_timer = self.sm.create_timer(10, self.recovery_cb)
        self.recovery_timer.cancel()

    def init(self):
        self.recovery_timer.reset()
        
    def deinit(self):
        self.recovery_timer.cancel()

    def recovery_cb(self):
        self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)