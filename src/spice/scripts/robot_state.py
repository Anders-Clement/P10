import enum

from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage, NavigateToPose_Feedback

from spice_msgs.msg import Id, RobotType, Layer, Node, Work
from spice_msgs.srv import RegisterRobot, RobotTask, AllocWorkCell, RegisterWork

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
        self.current_work = self.sm.taskTree.next_task(lastWorkType=self.sm.current_task.robot_type)
        if not self.current_work: # no more work
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)
            return

        self.work_cell_allocator_client = self.sm.create_client(AllocWorkCell, "/allocate_work_cell")
        self.alloc_workcell()
    
    def alloc_workcell(self):
        alloc_workcell_request = AllocWorkCell.Request()
        alloc_workcell_request.robot_id = self.sm.id
        
        for work in self.current_work:
            alloc_workcell_request.robot_types.append(work.work_type)

        if not self.work_cell_allocator_client.wait_for_service(timeout_sec=5.0):
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
            self.sm.current_task = response.workcell_id
            self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
                nav_goal, self.sm.on_nav_feedback)
            self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)
            
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

    class ProcessStates(enum.IntEnum):
        REGISTER_WORK = 0
        WAIT_IN_QUEUE = 1
        READY_FOR_PROCESS = 2
        PROCESSING = 3
        EXIT_WC = 4
        ERROR = 99

    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        if self.sm.current_task is None:
            self.sm.get_logger().warn('Entered ProcessingState with no task available!')
            self.sm.change_state(ROBOT_STATE.ERROR)

        self.processing_states(state=0)

   
    def processing_states(self, state):
        match state:
            case self.ProcessStates.REGISTER_WORK:
                self.process_state = self.ProcessStates.REGISTER_WORK
                self.register_work_client = self.sm.create_client(
                    RegisterWork, self.sm.current_task.id+'/register_work')
                self.register_work()
            
            case self.ProcessStates.WAIT_IN_QUEUE:
                self.process_state = self.ProcessStates.WAIT_IN_QUEUE #enterWorkCell = True
                self.srv_call_robot = self.sm.create_service(
                    Trigger, self.sm.id.id + "/call_robot", self.call_robot_cb)
                                       
            case self.ProcessStates.READY_FOR_PROCESS:
                self.process_state = self.ProcessStates.READY_FOR_PROCESS
                self.robot_ready_process_client = self.sm.create_client(
                    Trigger, self.sm.current_task.id + "/robot_ready_for_processing")
                self.robot_ready_process() #readyforprocessing = True

            case self.ProcessStates.PROCESSING:
                self.process_state = self.ProcessStates.PROCESSING  #waitforProcessing = True
                self.srv_done_processing = self.sm.create_service(
                    Trigger, self.sm.id + "/done_processing", self.done_processing_cb)   
                   
            case self.ProcessStates.EXIT_WC:
                self.process_state = self.ProcessStates.EXIT_WC
                self.sm.change_state(ROBOT_STATE.FIND_WORKCELL)  #doneProcessing = True
                                               
            case self.ProcessStates.ERROR:
                self.process_state = self.ProcessStates.ERROR  #thingsAreWorking = False
                self.sm.change_state(ROBOT_STATE.ERROR)
                                                

    def register_work(self):
        register_work_request = RegisterWork.Request()
        register_work_request.robot_id = self.sm.id
        register_work_request.work.type = self.sm.current_task.robot_type
        register_work_request.work.info = "register work pls"

        if not self.register_work_client.wait_for_service(timeout_sec=1.0):
            self.sm.get_logger().info("register work server not avialable")
            return
        
        self.register_work_future = self.register_work_client.call_async(register_work_request)
        self.register_work_future.add_done_callback(self.register_work_cb)
    

    def register_work_cb(self, future:Future):
        response : RegisterWork.Response = future.result()

        if response.work_is_enqueued:
            self.processing_states(state=self.ProcessStates.WAIT_IN_QUEUE)
        else:
            self.processing_states(state=self.ProcessStates.ERROR)


    def call_robot_cb(self, request:Trigger.Request, response:Trigger.Response) -> Trigger.Response:
        
        if self.process_state == self.ProcessStates.WAIT_IN_QUEUE:
            response = True
            #TODO: Navigate to cell entrance
            self.processing_states(state=self.ProcessStates.READY_FOR_PROCESS)

        else:
            response = False
        
        return response


    def robot_ready_process(self):
        robot_ready_process_request = Trigger.Request()

        if not self.robot_ready_process_client.wait_for_service(timeout_sec=1.0):
            self.sm.get_logger().info("robot ready for process server not avialable")
            return
        
        self.robot_ready_process_future = self.register_work_client.call_async(robot_ready_process_request)
        self.robot_ready_process_future.add_done_callback(self.robot_ready_process_done_cb)


    def robot_ready_process_done_cb(self, future:Future):
        response : Trigger.Response = future.result()

        if(response):
            self.processing_states(state=self.ProcessStates.PROCESSING)
        else:
            self.processing_states(state=self.ProcessStates.ERROR)


    def done_processing_cb(self):
        if(self.process_state == self.ProcessStates.PROCESSING):
            response = True
            self.processing_states(state=self.ProcessStates.EXIT_WC)
        else:
            response = False
        return response
    

    def deinit(self):

        if self.register_work_future:
            if not self.register_work_future.cancelled():
                self.register_work_future.cancel()
        
        if self.robot_ready_process_future:
            if not self.robot_ready_process_future.cancelled():
                   self.robot_ready_process_future.cancel()
        
        self.register_work_client.destroy()
        self.robot_ready_process_client.destroy()
        self.srv_call_robot.destroy()
        self.srv_done_processing.destroy()


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