import enum

from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage, NavigateToPose_Feedback

from spice_msgs.msg import PlannerType
from spice_msgs.srv import RegisterRobot, RobotTask, AllocWorkCell, RegisterWork, SetPlannerType

from work_tree import WorkTree
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
        self.set_planner_type_ready = False
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
        elif not self.set_planner_type_ready:
            self.wait_for_planner_type_service()
        else: # nav_stack is good, and we are registered
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)

    def check_nav2_stack_status(self):
        self.sm.get_logger().info('wait for service: lifecycle_manager_navigation/is_active')
        while not self.navigation_is_active_client.wait_for_service(10):
            self.sm.get_logger().info('timeout on wait for service: lifecycle_manager_navigation/is_active')
        
        self.nav_stack_is_active_future = self.navigation_is_active_client.call_async(Trigger.Request())
        self.nav_stack_is_active_future.add_done_callback(self.nav_stack_is_active_cb)

    def nav_stack_is_active_cb(self, future: Future):
        result: Trigger.Response = future.result()
        if result.success:
            self.nav_stack_is_active = True

    def wait_for_planner_type_service(self):
        self.set_planner_type_ready = self.sm.change_planner_type_client.wait_for_service(10.0)
        if not self.set_planner_type_ready:
            self.sm.get_logger().info('timeout on wait for service: set_planner_type')

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
        if self.sm.task_tree is not None:
            self.sm.get_logger().warn('Entering ready_for_job state, but task tree was not None')
        self.sm.task_tree = None

        self.nav_response_future = None
        self.nav_goal_done_future = None

    def deinit(self):
        pass

    def on_allocate_task(self, request: RobotTask.Request, response: RobotTask.Response) -> RobotTask.Response:
        if self.sm.task_tree is not None:
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
        
        self.sm.task_tree = WorkTree(request.task.layers) # create task tree of robot task
                   
        response.job_accepted = True
        self.sm.change_state(ROBOT_STATE.FIND_WORKCELL)
        return response


class FindWorkCell(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        self.sm.current_work = self.sm.task_tree.get_next_work_types() 

        if len(self.sm.current_work) == 0: # no more work
            self.sm.task_tree = None
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)
            return

        self.work_cell_allocator_client = self.sm.create_client(AllocWorkCell, "/allocate_work_cell")
        self.alloc_workcell()
    
    def alloc_workcell(self):
        alloc_workcell_request = AllocWorkCell.Request()
        alloc_workcell_request.robot_id = self.sm.id
        
        alloc_workcell_request.robot_types = self.sm.current_work

        if not self.work_cell_allocator_client.wait_for_service(timeout_sec=5.0):
            self.sm.get_logger().warn("workcell allocator not available")
            self.sm.change_state(ROBOT_STATE.ERROR)
            return

        self.register_future = self.work_cell_allocator_client.call_async(alloc_workcell_request)
        self.register_future.add_done_callback(self.alloc_workcell_done_callback)

    def alloc_workcell_done_callback(self, future: Future):
        response: AllocWorkCell.Response = future.result()
        
        if response.found_job:
            self.sm.task_tree.select_next_work_type(response.workcell_id.robot_type)
            self.sm.current_task = response

            set_planner_type_request = SetPlannerType.Request()
            set_planner_type_request.planner_type = PlannerType(type=PlannerType.PLANNER_PRIORITIZED)
            change_planner_type_future = self.sm.change_planner_type_client.call_async(set_planner_type_request)
            change_planner_type_future.add_done_callback(self.navigate_to_goal)
            
        else:
            self.sm.get_logger().info('Failed to allocate workcell to robot, are they available?')
            self.sm.change_state(ROBOT_STATE.ERROR)

    def navigate_to_goal(self, future: Future):
        result: SetPlannerType.Response = future.result()
        if not result.success:
            self.sm.get_logger().warn('Failed to change planner type')
            self.sm.change_state(ROBOT_STATE.ERROR)
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.sm.current_task.goal_pose
        self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback)
        self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)
            

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
            self.sm.change_state(ROBOT_STATE.REGISTER_WORK)
        else:
            self.sm.get_logger().info('Failed navigation, going to ERROR!')
            self.sm.change_state(ROBOT_STATE.ERROR)
    def on_nav_feedback(self, msg: NavigateToPose_FeedbackMessage):
        # feedback: NavigateToPose_Feedback = msg.feedback
        # self.sm.get_logger().info(str(feedback))
        pass


class ProcessRegisterWorkState(RobotStateTemplate):

    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        if self.sm.current_task is None:
            self.sm.get_logger().warn('Entered ProcessingState with no task available!')
            self.sm.change_state(ROBOT_STATE.ERROR)
        self.register_work_client = self.sm.create_client(
                    RegisterWork, '/'+self.sm.current_task.workcell_id.id+'/register_work')
        self.register_work_future = None
        self.register_work()


    def register_work(self):
        register_work_request = RegisterWork.Request()
        register_work_request.robot_id = self.sm.id
        register_work_request.work.type = self.sm.current_task.workcell_id.robot_type
        register_work_request.work.info = "register work pls"

        if not self.register_work_client.wait_for_service(timeout_sec=1.0):
            self.sm.get_logger().info("register work server not avialable")
            return
        
        self.register_work_future = self.register_work_client.call_async(register_work_request)
        self.register_work_future.add_done_callback(self.register_work_cb)


    def register_work_cb(self, future:Future):
        self.sm.get_logger().info(self.sm.id.id+  ' regisiter_work_cb')
        response : RegisterWork.Response = future.result()
        if response.work_is_enqueued:
            self.sm.current_work_cell_info = response
            self.sm.change_state(ROBOT_STATE.WAIT_IN_QUEUE)
        else:
            self.sm.get_logger().info("Could not register work at work cell, going to ERROR!")
            self.sm.change_state(ROBOT_STATE.ERROR)
        

    def deinit(self):

        if self.register_work_future:
            if not self.register_work_future.cancelled():
                self.register_work_future.cancel()
        
        self.register_work_client.destroy()


class ProcessWaitQueueState(RobotStateTemplate):
    
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    
    def init(self):
        self.robot_is_called = False
        self.srv_call_robot = self.sm.create_service(
                    Trigger, 'call_robot', self.call_robot_cb)
        self.timer = self.sm.create_timer(0.1, self.check_service_cb)
    
    def check_service_cb(self):
        if self.robot_is_called:
            self.sm.change_state(ROBOT_STATE.ENTER_WORKCELL)
    
    def call_robot_cb(self, request:Trigger.Request, response:Trigger.Response) -> Trigger.Response:
        if self.sm.current_state != ROBOT_STATE.WAIT_IN_QUEUE:
            response.success = False
            self.sm.get_logger().warn('call_robot_cb, but ROBOT_STATE is not ROBOT_STATE.WAIT_IN_QUEUE')
            return response
        
        if self.robot_is_called:
            response.success = False
            self.sm.get_logger().warn('call_robot_cb, but robot is already called')
            return response
        
        self.sm.get_logger().info(self.sm.id.id+  ' call_robot_cb') 
        response.success = True
        self.robot_is_called = True
        return response
    
    def deinit(self):
        
        self.timer.cancel()
        self.srv_call_robot.destroy()


class EnterWorkCellState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        change_planner_type_request = SetPlannerType.Request()
        change_planner_type_request.planner_type.type = PlannerType.PLANNER_STRAIGHT_LINE
        change_planner_type_future = self.sm.change_planner_type_client.call_async(change_planner_type_request)
        change_planner_type_future.add_done_callback(self.navigate_into_cell)

    def navigate_into_cell(self, future: Future):
        result: SetPlannerType.Response = future.result()
        if not result.success:
            self.sm.get_logger().warn('Failed to change planner type')
            self.sm.change_state(ROBOT_STATE.ERROR)
            return

        current_work_cell_info : RegisterWork.Response = self.sm.current_work_cell_info

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = current_work_cell_info.processing_pose
        self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback)
        self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting enter work cell')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)

    def on_nav_done(self, future: Future):
        nav_goal_result: GoalStatus = future.result().status
        self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.sm.change_state(ROBOT_STATE.READY_FOR_PROCESS)
        else:
            self.sm.change_state(ROBOT_STATE.ERROR)

    def deinit(self):
        pass
            

class ProcessReadyForProcessingState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    
    def init(self):
        self.robot_ready_process_client = self.sm.create_client(
                    Trigger, '/'+self.sm.current_task.workcell_id.id + "/robot_ready_for_processing")
        self.timer = self.sm.create_timer(5.0, self.robot_ready_process)
        self.robot_ready_process_future = None

    def robot_ready_process(self):
        if not self.robot_ready_process_client.wait_for_service(timeout_sec=1.0):
            self.sm.get_logger().info("robot ready for process server not avialable")
            return
                
        self.robot_ready_process_future = self.robot_ready_process_client.call_async(Trigger.Request())
        self.robot_ready_process_future.add_done_callback(self.robot_ready_process_done_cb)

    def robot_ready_process_done_cb(self, future:Future):

        if self.sm.current_state != ROBOT_STATE.READY_FOR_PROCESS:
            response.success = False
            self.sm.get_logger().warn('robot_ready_process_done_cb, but ROBOT_STATE is not ROBOT_STATE.READY_FOR_PROCESS')
            return
        
        response : Trigger.Response = future.result()
        
        self.sm.get_logger().info( 'robot_ready_process_done_cb reponse: ' + response.success.__str__())
        if(response.success):
            self.sm.change_state(ROBOT_STATE.PROCESS_DONE) # change state to processing
        
            self.sm.get_logger().info(self.sm.id.id+  ' robot_ready_process_done_cb')


    def deinit(self):
        if self.robot_ready_process_future:
            if not self.robot_ready_process_future.cancelled():
                self.robot_ready_process_future.cancel()
        self.robot_ready_process_client.destroy()
        self.timer.cancel()


class ProcessProcessingDoneState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    
    def init(self):
        self.processing_is_done = False
        self.srv_done_processing = self.sm.create_service(
                    Trigger, 'robot_done_processing', self.done_processing_cb)
            
        self.timer = self.sm.create_timer(0.1, self.check_service_cb)
        
    def done_processing_cb(self,request:Trigger.Request, response:Trigger.Response) -> Trigger.Response:
        if self.sm.current_state != ROBOT_STATE.PROCESS_DONE:
            response.success = False
            self.sm.get_logger().warn('done_processing_cb, but ROBOT_STATE is not ROBOT_STATE.PROCESS_DONE')
            return response
        
        if self.processing_is_done:
            response.success = False
            self.sm.get_logger().warn('done processing but robot is already called')
            return response

        self.sm.get_logger().info(self.sm.id.id+  ' done_processing_cb')

        response.success = True
        self.processing_is_done = True
        return response  

    
    def check_service_cb(self):
        if self.processing_is_done:
            self.sm.change_state(ROBOT_STATE.EXIT_WORKCELL)
    
   
    def deinit(self):
        self.timer.cancel()
        self.srv_done_processing.destroy()


class ProcessExitWorkCellState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    
    def init(self):
        self.sm.get_logger().info(self.sm.id.id+  ' is done processing at ' + self.sm.current_task.workcell_id.id + ' exiting work cell')
        change_planner_type_request = SetPlannerType.Request()
        change_planner_type_request.planner_type.type = PlannerType.PLANNER_STRAIGHT_LINE
        change_planner_type_future = self.sm.change_planner_type_client.call_async(change_planner_type_request)
        change_planner_type_future.add_done_callback(self.navigate_exit_cell)

    def navigate_exit_cell(self, future: Future):
        result: SetPlannerType.Response = future.result()
        if not result.success:
            self.sm.get_logger().warn('Failed to change planner type')
            self.sm.change_state(ROBOT_STATE.ERROR)
            return
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.sm.current_work_cell_info.exit_pose
        self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback)
        self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting exit work cell')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)

    def on_nav_done(self, future: Future):
        nav_goal_result: GoalStatus = future.result().status
        self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.sm.change_state(ROBOT_STATE.FIND_WORKCELL)
        else:
            self.sm.current_task = None
            self.sm.change_state(ROBOT_STATE.ERROR)

    def deinit(self):
        pass

class ErrorState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
        self.recovery_timer = self.sm.create_timer(10, self.recovery_cb)
        self.recovery_timer.cancel()

    def init(self):
        self.recovery_timer.reset()
        # clear job and task tree
        self.sm.current_task = None
        self.sm.task_tree = None
        
    def deinit(self):
        self.recovery_timer.cancel()

    def recovery_cb(self):
        self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)