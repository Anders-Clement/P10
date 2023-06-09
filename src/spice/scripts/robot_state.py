import enum

from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage, NavigateToPose_Feedback

from spice_msgs.msg import PlannerType, QueuePoints, QueuePoint, TaskData
from spice_msgs.srv import RegisterRobot, RobotTask, AllocWorkCell, RegisterWork, SetPlannerType, RobotReady
from spice_mapf_msgs.action import NavigateMapf
from spice_mapf_msgs.action._navigate_mapf import NavigateMapf_FeedbackMessage

from work_tree import WorkTree
from robot_state_manager_node import RobotStateManager, ROBOT_STATE, HeartBeatHandler
from geometry_msgs.msg import PoseStamped
from datetime import *

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
        self.nav_stack_is_active = True
        self.registered_robot = False
        self.set_planner_type_ready = True
        self.set_planner_type = True
        self.timer = self.sm.create_timer(1, self.try_initialize)

        self.register_robot_client = self.sm.create_client(RegisterRobot, '/register_robot')
        self.navigation_is_active_client = self.sm.create_client(Trigger, 'lifecycle_manager_navigation/is_active')

        self.register_future = None
        self.nav_stack_is_active_future = None
        self.set_planner_future = None

        # ensure no heartbeat in this state
        self.sm.heartbeat.deactivate()

    def try_initialize(self):
        if not self.nav_stack_is_active:
            self.check_nav2_stack_status()
        elif not self.registered_robot:
            self.register_robot()
        elif not self.set_planner_type_ready:
            self.wait_for_planner_type_service()
        elif not self.set_planner_type:
            self.set_planner()
        else: # nav_stack is good, and we are registered
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)

    def set_planner(self):
        if self.set_planner_future is not None:
            return
        set_planner_type_request = SetPlannerType.Request()
        set_planner_type_request.planner_type = PlannerType(type=PlannerType.PLANNER_STRAIGHT_LINE)
        self.set_planner_future = self.sm.change_planner_type_client.call_async(set_planner_type_request)
        self.set_planner_future.add_done_callback(self.set_planner_cb)

    def set_planner_cb(self, future: Future):
        result = future.result()
        if result.success:
            self.set_planner_type = True
        else:
            self.sm.get_logger().warn('Failed to set planner type during startup, retrying...')
            self.set_planner_future = None
            self.set_planner()

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
        self.sm.heartbeat.activate()
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
        
        if not self.sm.mapf_navigation_client.wait_for_server(5):
            response.job_accepted = False
            self.sm.get_logger().warn("Got a new task, but received timeout \
                                      on wait for navigation client server, \
                                      ignoring the task")
            return response
        
        self.sm.task_tree = WorkTree(request.task.layers)# create task tree of robot task
        self.msg = TaskData()
        
        self.msg.task_state = TaskData.TASKSTART
        self.msg.robot_id = self.sm.id
        self.msg.task_id = request.task_id
        self.sm.current_task_id = request.task_id


        self.msg.stamp = self.sm.get_clock().now().to_msg()
        self.sm.task_start_time = self.sm.get_clock().now().seconds_nanoseconds()[0]
        self.msg.total_time = 0
        
        self.sm.state_data_pub.publish(self.msg)

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
            self.sm.get_logger().info("Job done, getting ready for a new job")
        
            self.msg = TaskData()
            self.msg.task_state = TaskData.TASKDONE
            self.msg.robot_id = self.sm.id
            self.msg.stamp = self.sm.get_clock().now().to_msg()
            self.msg.total_time = self.sm.get_clock().now().seconds_nanoseconds()[0] - self.sm.task_start_time
            self.msg.task_id = self.sm.current_task_id
            
            self.sm.state_data_pub.publish(self.msg)
            self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)
            return

        self.work_cell_allocator_client = self.sm.create_client(AllocWorkCell, "/allocate_work_cell")
        self.alloc_workcell_timer = self.sm.create_timer(1.0, self.alloc_workcell)
        self.alloc_workcell_timer.cancel() # only used for retries
        self.alloc_workcell()
    
    def alloc_workcell(self):
        self.alloc_workcell_timer.cancel()
        alloc_workcell_request = AllocWorkCell.Request()
        alloc_workcell_request.robot_id = self.sm.id
        
        alloc_workcell_request.robot_types = self.sm.current_work

        if not self.work_cell_allocator_client.wait_for_service(timeout_sec=5.0):
            self.sm.get_logger().warn("workcell allocator not available, retrying...")
            self.alloc_workcell_timer.reset()
            return

        self.register_future = self.work_cell_allocator_client.call_async(alloc_workcell_request)
        self.register_future.add_done_callback(self.alloc_workcell_done_callback)

    def alloc_workcell_done_callback(self, future: Future):
        response: AllocWorkCell.Response = future.result()
        
        if response.found_job:
            self.sm.task_tree.select_next_work_type(response.workcell_id.robot_type)
            self.sm.current_task = response
            self.sm.change_state(ROBOT_STATE.REGISTER_WORK)
            
        else:
            self.sm.get_logger().warn('Failed to allocate workcell to robot, are they available? Retrying...')
            self.alloc_workcell_timer.reset()

    def deinit(self):
        self.work_cell_allocator_client.destroy()


class ProcessRegisterWorkState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        if self.sm.current_task is None:
            self.sm.get_logger().warn('Entered ProcessingState with no task available! Going to ERROR')
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
        self.sm.get_logger().info(self.sm.id.id+  ' register_work_cb')
        response : RegisterWork.Response = future.result()
        if response.work_is_enqueued:
            self.sm.current_work_cell_info = response
            self.sm.work_cell_heartbeat = HeartBeatHandler(
                "/" + self.sm.current_task.workcell_id.id + "/heartbeat",
                2.5,
                self.sm.id,
                lambda arg : arg.change_state(ROBOT_STATE.ERROR),
                self.sm)
            self.sm.work_cell_heartbeat.activate()
            self.sm.change_state(ROBOT_STATE.ENQUEUED)
            
        else:
            self.sm.get_logger().warn(f"Could not register work at work cell: {self.sm.current_task.workcell_id.id}, going to ERROR!")
            self.sm.change_state(ROBOT_STATE.ERROR)       

    def deinit(self):

        if self.register_work_future:
            if not self.register_work_future.cancelled():
                self.register_work_future.cancel()
        
        self.register_work_client.destroy()


class MovingState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        pass

    def deinit(self):
        pass
    

class EnqueuedState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
    
    def init(self):
        self.goal_handle = None
        self.got_queue_points = False
        self.num_navigation_erorrs = 0
        self.MAX_NAVIGATION_RETRIES = 5
        self.ROBOT_READY_AT_CELL_DIST = 1.0 # m

        self.robot_is_ready = False
        self.robot_is_called = False
        self.robot_is_at_queue_point = False
        self.srv_call_robot = self.sm.create_service(
                    Trigger, 'call_robot', self.call_robot_cb)
        
        self.msg = TaskData()

        self.msg.task_state = TaskData.ENQUEUED
        self.msg.robot_id = self.sm.id
        self.msg.stamp = self.sm.get_clock().now().to_msg()
        self.sm.enqueud_start_time = self.sm.get_clock().now().seconds_nanoseconds()[0]
        self.msg.total_time = 0#self.sm.get_clock().now().seconds_nanoseconds()[0] - self.sm.task_start_time
        self.msg.task_id = self.sm.current_task_id

        self.sm.state_data_pub.publish(self.msg)
        
        # set_planner_type_request = SetPlannerType.Request()
        # set_planner_type_request.planner_type = PlannerType(type=PlannerType.PLANNER_STRAIGHT_LINE)
        # change_planner_type_future = self.sm.change_planner_type_client.call_async(set_planner_type_request)
        # change_planner_type_future.add_done_callback(self.set_planner_cb)
        self.goal_update_pub = self.sm.create_publisher(PoseStamped, 'goal_update',10)

        current_task: AllocWorkCell.Response = self.sm.current_task
        queue_points_topic_name = "/" + current_task.workcell_id.id + "/queue_points"
        self.sm.get_logger().info(f'queue_points:topic: {queue_points_topic_name}')
        self.queue_points_sub = self.sm.create_subscription(
            QueuePoints, 
            queue_points_topic_name,
            self.queue_points_cb,
            10
            )
        
        self.timer = self.sm.create_timer(0.1, self.check_service_cb) ##What is going on here??
        self.timer.cancel()

        #self.nav2queue_timer = self.sm.create_timer(0.5, self.navigate_to_queue_point)

        self.navigate_to_queue_point()


        
    def queue_points_cb(self, msg: QueuePoints) -> None:
        for queue_point in msg.queue_points:
            queue_point : QueuePoint = queue_point
            current_work_cell_info : RegisterWork.Response = self.sm.current_work_cell_info
            if queue_point.queue_robot_id == self.sm.id:
                # self.sm.get_logger().info(f'new queue point: {queue_point.queue_transform.translation}')
                current_work_cell_info.queue_pose.pose.position.x = queue_point.queue_transform.translation.x
                current_work_cell_info.queue_pose.pose.position.y = queue_point.queue_transform.translation.y
                current_work_cell_info.queue_pose.pose.position.z = queue_point.queue_transform.translation.z
                current_work_cell_info.queue_pose.pose.orientation.x = queue_point.queue_transform.rotation.x
                current_work_cell_info.queue_pose.pose.orientation.y = queue_point.queue_transform.rotation.y
                current_work_cell_info.queue_pose.pose.orientation.z = queue_point.queue_transform.rotation.z
                current_work_cell_info.queue_pose.pose.orientation.w = queue_point.queue_transform.rotation.w

                self.robot_is_at_queue_point = False
                # self.update_nav_goal()
                if self.goal_handle is not None:
                    self.got_queue_points = True
                else:
                    self.navigate_to_queue_point()
                
                
                #self.num_navigation_erorrs -= 1
                #self.navigate_to_queue_point()
                return
            
    # def set_planner_cb(self, future: Future):
    #     result: SetPlannerType.Response = future.result()
    #     if not result.success:
    #         self.sm.get_logger().warn('Failed to change planner type, going to ERROR')
    #         self.sm.change_state(ROBOT_STATE.ERROR)
    #         return
    #     self.navigate_to_queue_point()

    def navigate_to_queue_point(self):
        if not self.robot_is_at_queue_point:
            # TODO: NAV
            nav_goal = NavigateMapf.Goal()
            nav_goal.goal_pose = self.sm.current_work_cell_info.queue_pose
            nav_goal.workcell_id = self.sm.current_task.workcell_id
            self.nav_response_future = self.sm.mapf_navigation_client.send_goal_async(
                nav_goal,
                self.sm.on_nav_feedback
            )
            self.nav_response_future.add_done_callback(self.nav_goal_response_cb)
            # nav_goal = NavigateToPose.Goal()
            # nav_goal.pose = self.sm.current_work_cell_info.queue_pose
            # self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
            #     nav_goal,
            #     self.sm.on_nav_feedback)
            # self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)

    # def update_nav_goal(self):
    #     msg = PoseStamped()
    #     msg.header.frame_id = "map"
    #     msg.header.stamp = self.sm.get_clock().now().to_msg()  #datetime.now()
    #     msg.pose = self.sm.current_work_cell_info.queue_pose.pose
    #     self.goal_update_pub.publish(msg)

    def nav_goal_response_cb(self, future: Future):
        self.goal_handle: ClientGoalHandle = future.result()
        if not self.goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting task. Going to ERROR')
            self.sm.change_state(ROBOT_STATE.ERROR)
        self.nav_goal_done_future: Future = self.goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)

    def on_nav_feedback(self, msg: NavigateMapf_FeedbackMessage):
        pass
        # TODO: NAV
        # should only be 0.0 when actually at goal
        # if msg.feedback.distance_to_goal < self.ROBOT_READY_AT_CELL_DIST:
        #     self.call_robot_ready_in_queue()

        # can be 0 at very start of a navigation task
        # if msg.feedback.distance_remaining == 0.0:
        #     return
        # if msg.feedback.distance_remaining < self.ROBOT_READY_AT_CELL_DIST:
        #     self.call_robot_ready_in_queue()

    def call_robot_ready_in_queue(self):  
        self.got_queue_points = False
        if self.robot_is_ready:
            return
        
        self.robot_is_ready = True
        robot_ready_request = RobotReady.Request()
        robot_ready_request.robot_id = self.sm.id
        current_task: AllocWorkCell.Response = self.sm.current_task
        robot_ready_service_name = '/' + current_task.workcell_id.id + "/robot_ready_in_queue"
        self.robot_ready_client = self.sm.create_client(RobotReady, robot_ready_service_name)
        if not self.robot_ready_client.wait_for_service(5.0):
            self.sm.get_logger().error('Timeout on wait for service: ' + robot_ready_service_name)
            self.sm.change_state(ROBOT_STATE.ERROR)
        robot_ready_future = self.robot_ready_client.call_async(robot_ready_request)
        robot_ready_future.add_done_callback(self.robot_ready_cb)

    def on_nav_done(self, future: Future):
        self.goal_handle = None
        nav_goal_result: GoalStatus = future.result().status
        #self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.robot_is_at_queue_point = True
            self.call_robot_ready_in_queue()
            if self.got_queue_points:
                self.navigate_to_queue_point()
        else:
            self.num_navigation_erorrs += 1
            self.sm.get_logger().warn(f'Failed navigation, number of tries: {self.num_navigation_erorrs}/{self.MAX_NAVIGATION_RETRIES}')
            if self.num_navigation_erorrs > self.MAX_NAVIGATION_RETRIES:
                self.sm.get_logger().warn(f'Too many navigation failures {self.num_navigation_erorrs}/{self.MAX_NAVIGATION_RETRIES}, going to ERROR')
                self.sm.change_state(ROBOT_STATE.ERROR)
                return
            self.navigate_to_queue_point()

    def robot_ready_cb(self, future: Future):
        result: RobotReady.Response = future.result()
        if not result.success:
            self.sm.get_logger().warn("Failed to call robot ready at work cell queue")
            self.sm.change_state(ROBOT_STATE.ERROR)
        else:
            self.timer.reset()        
    
    def check_service_cb(self):
        if self.robot_is_called and self.goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.timer.cancel()
            self.sm.change_state(ROBOT_STATE.ENTER_WORKCELL)
    
    def call_robot_cb(self, request:Trigger.Request, response:Trigger.Response) -> Trigger.Response:
        if self.sm.current_state != ROBOT_STATE.ENQUEUED:
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
        #self.nav2queue_timer.cancel()
        return response
    
    def deinit(self):
        self.timer.destroy()
        self.srv_call_robot.destroy()
        #self.nav2queue_timer.destroy()
        self.sm.destroy_subscription(self.queue_points_sub)
        self.sm.destroy_publisher(self.goal_update_pub)


class EnterWorkCellState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm

    def init(self):
        # change_planner_type_request = SetPlannerType.Request()
        # change_planner_type_request.planner_type.type = PlannerType.PLANNER_STRAIGHT_LINE
        # change_planner_type_future = self.sm.change_planner_type_client.call_async(change_planner_type_request)
        # change_planner_type_future.add_done_callback(self.navigate_to_cell_entry)
        self.robot_exited_client = self.sm.create_client(Trigger, '/'+self.sm.current_task.workcell_id.id + "/robot_exited")


        self.num_navigation_errors_entry = 0
        self.num_navigation_errors_center = 0
        self.MAX_NAVIGATION_RETRIES_ENTRY = 5
        self.MAX_NAVIGATION_RETRIES_CENTER = 5
                            
        self.msg = TaskData()
        self.msg.task_state = TaskData.ENTERWORKCELL
        self.msg.robot_id = self.sm.id
        self.msg.stamp = self.sm.get_clock().now().to_msg()
        self.msg.total_time = self.sm.get_clock().now().seconds_nanoseconds()[0] - self.sm.enqueud_start_time
        self.msg.task_id = self.sm.current_task_id
        self.sm.state_data_pub.publish(self.msg)

        self.navigate_to_cell_entry()

    def navigate_to_cell_entry(self):
        # result: SetPlannerType.Response = future.result()
        # if not result.success:
        #     self.sm.get_logger().warn('Failed to change planner type')
        #     self.sm.change_state(ROBOT_STATE.ERROR)
        #     return

        # # TODO: NAV
        current_work_cell_info : RegisterWork.Response = self.sm.current_work_cell_info
        nav_goal = NavigateMapf.Goal()
        nav_goal.workcell_id = self.sm.current_task.workcell_id
        nav_goal.goal_pose = current_work_cell_info.entry_pose
        self.nav_response_future = self.sm.mapf_navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback)
        self.nav_response_future.add_done_callback(self.cell_entry_nav_goal_response_cb)
        # nav_goal = NavigateToPose.Goal()
        # nav_goal.pose = current_work_cell_info.entry_pose
        # self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
        #     nav_goal, self.sm.on_nav_feedback)
        # self.nav_reponse_future.add_done_callback(self.cell_entry_nav_goal_response_cb)

    def cell_entry_nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting going to entry of work cell')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.on_cell_entry_nav_done)

    def on_cell_entry_nav_done(self, future: Future):
        nav_result = future.result()
        nav_goal_result: GoalStatus = nav_result.status
        self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            # change_planner_type_request = SetPlannerType.Request()
            # change_planner_type_request.planner_type.type = PlannerType.PLANNER_STRAIGHT_LINE
            # change_planner_type_future = self.sm.change_planner_type_client.call_async(change_planner_type_request)
            # change_planner_type_future.add_done_callback(self.change_planner_cb)
            self.navigate_into_cell()
        else:
            self.sm.get_logger().warn(f'Goal status not succeeded to go to entry of work cell')#, number of tries: {self.num_navigation_errors_entry}/{self.MAX_NAVIGATION_RETRIES_ENTRY}')
          
            #self.sm.get_logger().error(f'Too many failures to go to entry of work cell: {self.num_navigation_errors_entry}/{self.MAX_NAVIGATION_RETRIES_ENTRY}, going to ERROR')
            self.call_robot_exited_cell()
            # self.sm.change_state(ROBOT_STATE.ERROR)
            return

            # self.sm.get_logger().warn('Goal status not succeeded')
            # self.sm.change_state(ROBOT_STATE.ERROR)

    def call_robot_exited_cell(self):
        robot_exited_request = Trigger.Request()
        robot_exited_future = self.robot_exited_client.call_async(robot_exited_request)
        robot_exited_future.add_done_callback(self.robot_exited_cb)

    def robot_exited_cb(self, future: Future):
        result: Trigger.Response = future.result()
        
        if result.success:
            self.sm.get_logger().error('failed to go to entry of work cell:')
            if self.sm.work_cell_heartbeat is not None:
                self.sm.work_cell_heartbeat.deactivate()
            self.sm.change_state(ROBOT_STATE.ERROR)

        elif self.num_navigation_errors_entry > self.MAX_NAVIGATION_RETRIES_ENTRY:
            self.sm.get_logger().error(f'failed to go to entry of work cell: {self.num_navigation_errors_entry}/{self.MAX_NAVIGATION_RETRIES_ENTRY}, going to ERROR')
            if self.sm.work_cell_heartbeat is not None:
                self.sm.work_cell_heartbeat.deactivate()
            self.sm.change_state(ROBOT_STATE.ERROR)

        else:
            self.call_robot_exited_cell()
            self.num_navigation_errors_entry +=1

    # def change_planner_cb(self, future: Future):
    #     result: SetPlannerType.Response = future.result()
    #     if not result.success:
    #         self.sm.get_logger().warn('Failed to change planner type, going to ERROR')
    #         self.sm.change_state(ROBOT_STATE.ERROR)
    #         return
    #     self.navigate_into_cell()

    def navigate_into_cell(self):
        if self.sm.current_work_cell_info is None or self.sm.current_task is None:
            self.sm.get_logger().error(f'Task or workcell info is None, but should be present')
            self.sm.change_state(ROBOT_STATE.ERROR)
        elif self.sm.current_task.workcell_id is None:
            self.sm.get_logger().error(f'current_task.workcell_id is None, but should be present')
            self.sm.change_state(ROBOT_STATE.ERROR)

        current_work_cell_info : RegisterWork.Response = self.sm.current_work_cell_info

        # TODO: NAV
        nav_goal = NavigateMapf.Goal()
        nav_goal.goal_pose = current_work_cell_info.processing_pose
        nav_goal.workcell_id = self.sm.current_task.workcell_id
        self.nav_response_future = self.sm.mapf_navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback
        )
        self.nav_response_future.add_done_callback(self.nav_goal_response_cb)
        # nav_goal = NavigateToPose.Goal()
        # nav_goal.pose = current_work_cell_info.processing_pose
        # self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
        #     nav_goal, self.sm.on_nav_feedback)
        # self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().error('Nav 2 goal was rejected, aborting going to center of work cell')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)

    def on_nav_done(self, future: Future):
        nav_goal_result: GoalStatus = future.result().status
        self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.sm.change_state(ROBOT_STATE.READY_FOR_PROCESS)
        else:
            self.num_navigation_errors_center += 1
            self.sm.get_logger().info(f'Failed navigation to center of work cell, number of tries: {self.num_navigation_errors_center}/{self.MAX_NAVIGATION_RETRIES_CENTER}')
            if self.num_navigation_errors_center > self.MAX_NAVIGATION_RETRIES_CENTER:
                self.sm.get_logger().warn(f'Too many navigation failures to center of work cell: {self.num_navigation_errors_center}/{self.MAX_NAVIGATION_RETRIES_CENTER}, going to ERROR')
                self.sm.change_state(ROBOT_STATE.ERROR)
                return
            
            self.navigate_into_cell()

            # pass
            # self.sm.get_logger().error('Nav 2 goal failed, aborting enter work cell')
            # self.sm.change_state(ROBOT_STATE.ERROR)

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
        self.robot_exited_client = self.sm.create_client(Trigger, '/'+self.sm.current_task.workcell_id.id + "/robot_exited")
        
        # change_planner_type_request = SetPlannerType.Request()
        # change_planner_type_request.planner_type.type = PlannerType.PLANNER_STRAIGHT_LINE
        # change_planner_type_future = self.sm.change_planner_type_client.call_async(change_planner_type_request)
        # change_planner_type_future.add_done_callback(self.navigate_exit_cell)
        self.navigate_exit_cell()

    def navigate_exit_cell(self):
        # result: SetPlannerType.Response = future.result()
        # if not result.success:
        #     self.sm.get_logger().warn('Failed to change planner type going to error')
        #     self.sm.change_state(ROBOT_STATE.ERROR)
        #     return
        
        # TODO: NAV
        nav_goal = NavigateMapf.Goal()
        nav_goal.goal_pose = self.sm.current_work_cell_info.exit_pose
        nav_goal.workcell_id = self.sm.current_task.workcell_id
        self.nav_response_future = self.sm.mapf_navigation_client.send_goal_async(
            nav_goal, self.sm.on_nav_feedback
        )
        self.nav_response_future.add_done_callback(self.nav_goal_response_cb)
        # nav_goal = NavigateToPose.Goal()
        # nav_goal.pose = self.sm.current_work_cell_info.exit_pose
        # self.nav_reponse_future = self.sm.navigation_client.send_goal_async(
        #     nav_goal, self.sm.on_nav_feedback)
        # self.nav_reponse_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.sm.get_logger().warn('Nav 2 goal was rejected, aborting exit work cell, going to error')
            self.sm.change_state(ROBOT_STATE.ERROR)
        
        self.nav_goal_done_future: Future = goal_handle.get_result_async()
        self.nav_goal_done_future.add_done_callback(self.sm.on_nav_done)

    def on_nav_done(self, future: Future):
        nav_goal_result: GoalStatus = future.result().status
        self.sm.get_logger().info('Navigation result: ' + str(nav_goal_result))
        if nav_goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.call_robot_exited_cell()
        else:
            self.sm.current_task = None
            self.sm.get_logger().warn('Robot error when going to exit of work cell')
            self.sm.change_state(ROBOT_STATE.ERROR)

    def call_robot_exited_cell(self):
        robot_exited_request = Trigger.Request()
        robot_exited_future = self.robot_exited_client.call_async(robot_exited_request)
        robot_exited_future.add_done_callback(self.robot_exited_cb)

    def robot_exited_cb(self, future: Future):
        result: Trigger.Response = future.result()
        if result.success:
            self.sm.work_cell_heartbeat.deactivate()
            self.sm.change_state(ROBOT_STATE.FIND_WORKCELL)
        else:
            self.sm.get_logger().warn(
                f"Call to {'/'+self.sm.current_task.workcell_id.id + '/robot_exited'} failed, workcell may be in invalid state, retrying")
            self.call_robot_exited_cell()

    def deinit(self):
        self.robot_exited_client.destroy()


class ErrorState(RobotStateTemplate):
    def __init__(self, sm: RobotStateManager) -> None:
        self.sm = sm
        self.recovery_timer = self.sm.create_timer(10, self.recovery_cb)
        self.recovery_timer.cancel()

    def init(self):
        self.recovery_timer.reset()
        # clear job and task tree
        self.msg = TaskData()
        self.msg.task_state = TaskData.ERROR
        self.msg.robot_id = self.sm.id
        self.msg.stamp = self.sm.get_clock().now().to_msg()
        self.msg.total_time = self.sm.get_clock().now().seconds_nanoseconds()[0] - self.sm.task_start_time
        self.msg.task_id = self.sm.current_task_id
        
        self.sm.state_data_pub.publish(self.msg)
        self.sm.current_task = None
        self.sm.task_tree = None
        if(self.sm.work_cell_heartbeat is not None):
            self.sm.work_cell_heartbeat.deactivate()
        
    def deinit(self):
        self.recovery_timer.cancel()

    def recovery_cb(self):
        self.sm.change_state(ROBOT_STATE.READY_FOR_JOB)