#!/usr/bin/env python3

import os
import random
import numpy as np
import matplotlib as mpl
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
import spice_msgs.msg as spice_msgs
import spice_msgs.srv as spice_srvs
import spice_mapf_msgs.msg as spice_mapf_msgs
import spice_mapf_msgs.srv as spice_mapf_srvs
from Planning import Planner, PlanningResult
from Agent import Agent
from Map import Map
from Visualizer import Visualizer

import tf2_ros
from tf2_ros import TransformException
import tf_transformations

class MapfPlanner(Node):
    def __init__(self):
        super().__init__('MapfPlanner')
        self.map = Map(self, load_map_from_topic=True)
        self.agents: list[Agent] = []
        self.planner = Planner(self.map, self.agents, self)
        self.workcell_obstacle = WorkcellObstacle(self)
        self.visualizer = Visualizer(self.map, self.agents, self.workcell_obstacle, True)
        self.visualizer.visualize()
        self.timer = self.create_timer(0.1, self.tick)
        self.visualizer_timer = self.create_timer(0.1, self.visualizer.visualize)
        self.GOAL_TOLERANCE = 0.25
        self.declare_parameter('goal_tolerance', self.GOAL_TOLERANCE)

        self.timestep = 0
        self.time_interpolated = 0
        self.time_interpolation_steps = 3
        self.target_simulated_agents = 0

        self.join_planner_service = self.create_service(spice_mapf_srvs.JoinPlanner, "/join_planner", self.join_planner_cb)
        self.request_goal_service = self.create_service(spice_mapf_srvs.RequestGoal, "/request_goal", self.request_goal_cb)
        self.paths_publisher = self.create_publisher(spice_mapf_msgs.RobotPoses, "/mapf_paths", 10)
        self.debug_paths_publisher = self.create_publisher(MarkerArray, "/planned_paths", 10)
        self.robot_pos_subscriber = self.create_subscription(spice_mapf_msgs.RobotPose, "/robot_pos", self.robot_pos_cb, 10)

    def robot_pos_cb(self, msg: spice_mapf_msgs.RobotPose) -> None:
        for agent in self.agents:
            if agent.id == msg.id:
                agent.current_pos = self.map.world_to_map_float(msg.position)
                return

        self.get_logger().warn(f'Got robot_loc from agent: {msg.id.id}, but it has not joined the planner yet')
        msg.rejoin = True
        reset_msg = spice_mapf_msgs.RobotPoses()
        reset_msg.poses.append(msg)
        self.paths_publisher.publish(reset_msg)

    def join_planner_cb(self, request: spice_mapf_srvs.JoinPlanner.Request, response: spice_mapf_srvs.JoinPlanner.Response):
        if not self.map.has_map:
            self.get_logger().info('Robot tried to join planner, but planner is awaiting map', once=True)
            return response
        
        join_location = self.map.world_to_map(request.robot_pose.position)
        self.get_logger().info(
            f'Trying to add agent {request.robot_pose.id.id} at world x,y: {request.robot_pose.position.x:.2f},{request.robot_pose.position.y:.2f}'
            )
        
        # check if agent is already present, but old, if so, remove it before adding it
        for agent in self.agents:
                if agent.id.id == request.robot_pose.id.id:
                    self.agents.remove(agent)
                    self.get_logger().warn(f'Agent: {agent.id.id} has rejoined the planner')
                    break
        if self.can_add_agent_at_loc(join_location):
            self.add_agent(join_location, request.robot_pose)
            self.publish_robot_paths() # to give initial pose to robots
            response.success = True
        else:
            response.success = False
            self.get_logger().warn(f'Requested to join planner at: {(request.robot_pose.position.y, request.robot_pose.position.x)}[y,x], but it is not possible currently')
        
        return response
    
    def request_goal_cb(self, request: spice_mapf_srvs.RequestGoal.Request, response: spice_mapf_srvs.RequestGoal.Response):
        goal = self.map.world_to_map(request.robot_pose.position)

        self.get_logger().info(f'Got goal world x,y: {request.robot_pose.position.x},{request.robot_pose.position.y}, map y,x: {goal} from agent: {request.robot_pose.id.id}')

        for agent in self.agents:
            if agent.id.id == request.robot_pose.id.id:
                if goal == agent.current_goal and goal == agent.current_loc: # robot is already at requested goal
                    response.success = True
                    goal_in_world = self.map.map_to_world(goal)
                    response.goal_position.x = goal_in_world[0]
                    response.goal_position.y = goal_in_world[1]
                    self.get_logger().info(f'Agent: {agent.id.id} tried to navigate to its current position')
                    return response
        
    # def is_goal_free(self, goal: tuple[int,int]) -> bool:
    #     if self.map.map[goal[0]][goal[1]]:
    #         return False
    #     for agent in self.agents:
    #         if agent.target_goal == goal:
    #             return False
    #         if agent.current_goal == goal:
    #             return False
    #     return True
    
        if self.map.map[goal[0]][goal[1]]:
            self.get_logger().warn(f'Agent: {request.robot_pose.id.id} requested a goal which was not free in map')
            return response

        for agent in self.agents:
            if agent.target_goal == goal or agent.current_goal == goal:
                response.currently_occupied = True
                return response
        
        # assign goal
        for agent in self.agents:
            if agent.id == request.robot_pose.id:
                # TODO: Consider clearing obsolete constraints if agent preempts a goal

                if agent.target_goal is not None or len(agent.path) > 0 or agent.current_loc != agent.next_loc:
                    # self.get_logger().warn(f'Agent: {request.robot_pose.id.id} tried to preempt goal, but it is not supported')
                    # response.currently_occupied = True
                    # return response
                    self.get_logger().info(f'Agent: {request.robot_pose.id.id} preempted goal, this is not yet tested')
                
                agent.workcell_id = request.workcell_id                
                agent.target_goal = goal
                agent.current_loc = agent.next_loc # ensure update of current location
                agent.start_loc = agent.current_loc
                agent.path = []
                goal_in_world = self.map.map_to_world(goal)
                response.goal_position.x = goal_in_world[0]
                response.goal_position.y = goal_in_world[1]
                planning_result = self.planner.replan_agent(agent)
                if planning_result.value == PlanningResult.SUCCESS or planning_result.value == PlanningResult.WAITING:
                    response.success = True                
                    self.get_logger().info(f'Accepted goal from agent: {request.robot_pose.id.id}')
                else:
                    self.get_logger().warn(f'Unreachable goal requested from agent: {request.robot_pose.id.id}')
                return response
            
        # agent is unknown:
        self.get_logger().warn(f'Got goal request from unknown agent: {request.robot_pose.id.id}')
        return response
    
    def calculate_heading(self, current_loc: tuple[int,int], next_loc: tuple[int,int]):
        x_diff = next_loc[1] - current_loc[1]
        y_diff = next_loc[0] - current_loc[0]
        y_diff *= -1 # flip y-axis to match with world coordinates
        heading = np.arctan2(y_diff, x_diff)
        q = tf_transformations.quaternion_from_euler(0,0,heading)
        return Quaternion(x = q[0], y=q[1], z=q[2], w=q[3])
    
    def publish_robot_paths(self):
        paths_msg = spice_mapf_msgs.RobotPoses()
        for agent in self.agents:
            path_msg = spice_mapf_msgs.RobotPose()
            path_msg.id = agent.id
            x,y = self.map.map_to_world(agent.next_loc)
            path_msg.position.x = x
            path_msg.position.y = y
            path_msg.heading = agent.next_heading
            paths_msg.poses.append(path_msg)
        self.paths_publisher.publish(paths_msg)

    def tick(self):
        # ensure that a map has been received first
        if not self.map.has_map:
            self.get_logger().info('Awaiting map', once=True)
            return
        
        self.get_logger().info('Got map', once=True)
        
        # add initial simulated agents
        if len(self.agents) < self.target_simulated_agents:
            if self.add_random_agent():
                self.make_random_goal_for_agent(self.agents[-1])
                self.planner.replan_agent(self.agents[-1])

        # only proceed if all agents are at their next locations
        if not self.ready_to_tick():
            return
        
        # update agents' next locations
        for agent in self.agents:
            agent.current_loc = agent.next_loc
            if len(agent.path) > 0:
                agent.next_loc = agent.path.pop(0)
                if len(agent.path) > 0: # still have more poses after current goal
                    agent.next_heading = self.calculate_heading(agent.next_loc, agent.path[0])
                else: # otherwise maintain heading used to go to cell
                    agent.next_heading = self.calculate_heading(agent.current_loc, agent.next_loc)

        # add new random goal for simulated agents
        for agent in self.agents:
            if agent.is_simulated:
                if agent.target_goal is None and len(agent.path) == 0:
                    self.make_random_goal_for_agent(agent)

        self.planner.tick(self.timestep, self.workcell_obstacle.workcell_locations)

        self.publish_robot_paths()

        self.debug_publish_planned_paths()

        # simulate positions in steps for simulated robots
        for i in range(1, self.time_interpolation_steps+1): #shift by one so last draw is at next_loc
            self.time_interpolated = self.timestep + i*(1/self.time_interpolation_steps)
            has_simulated_agents = False
            # update agent positions
            for agent in self.agents:
                if agent.is_simulated:
                    has_simulated_agents = True
                    pos_last = agent.current_loc
                    pos_next = agent.next_loc
                    agent.current_pos = (np.array(pos_next) - np.array(pos_last)) * (self.time_interpolated - self.timestep) + np.array(pos_last)

            # check for collisions
            for j in range(len(self.agents)):
                for l in range(j+1, len(self.agents)):
                    agent_l_pos = np.array(self.agents[l].current_pos)
                    agent_j_pos = np.array(self.agents[j].current_pos)
                    if np.linalg.norm(agent_l_pos-agent_j_pos) < 0.7:
                        self.get_logger().error(f'COLLISION between agent ({l},{j}) at time {self.time_interpolated}')
                        #raise Exception()

            # visualize time step
            if has_simulated_agents:
                self.visualizer.visualize()

    def ready_to_tick(self):
        GOAL_TOLERANCE = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        non_ready_agents: list[Agent] = []
        ready_agents: list[Agent] = []
        for agent in self.agents:
            diff = np.array(agent.current_pos)-np.array(agent.next_loc)
            dist = np.linalg.norm(diff)/2.0 # have to divide by two here to make it a vector norm, I do not know why?
            if dist > GOAL_TOLERANCE:
                non_ready_agents.append((agent, dist))
            else:
                ready_agents.append((agent,dist))
        if not len(non_ready_agents) is 0: 
            self.get_logger().info(f'Non ready agents: {[a.debug_str(self) + f" dist: {dist:.2f}" for a,dist in non_ready_agents]}')                
        #self.get_logger().info(f'Ready agents: {[a.debug_str(self) + f" dist: {dist:.2f}" for a,dist in ready_agents]}')                
        
        return len(non_ready_agents) == 0    
    
    def can_add_agent_at_loc(self, loc):
        y_start = loc[0]
        x_start = loc[1]

        if self.map.map[y_start][x_start]:
            return False
        
        start = (y_start, x_start)
        for agent in self.agents:
            if start == agent.current_loc or start == agent.next_loc:
                return False
            for agent_loc in agent.path:
                if (np.array(agent_loc) == loc).all():
                    return False
        return True
    
    def add_agent(self, loc: tuple[int,int], robot_pose: spice_mapf_msgs.RobotPose, is_simulated=False):
        self.get_logger().info(f"Adding agent at position: {loc}, world: {self.map.map_to_world(loc)}, is_simulated: {is_simulated}")
        self.agents.append(Agent(loc, robot_pose.heading, robot_pose.id, is_simulated))

    def add_random_agent(self):
        num_tries = 0
        while num_tries < 1000:
            num_tries += 1
            x_start = int(random.random()*len(self.map.map[0])-1)
            y_start = int(random.random()*len(self.map.map)-1)
            if self.can_add_agent_at_loc([y_start, x_start]):
                id = spice_msgs.Id(id=str(len(self.agents)))
                self.add_agent((y_start,x_start), spice_mapf_msgs.RobotPose(id=id), is_simulated=True)
                return True
        return False
    
    def make_random_goal_for_agent(self, agent: Agent):
        goal = self.planner.make_random_goal()
        agent.target_goal = goal
        self.get_logger().info(f'Created goal at: {goal} for agent {agent.id.id} from {agent.current_loc}')

    def debug_publish_planned_paths(self):
        marker_array_msg = MarkerArray()
        
        for agent in self.agents:
            if not agent.waiting:
                marker = Marker()
                marker.action = 0
                marker.type = 4
                marker.scale.x = 0.05
                marker.lifetime = Duration(seconds=30).to_msg()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = agent.id.id
                marker.id = 0
                marker.header.frame_id = "map"
                r,g,b = mpl.colors.to_rgb(agent.color)
                marker.color.a = 1.0
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                x,y = self.map.map_to_world(agent.current_loc)
                point = Point()
                point.x = x
                point.y = y
                marker.points.append(point)
                x,y = self.map.map_to_world(agent.next_loc)
                point = Point()
                point.x = x
                point.y = y
                marker.points.append(point)

                for location in agent.path:
                    x,y = self.map.map_to_world(location)
                    point = Point()
                    point.x = x
                    point.y = y
                    marker.points.append(point)
                
                marker_array_msg.markers.append(marker)

        self.debug_paths_publisher.publish(marker_array_msg)


class WorkcellObstacle():
    def __init__(self, mapf_planner: MapfPlanner):
        self.mapf_planner = mapf_planner
        self.get_robots_timer = self.mapf_planner.create_timer(1.0, self.get_robots_timer_cb)
        self.get_workcells_client = self.mapf_planner.create_client(spice_srvs.GetRobotsByType, "/get_robots_by_type")
        self.workcell_ids: list[spice_msgs.Id] = []
        self.workcell_locations: list[tuple[spice_msgs.Id, tuple[int,int]]] = []
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self.mapf_planner)

    def get_robots_timer_cb(self):
        request = spice_srvs.GetRobotsByType.Request()
        request.type = spice_msgs.RobotType(type=spice_msgs.RobotType.WORK_CELL_ANY)
        self.get_workcells_client.call_async(request).add_done_callback(self.get_robots_cb)

    def get_robots_cb(self, future: Future):
        result: spice_srvs.GetRobotsByType.Response = future.result()
        robots: list[spice_msgs.Robot] = result.robots
        for robot in robots:
            if robot.id not in self.workcell_ids:
                self.workcell_ids.append(robot.id)

        self.get_workcell_locations()

    def get_workcell_locations(self):
        self.workcell_locations = []
        for workcell_id in self.workcell_ids:
            post_fixes = ["", "_entry", "_exit"]
            for post_fix in post_fixes:
                obstacle = self.get_workcell_obstacle(workcell_id, post_fix)
                if obstacle is not None:
                    self.workcell_locations.append(obstacle)
            
                
    def get_workcell_obstacle(self, workcell_id: spice_msgs.Id, frame_postfix: str) -> tuple[spice_msgs.Id, tuple[int,int]]:
        
        from_frame_rel = workcell_id.id + frame_postfix
        to_frame_rel = "map"
        try:
            workcell_transform = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, Time())
        except TransformException as e:
            self.mapf_planner.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}', once=True)
            return None
        
        pos = spice_mapf_msgs.Position()
        pos.x = workcell_transform.transform.translation.x
        pos.y = workcell_transform.transform.translation.y
        return (workcell_id, self.mapf_planner.map.world_to_map(pos))

        

if __name__ == '__main__':
    rclpy.init()
    planner = MapfPlanner()
    rclpy.spin(planner)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass