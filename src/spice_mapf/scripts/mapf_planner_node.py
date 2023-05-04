#!/usr/bin/env python3

import os
import random
import numpy as np
import rclpy
from rclpy.node import Node
import spice_msgs.msg as spice_msgs
import spice_mapf_msgs.msg as spice_mapf_msgs
import spice_mapf_msgs.srv as spice_mapf_srvs
from Planning import Planner
from Agent import Agent
from Map import Map
from Visualizer import Visualizer


class MapfPlanner(Node):
    def __init__(self):
        super().__init__('MapfPlanner')
        self.map = Map(self)
        self.agents: list[Agent] = []
        self.planner = Planner(self.map, self.agents)
        self.visualizer = Visualizer(self.map, self.agents)
        self.visualizer.visualize()
        self.timer = self.create_timer(0.1, self.tick)

        self.timestep = 0
        self.time_interpolated = 0
        self.time_interpolation_steps = 10

        self.join_planner_service = self.create_service(spice_mapf_srvs.JoinPlanner, "/join_planner", self.join_planner_cb)
        self.request_goal_service = self.create_service(spice_mapf_srvs.RequestGoal, "/request_goal", self.request_goal_cb)
        self.paths_publisher = self.create_publisher(spice_mapf_msgs.Paths, "/mapf_paths", 10)
        self.robot_pos_subscriber = self.create_subscription(spice_mapf_msgs.Position, "/robot_pos", self.robot_pos_cb, 10)

    def robot_pos_cb(self, msg: spice_mapf_msgs.Position) -> None:
        for agent in self.agents:
            if agent.id == msg.id:
                agent.current_pos = (msg.y, msg.x)
                return

        self.get_logger().warn(f'Got robot_loc from agent: {msg.id.id}, but it has not joined the planner yet')

    def join_planner_cb(self, request: spice_mapf_srvs.JoinPlanner.Request, response: spice_mapf_srvs.JoinPlanner.Response):
        if self.can_add_agent_at_loc([request.location.y, request.location.x]):
            self.add_agent((request.location.y, request.location.x), request.id)
            response.success = True
        else:
            response.success = False
            self.get_logger().warn(f'Requested to join planner at: {(request.location.y, request.location.x)}[y,x], but it is not possible currently')
        
        return response
    
    def request_goal_cb(self, request: spice_mapf_srvs.RequestGoal.Request, response: spice_mapf_srvs.RequestGoal.Response):
        goal = (request.goal.y, request.goal.x)
        if self.is_goal_free(goal):
            # assign goal
            for agent in self.agents:
                if agent.id == request.id:
                    if agent.target_goal is not None:
                        return response
                    if len(agent.path) > 0:
                        self.get_logger().warn(f'Agent: {agent.id.id} tried to preempt goal, but it is not supported')
                        return response
                    
                    agent.target_goal = goal
                    response.success = True
                    break
        return response

    def tick(self):
        # ensure that a map has been received first
        if not self.map.has_map:
            return
        
        # add initial simulated agents
        target_num_agents = 0
        if len(self.agents) < target_num_agents:
            if self.add_random_agent():
                goal = self.planner.make_random_goal()
                self.agents[-1].target_goal = goal
                self.get_logger().info(f'Created goal at: {goal} for agent {self.agents[-1].id.id} from {self.agents[-1].current_loc}')
                
        self.visualizer.visualize()
        # only proceed if all agents are at their next locations
        if not self.ready_to_tick():
            return
                
        # update agents' next locations
        for agent in self.agents:
            agent.current_loc = agent.next_loc
            if len(agent.path) > 0:
                agent.next_loc = agent.path.pop(0)

        # publish all paths
        paths_msg = spice_mapf_msgs.Paths()
        for agent in self.agents:
            path_msg = spice_mapf_msgs.Path()
            path_msg.id = agent.id
            loc_msg = spice_mapf_msgs.Location()
            y,x = agent.next_loc
            loc_msg.x = x
            loc_msg.y = y
            path_msg.location = loc_msg
            paths_msg.paths.append(path_msg)
        self.paths_publisher.publish(paths_msg)

        self.planner.tick(self.timestep)

        # simulate positions in steps for simulated robots
        for i in range(1, self.time_interpolation_steps+1): #shift by one so last draw is at next_loc
            self.time_interpolated = self.timestep + i*(1/self.time_interpolation_steps)
            # update agent positions
            for agent in self.agents:
                if agent.is_simulated:
                    pos_last = agent.current_loc
                    pos_next = agent.next_loc
                    agent.current_pos = (np.array(pos_next) - np.array(pos_last)) * (self.time_interpolated - self.timestep) + np.array(pos_last)

            # check for collisions
            for j in range(len(self.agents)):
                for l in range(j+1, len(self.agents)):
                    agent_l_pos = np.array(self.agents[l].current_pos)
                    agent_j_pos = np.array(self.agents[j].current_pos)
                    if np.linalg.norm(agent_l_pos-agent_j_pos) < 0.7:
                        self.get_logger().info(f'COLLISION between agent ({l},{j}) at time {self.time_interpolated}')
                        raise Exception()

            # visualize time step
            self.visualizer.visualize()

    def ready_to_tick(self):
        non_ready_agents = []
        for agent in self.agents:
            if not np.allclose(agent.current_pos, agent.next_loc, atol=0.1):
                non_ready_agents.append(agent)

        if len(non_ready_agents) > 0:
            self.get_logger().info(f'Non ready agents: {[a.__str__() for a in non_ready_agents]}')                
            return False
        
        return True    
    
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
    
    def add_agent(self, loc: tuple[int,int], id: spice_msgs.Id, is_simulated=False):
        self.get_logger().info(f"Adding agent at position: {loc}")
        self.agents.append(Agent(loc, id, is_simulated))

    def add_random_agent(self):
        num_tries = 0
        while num_tries < 1000:
            num_tries += 1
            x_start = int(random.random()*len(self.map.map)-1)
            y_start = int(random.random()*len(self.map.map)-1)
            if self.can_add_agent_at_loc([y_start, x_start]):
                self.add_agent((y_start,x_start), spice_msgs.Id(id=str(len(self.agents))), is_simulated=True)
                return True
        return False
    
    def is_goal_free(self, goal: tuple[int,int]) -> bool:
        for agent in self.agents:
            if agent.target_goal == goal:
                return False
            if agent.current_goal == goal:
                return False
        return True



if __name__ == '__main__':
    rclpy.init()
    planner = MapfPlanner()
    rclpy.spin(planner)