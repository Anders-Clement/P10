from __future__ import annotations
import time
import random
from enum import IntEnum
from rclpy.node import Node
import PrioritizedPlanner
from Map import Map
from Agent import Agent

class PlanningResult(IntEnum):
    SUCCESS = 0
    INVALID_GOAL = 1 # unable to get to goal using Dijkstra
    WAITING = 2 # currently blocked by other goals, but possible using Dijkstra

class Planner:
    def __init__(self, map: Map, agents: list[Agent], nodehandle: Node) -> None:
        self.map = map
        self.agents = agents
        self.logger = nodehandle.get_logger()
        # constraints as a list for timesteps, with a dict of constraints
        self.constraints: list[dict] = []
        self.goal_constraints = []
        self.prio_planner = PrioritizedPlanner.PrioritizedPlanner()
        self.paths_planned = 0

    def tick(self, timestep: int):

        # clear now old constraints, done here, instead of after planning, to allow agent planning between ticks
        if len(self.constraints) > 0:
            self.constraints.pop(0)

        self.goal_constraints = []
        for agent in self.agents:
            if agent.waiting: # add waiting agents as goal constraints
                self.goal_constraints.append((agent.current_loc, 0, agent.id))
            else: # add pathing agents' goal as constraints
                self.goal_constraints.append((agent.current_goal, len(agent.path)-1, agent.id))

        for agent in self.agents:
            if agent.is_simulated:
                if len(agent.path) == 0 and agent.current_loc == agent.current_goal:
                    # agent is either at goal, or at startup without initial plan
                    result = self.replan_agent(agent)
                    self.logger.info(f'Planning result for simulated agent {agent.id.id}: {result.name}')
            else:
                if agent.waiting:
                    self.replan_agent(agent)

    def replan_agent(self, agent: Agent) -> PlanningResult:
        if agent.target_goal is None:
            self.logger.error(f'ERROR: asked to replan agent {agent.id.id}, but it has no target_goal')
            raise Exception()
        
        h_values = PrioritizedPlanner.PrioritizedPlanner.compute_heuristics(self.map.map, agent.target_goal)
        if h_values.get(agent.current_loc) is None:
            self.logger.warn(f'WARN: start location: {agent.current_loc} is unreachable from goal location: {agent.target_goal}')
            return PlanningResult(value=PlanningResult.INVALID_GOAL)
        
        path = self.prio_planner.a_star(
            self.map.map,
            agent,
            self.constraints,
            len(self.constraints),
            self.goal_constraints,
            h_values
        )

        if path is None:
            agent.waiting = True
            return PlanningResult(value=PlanningResult.WAITING)
        else:
            # print(f'Path for agent {agent.id}: {path}')
            agent.path = path
            agent.path_id = self.paths_planned
            self.paths_planned += 1
            agent.current_goal = agent.target_goal
            agent.target_goal = None
            agent.waiting = False

            self.goal_constraints.append((agent.path[-1], len(agent.path)-1, agent.id)) # add goal constraints for other agents
            self.add_constraints(path, agent)
            return PlanningResult(value=PlanningResult.SUCCESS)

    def add_constraints(self, path: list[tuple[int,int]], agent: Agent) -> None:
        while len(path) > len(self.constraints) - 1:
            self.constraints.append({})
        for timestep, vertex in enumerate(path):
            self.constraints[timestep][vertex] = agent.id # avoid vertex collision
            if timestep > 0:
                self.constraints[timestep][path[timestep-1]] = agent.id # to maintain 1 extra seperation behind
            if timestep < len(path)-1:
                self.constraints[timestep][path[timestep+1]] = agent.id # avoid edge collision
            if timestep < len(path)-2:
                self.constraints[timestep][path[timestep+2]] = agent.id # to maintain 1 extra seperation

    def make_random_goal(self):
        """Create a new goal, which is in free space in map, and unique to other agents"""
        while True:
            goal = self.map.get_random_freespace()
            valid = True
            for agent in self.agents:
                if agent.target_goal == goal:
                    valid = False
                    break
                if agent.current_goal == goal and not agent.waiting:
                    valid = False
                    break
            if not valid:
                continue
            return goal       
    