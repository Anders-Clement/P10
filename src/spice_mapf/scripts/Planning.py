from __future__ import annotations
import time
import random
import PrioritizedPlanner
from Map import Map
from Agent import Agent


class Planner:
    def __init__(self, map_: Map, agents: list[Agent]) -> None:
        self.map = map_
        self.agents = agents
        # constraints as a list for timesteps, with a dict of constraints
        self.constraints: list[dict] = []
        self.prio_planner = PrioritizedPlanner.PrioritizedPlanner()
        self.paths_planned = 0
        self.planning_time = 0

    def tick(self, timestep: int):

        # clear now old constraints, done here, instead of after, to allow agent planning between ticks
        if len(self.constraints) > 0:
            self.constraints.pop(0)

        self.goal_constraints = []
        # add waiting agents as goal constraints
        for agent in self.agents:
            if agent.waiting:
                self.goal_constraints.append((agent.current_loc, 0, agent.id))
            else:
                self.goal_constraints.append((agent.current_goal, len(agent.path)-1, agent.id))

        self.planning_time = 0
        for agent in self.agents:
            if len(agent.path) == 0 and agent.current_loc == agent.current_goal:
                # agent is either at goal, or at start without plan
                self.replan_agent(agent)
        # print(f'INFO: a* planning time for all agents: {self.planning_time:.3f}')
        # print(f'constraints:')
        # for constraint in self.constraints:
        #     print(constraint)
        

        # for agent in self.agents:
        #     print(f'Path for agent {agent.id}: {agent.path}')

    def replan_agent(self, agent: Agent) -> bool:
        if agent.target_goal is None:
            # print(f'WARN: asked to replan agent {agent.id.id}, but it has no target_goal')
            return False
        start_time = time.time()
        path = self.prio_planner.a_star(
            self.map.map,
            agent.current_loc,
            agent.target_goal,
            agent.id,
            self.constraints,
            len(self.constraints),
            self.goal_constraints,
        )
        duration = time.time() - start_time
        # print(f'Planned path in {duration} seconds')
        self.planning_time += duration

        if path is None:
            agent.waiting = True
            return False
        else:
            # print(f'Path for agent {agent.id}: {path}')
            agent.path = path
            agent.path_id = self.paths_planned
            self.paths_planned += 1
            agent.current_goal = agent.target_goal
            agent.target_goal = None
            agent.waiting = False
            agent.current_loc = path[0]
            self.goal_constraints.append((agent.path[-1], 0, agent.id)) # add goal constraints for other agents
            self.add_constraints(path, agent)
            return True

    def add_constraints(self, path: list[tuple[int,int]], agent: Agent) -> None:
        while len(path) > len(self.constraints) - 1:
            self.constraints.append({})
        for timestep, vertex in enumerate(path):
            self.constraints[timestep][vertex] = agent.id
            if timestep < len(path)-1:
                self.constraints[timestep][path[timestep+1]] = agent.id

    def make_random_goal(self):
        """Create a new goal, which is in free space in map, and unique to other agents"""
        while True:
            goal = self.map.get_random_freespace()
            valid = True
            for agent in self.agents:
                if agent.waiting and agent.target_goal == goal:
                    valid = False
                    break
                if agent.current_goal == goal and not agent.waiting:
                    valid = False
                    break
            if not valid:
                continue
            return goal       
    