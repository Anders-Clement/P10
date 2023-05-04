from __future__ import annotations
import random
import time
import matplotlib.pyplot as plt
import random
import numpy as np

import Planning
from Map import Map
from Agent import Agent
from Visualizer import Visualizer
            

class Simulator:
    def __init__(self) -> None:
        self.map = Map("test_scenarios/exp8_8.txt")
        self.map.print_map()
        self.agents: list[Agent] = []
        self.planner = Planning.Planner(self.map, self.agents)

        self.visualizer = Visualizer(self.agents, self.map)

        self.timestep = 0
        self.time_interpolated = 0
        self.time_interpolation_steps = 10

        self.target_num_agents = 16
        self.add_starting_agents(self.target_num_agents)

    def tick(self):
        
        # populate agents
        if self.timestep % 10 == 0:
            if len(self.agents) < self.target_num_agents:
                self.add_random_agent()

        print(f'Tick {self.timestep}')
        start_time = time.time()

        # run the planner - and propagation of robot positions
        self.planner.tick(self.timestep)

        # if all are stuck waiting, reset their goals for next tick to avoid deadlock
        some_not_waiting = False
        for agent in self.agents:
            if not agent.waiting:
                some_not_waiting = True
                break
        if not some_not_waiting:
            for agent in self.agents:
                agent.target_goal = None

        end_tick_time = time.time()
        duration = end_tick_time -start_time
        print(f'planning duration: {duration}')

        # simulate positions in steps
        for i in range(1, self.time_interpolation_steps+1): #shift by one so last draw is at next_loc
            self.time_interpolated = self.timestep + i*(1/self.time_interpolation_steps)
            # update agent positions
            for agent in self.agents:
                pos_last = agent.current_loc
                pos_next = agent.next_loc
                agent.current_pos = (np.array(pos_next) - np.array(pos_last)) * (self.time_interpolated - self.timestep) + np.array(pos_last)

            # check for collisions
            for j in range(len(self.agents)):
                for l in range(j+1, len(self.agents)):
                    agent_l_pos = np.array(self.agents[l].current_pos)
                    agent_j_pos = np.array(self.agents[j].current_pos)
                    if np.linalg.norm(agent_l_pos-agent_j_pos) < 0.7:
                        print(f'COLLISION between agent ({l},{j}) at time {self.time_interpolated}')
                        raise Exception()

            # visualize time step
            self.visualizer.visualize()
            plt.pause(0.01)

        self.timestep += 1
        self.time_interpolated = self.timestep

    def add_starting_agents(self, amount: int = 1):
        for _ in range(amount):
            self.add_random_agent()

    def add_random_agent(self):
        num_tries = 0
        while num_tries < 1000:
            num_tries += 1
            x_start = int(random.random()*len(self.map.map))
            y_start = int(random.random()*len(self.map.map))
            if not self.map.map[x_start][y_start]:
                start = (x_start, y_start)
                valid = True
                for agent in self.agents:
                    if start == agent.current_loc or start == agent.next_loc:
                        valid = False
                        break
                if not valid:
                    continue
                print(f"Adding agent at position: {start}")
                self.agents.append(Agent(start, len(self.agents)))
                return
            
        print('failed to add agent, took too many tries')


if __name__ == "__main__":
    simulator = Simulator()
    while True:
        simulator.tick()