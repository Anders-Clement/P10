import random
import os
import numpy as np
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
import nav_msgs.msg as nav_msgs
import spice_mapf_msgs.msg as spice_mapf_msgs

class Map:
    def __init__(self, nodehandle: Node, load_map_from_topic: bool = True, inflate_map: bool = True):
        self.has_map = False
        self.inflate_map_upon_load = inflate_map
        if not load_map_from_topic:
            self.load_txt_map()
            return

        qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )
        self.map_subscriber = nodehandle.create_subscription(nav_msgs.OccupancyGrid, "/map", self.map_cb, qos)
        self.map_info: nav_msgs.MapMetaData = None

    def map_cb(self, msg: nav_msgs.OccupancyGrid):
        if not self.has_map:
            self.map_info = msg.info
            map = np.array(msg.data)
            map = np.reshape(map, (self.map_info.height,self.map_info.width))
            # flip upside down, as y-axis is positive in map, but we store it opposite
            map = np.flip(map, 0)
            self.map = map
            if self.inflate_map_upon_load:
                self.inflate_map()
            self.has_map = True

    def inflate_map(self):
        for y in range(self.map.shape[0]):
            for x in range(self.map.shape[1]):
                if self.map[y][x] == 100: # map obstacle
                    for expand_y in range(-1+y,2+y,1):
                        if expand_y < 0: continue
                        if expand_y >= self.map.shape[0]: continue

                        for expand_x in range(-1+x,2+x,1):
                            if expand_x < 0: continue
                            if expand_x >= self.map.shape[1]: continue
                            
                            if self.map[expand_y][expand_x] == 0:
                                self.map[expand_y][expand_x] = 50



    def load_txt_map(self, test_file: str = 'test_scenarios/exp5_5.txt'):
        spice_mapf_dir = get_package_share_directory('spice_mapf')
        test_dir = os.path.join(spice_mapf_dir, '../../local/lib/python3.10/dist-packages/spice_mapf')
        test_map = os.path.join(test_dir, test_file)
        with open(test_map, 'r') as file:
            # first line: #rows #columns
            line = file.readline()
            rows, columns = [int(x) for x in line.split(' ')]
            self.rows = int(rows)
            self.columns = int(columns)
            # #rows lines with the map
            self.map = []
            for row in range(rows):
                line = file.readline()
                self.map.append([])
                for cell in line:
                    if cell == '@':
                        self.map[-1].append(True)
                    elif cell == '.':
                        self.map[-1].append(False)

        self.map = np.array(self.map)
        self.map_info = nav_msgs.OccupancyGrid().info
        self.map_info.resolution = 0.5
        self.map_info.height = len(self.map)
        self.map_info.width = len(self.map[0])
        self.has_map = True

    def print_map(self) -> None:
        starts_map = [[-1 for _ in range(len(self.map[0]))] for _ in range(len(self.map))]
        to_print = ''
        for x in range(len(self.map)):
            for y in range(len(self.map[0])):
                if starts_map[x][y] >= 0:
                    to_print += str(starts_map[x][y]) + ' '
                elif self.map[x][y]:
                    to_print += '@ '
                else:
                    to_print += '. '
            to_print += '\n'
        print(to_print)

    def get_random_freespace(self) -> tuple[int,int]:
        while True:
            x = int(random.random()*len(self.map[0]))
            y = int(random.random()*len(self.map))
            if not self.map[y][x]:
                return y,x
            
    def world_to_map(self, position: spice_mapf_msgs.Position) -> tuple[int,int]:
        y_map = len(self.map)-1 - (position.y/self.map_info.resolution)
        x_map = position.x/self.map_info.resolution
        
        position = (int(round(y_map,0)), int(round(x_map,0)))
        return position
    
    def world_to_map_float(self, position: spice_mapf_msgs.Position) -> tuple[float, float]:
        y_map = len(self.map)-1 - (position.y/self.map_info.resolution)
        x_map = position.x/self.map_info.resolution
        
        position = (y_map, x_map)
        return position
    
    def map_to_world(self, location: tuple[int,int]) -> tuple[float,float]:
        """Take location in planner map (y,x) and convert to world (x,y)"""
        y_world = (len(self.map)-1 - location[0])*self.map_info.resolution
        x_world = location[1]*self.map_info.resolution
        return (x_world, y_world)
    
    # def world_to_map(self, position: spice_mapf_msgs.Position) -> tuple[int,int]:
    #     y_map = len(self.map)-1 - (position.y/self.map_info.resolution + self.map_info.resolution/2)
    #     x_map = position.x/self.map_info.resolution + self.map_info.resolution/2
        
    #     position = (int(round(y_map,0)), int(round(x_map,0)))
    #     return position
    
    # def world_to_map_float(self, position: spice_mapf_msgs.Position) -> tuple[float, float]:
    #     y_map = len(self.map)-1 - (position.y/self.map_info.resolution + self.map_info.resolution/2)
    #     x_map = position.x/self.map_info.resolution + self.map_info.resolution/2
        
    #     position = (y_map, x_map)
    #     return position
    
    # def map_to_world(self, location: tuple[int,int]) -> tuple[float,float]:
    #     """Take location in planner map (y,x) and convert to world (x,y)"""
    #     y_world = (len(self.map)-1 - location[0])*self.map_info.resolution - self.map_info.resolution/2
    #     x_world = location[1]*self.map_info.resolution - self.map_info.resolution/2
    #     return (x_world, y_world)