import random
import os
import numpy as np
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
import nav_msgs.msg as nav_msgs

class Map:
    def __init__(self, nodehandle: Node, load_map_from_topic: bool = True):
        self.has_map = False
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
            self.has_map = True

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