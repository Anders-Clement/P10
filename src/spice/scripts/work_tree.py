from dataclasses import dataclass, field
from typing import List
from spice_msgs.msg import Id, RobotType, Layer, Node, Work
from rclpy.node import Node

@dataclass

class Vertex():
    id: int
    work_type: RobotType
    work_info: str
    children_: List['Vertex'] = field(default_factory=list)

    def get_children(self):
        return self.children_


class WorkTree():
    
    __currentTask = Vertex
    __rootVertex: Vertex
    __vertices: List['Vertex']

    def __init__(self, layers):
        self.layers = layers
        self.__vertices = []

        for layer in self.layers: # create work tree
            for node in layer.nodes:
                self.__vertices.append(Vertex(node.id, node.work.type, node.work.info))

        for layer in self.layers:
            for node in layer.nodes:
                children = []
                for child_id in node.children_id:
                    for vertex in self.__vertices:
                        if vertex.id == child_id:
                            children.append(vertex)
                            break
                for vertex in self.__vertices:
                    if vertex.id == node.id:
                        vertex.children_=children
                        break
        self.__rootVertex = self.__vertices[0]
        self.__currentTask = self.__rootVertex
        #self.__currentTask = self.__rootVertex
        
    def get_root(self):
        return self.__rootVertex
    
    def get_tree(self):
        return self.__vertices
    
    def get_next_work_types(self) -> list[RobotType]:
        # special case at root
        if self.__currentTask is self.__rootVertex:
            self.__rootVertex = None
            return [self.__currentTask.work_type]
        else:
            return [child.work_type for child in self.__currentTask.children_]
    
    def select_next_work_type(self, selected_work_type: RobotType):
        for child in self.__currentTask.children_:
            if child.work_type.type == selected_work_type.type:
                self.__currentTask = child
                break

        # for task in lastWorkType.children_:
        #     print("the current task is type is:" + task.work_type.type.__str__() + "the taken task is:" + lastWorkType.work_type.type.__str__())
        #     if task.work_type.type == lastWorkType.work_type.type:
        #         next_task = task.children_
        #         break
        # self.__currentTask = next_task
        # return next_task

