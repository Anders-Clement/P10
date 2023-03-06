from dataclasses import dataclass, field
from typing import List
from spice_msgs.msg import Id, RobotType, Layer, Node, Work

@dataclass

class Vertex():
    id: int
    work_type: str
    work_info: str
    children_: List['Vertex'] = field(default_factory=list)

    def get_children(self):
        return self.children_


class WorkTree():

    __vertex: Vertex
    __rootVertex: Vertex
    __vertices = []
    __currentTask: Vertex
    __firstTask: bool

    def __init__(self, layers):
        self.layers = layers

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
        
    def get_root(self):
        return self.__rootVertex
    
    def get_tree(self):
        return self.__vertices
    
    def next_task(self):
        if(self.__firstTask):
            self.__firstTask = False
            return self.get_root()
        
        next_task = self.__currentTask.children_
        self.__currentTask = next_task
        return next_task

