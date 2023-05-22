import spice_msgs.msg as spice_msgs
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA

class Agent:
    """Container for an agent, encapsulating pose and current plan"""
    def __init__(self, start_loc: tuple[int,int], heading: Quaternion, id: spice_msgs.Id, is_simulated = False) -> None:
        self.id = id
        self.start_loc = start_loc
        self.current_loc = start_loc
        self.current_pos = self.current_loc
        self.next_loc = start_loc
        self.next_heading = heading
        self.target_goal = None
        self.current_goal = start_loc
        self.waiting = False
        self.path_id = -1
        self.path = []
        self.is_simulated = is_simulated
        self.workcell_id = spice_msgs.Id()
        self.color = "yellow"

    def debug_str(self, mapf_planner) -> str:
        return f'Id: {self.id.id}, next_loc world: {mapf_planner.map.map_to_world(self.next_loc)}, cur_pos world: {mapf_planner.map.map_to_world(self.current_pos)}'