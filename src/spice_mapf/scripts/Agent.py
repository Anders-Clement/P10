import spice_msgs.msg as spice_msgs

class Agent:
    """Container for an agent, encapsulating pose and current plan"""
    def __init__(self, start_loc: tuple[int,int], id: spice_msgs.Id, is_simulated = False) -> None:
        self.id = id
        self.start_loc = start_loc
        self.current_loc = start_loc
        self.current_pos = self.current_loc
        self.next_loc = start_loc
        self.target_goal = None
        self.current_goal = start_loc
        self.waiting = False
        self.path_id = -1
        self.path = []
        self.is_simulated = is_simulated

    def __str__(self) -> str:
        return f'Id: {self.id.id}, next_loc: {self.next_loc}, cur_pos: {self.current_pos}'