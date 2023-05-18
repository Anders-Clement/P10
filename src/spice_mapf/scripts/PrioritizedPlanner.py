import heapq
import spice_msgs.msg as spice_msgs

class PrioritizedPlanner:
    @staticmethod
    def move(loc, dir):
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

    @staticmethod
    def compute_heuristics(my_map, goal):
        # Use Dijkstra to build a shortest-path tree rooted at the goal location
        open_list = []
        closed_list = dict()
        root = {'loc': goal, 'cost': 0}
        heapq.heappush(open_list, (root['cost'], goal, root))
        closed_list[goal] = root
        while len(open_list) > 0:
            (cost, loc, curr) = heapq.heappop(open_list)
            for dir in range(4):
                child_loc = PrioritizedPlanner.move(loc, dir)
                child_cost = cost + 1
                if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                child = {'loc': child_loc, 'cost': child_cost}
                if child_loc in closed_list:
                    existing_node = closed_list[child_loc]
                    if existing_node['cost'] > child_cost:
                        closed_list[child_loc] = child
                        heapq.heappush(open_list, (child_cost, child_loc, child))
                else:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))

        # build the heuristics table
        h_values = {}
        for loc, node in closed_list.items():
            h_values[loc] = node['cost']
        return h_values

    @staticmethod
    def get_path(goal_node):
        path = []
        curr = goal_node
        # trim off unneccessary wait at end
        while True:
            if curr['parent'] is None:
                break
            if curr['parent']['loc'] != curr['loc']:
                break
            curr = curr['parent']
        while curr is not None:
            path.append(curr['loc'])
            curr = curr['parent']
        path.reverse()
        return path

    @staticmethod
    def is_constrained(curr_loc, next_loc, next_time, constraint_table: list[dict], 
                       goal_constraints: list, agent, goal_loc, workcell_constraints: list[tuple[spice_msgs.Id, tuple[int,int]]]):
        if next_time < len(constraint_table):
            constraint = constraint_table[next_time].get(next_loc)
            if constraint is not None:
                if constraint != agent.id:
                    return True
        
        for constraint_pos, timestep, constraint_id  in goal_constraints:
            if agent.id == constraint_id: # do not constrain oneself
                continue
            if timestep <= next_time-1 and (constraint_pos == next_loc or constraint_pos == curr_loc):
                return True
            
        for workcell_id, workcell_loc in workcell_constraints:
            if agent.workcell_id != workcell_id:
                # never allow touching other work cells
                if workcell_loc == next_loc:
                    return True
            else:
                # only allow touching the target workcell, if it is at goal loc
                if next_loc != goal_loc:
                    return True
        
        return False

    @staticmethod
    def push_node(open_list, node):
        heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

    @staticmethod
    def pop_node(open_list):
        _, _, _, curr = heapq.heappop(open_list)
        return curr

    @staticmethod
    def compare_nodes(n1, n2):
        """Return true is n1 is better than n2."""
        return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

    @staticmethod
    def a_star(my_map, agent, constraint_table, earliest_goal_timestep, goal_constraints, h_values, 
               workcell_constraints: list[tuple[spice_msgs.Id, tuple[int,int]]]):
        """ my_map      - binary obstacle map
            start_loc   - start position
            goal_loc    - goal position
            agent       - the agent that is being re-planned
            constraints - constraints defining where robot should or cannot go at each timestep
        """
        
        start_loc = agent.start_loc
        goal_loc = agent.target_goal

        map_width = len(my_map[0])
        map_height = len(my_map)
        max_iter = map_width*map_height*4
        open_list = []
        closed_list = {}
        root = {'loc': start_loc, 'g_val': 0, 'h_val': h_values[start_loc], 'parent': None, 'time': 0}
        PrioritizedPlanner.push_node(open_list, root)
        closed_list[(root['loc'],0)] = root
        num_nodes_visited = 1
        num_nodes_opened = 1
        while len(open_list) > 0:
            curr = PrioritizedPlanner.pop_node(open_list)
            num_nodes_visited += 1

            if num_nodes_visited >= max_iter:
                # print(f'Got to max iterations for agent {agent_id.id} from: {start_loc} to {goal_loc}')
                return None
            # #############################
            # # Task 1.4: Adjust the goal test condition to handle goal constraints
            # if curr['loc'] == goal_loc: # and curr['time'] >= earliest_goal_timestep:
            #     print(f'Visited {num_nodes_visited} nodes for agent {agent}')
            #     return get_path(curr)

            if curr['loc'] == goal_loc and curr['time'] >= earliest_goal_timestep:
                path = PrioritizedPlanner.get_path(curr)
                # print(f'Visited {num_nodes_visited} nodes for agent {agent}, opened {num_nodes_opened} nodes, ratio: {num_nodes_opened/len(path)}')
                return path
            
            # add child nodes for four cardinal directions
            for direction in range(4):
                child_loc = PrioritizedPlanner.move(curr['loc'], direction)
                y,x = child_loc
                if x < 0 or y < 0:
                    continue
                if x >= map_width or y >= map_height:
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue

                child = {'loc': child_loc,
                        'g_val': curr['g_val'] + 1,
                        'h_val': h_values[child_loc],
                        'parent': curr,
                        'time': curr['time']+1}
                
                # do not add constrained nodes
                if PrioritizedPlanner.is_constrained(curr['loc'], child_loc, child['time'], constraint_table, goal_constraints, agent, goal_loc, workcell_constraints):
                    continue
                
                if (child['loc'], curr['time']+1) in closed_list:
                    existing_node = closed_list[(child['loc'],child['time'])]
                    if PrioritizedPlanner.compare_nodes(child, existing_node):
                        closed_list[(child['loc'],child['time'])] = child
                        PrioritizedPlanner.push_node(open_list, child)
                else:
                    closed_list[(child['loc'],child['time'])] = child
                    PrioritizedPlanner.push_node(open_list, child)

            # add node waiting at same spot
            child = {'loc': curr['loc'],
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[curr['loc']],
                    'parent': curr,
                    'time': curr['time']+1}
            
            # zero cost to wait at goal
            if child['loc'] == goal_loc:
                child['g_val'] = curr['g_val'] + 0.01 # very low cost for waiting at goal

            if PrioritizedPlanner.is_constrained(curr['loc'], child['loc'], curr['time']+1, constraint_table, goal_constraints, agent, goal_loc, workcell_constraints):
                    continue
            if (child['loc'], child['time']) in closed_list:
                existing_node = closed_list[(child['loc'],child['time'])]
                if PrioritizedPlanner.compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['time'])] = child
                    PrioritizedPlanner.push_node(open_list, child)
            else:
                closed_list[(child['loc'],child['time'])] = child
                PrioritizedPlanner.push_node(open_list, child)

            num_nodes_opened += 5
        raise Exception()  # Failed to find solutions, should never happen