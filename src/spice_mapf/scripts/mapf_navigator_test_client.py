import time
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus
import spice_mapf_msgs.action as spice_mapf_actions
import spice_mapf_msgs.msg as spice_mapf_msgs

from Map import Map

class ClientNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.client = ActionClient(self, spice_mapf_actions.NavigateMapf, "navigate_mapf")
        self.map = Map(self)
        self.x, self.y = 5,5
        self.run()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if not self.map.has_map:
                self.get_logger().info('Awaiting map', once=True)
                continue

            goal = spice_mapf_actions.NavigateMapf.Goal()
            x,y = 0,0
            while x < 6.5:
                x,y = self.map.map_to_world(self.map.get_random_freespace())
            # map_y, map_x = self.map.world_to_map(spice_mapf_msgs.Position(x=float(self.x), y=float(self.y)))
            # if self.map.map[map_y][map_x]:
            #     self.x -= 1
            # else:
            #     self.x += 0.5
            goal.goal_pose.pose.position.x = x
            goal.goal_pose.pose.position.y = y
            
            while not self.client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for action server...')
            future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
            rclpy.spin_until_future_complete(self, future)
            result: ClientGoalHandle = future.result()
            if not result.accepted:
                self.get_logger().info(f'goal not accepted')
                time.sleep(1)
                continue
            self.get_logger().info(f'Goal was accepted: {goal}')
            future = result.get_result_async()
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'Action result: {future.result()}')

    def feedback_cb(self, feedback):
        self.get_logger().info(f'Feedback: {feedback}')


if __name__ == "__main__":
    rclpy.init()
    client = ClientNode()
    rclpy.spin(client)