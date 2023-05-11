import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus

import spice_mapf_msgs.action as spice_mapf_actions

class ClientNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.client = ActionClient(self, spice_mapf_actions.NavigateMapf, "navigate_mapf")
        self.x = 4.0
        
        self.run()

    def run(self):
        self.get_logger().info(f'Running')
        goal = spice_mapf_actions.NavigateMapf.Goal()
        goal.goal_pose.pose.position.x = 4.0
        goal.goal_pose.pose.position.y = 7.0
        
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for action server...')
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result: ClientGoalHandle = future.result()
        if not result.accepted:
            self.get_logger().info(f'goal not accepted')
            return
        self.get_logger().info(f'Goal was accepted')
        future = result.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Action result: {future.result()}')

        self.x += 0.5
        self.run()

if __name__ == "__main__":
    rclpy.init()
    client = ClientNode()
    rclpy.spin(client)