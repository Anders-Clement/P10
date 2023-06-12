from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
import tf2_ros
import tf2_geometry_msgs
import rclpy
from tf2_ros import TransformException
import math
import tf_transformations

class MAPFController():
    def __init__(self, nodehandle: Node, tf_buffer: tf2_ros.buffer.Buffer):
        self.nodehandle = nodehandle
        self.tf_buffer = tf_buffer

        self.nodehandle.declare_parameter('min_linear_vel', 0.1)
        self.nodehandle.declare_parameter('max_linear_vel', 0.4)
        self.nodehandle.declare_parameter('min_angular_vel', 0.1)
        self.nodehandle.declare_parameter('max_angular_vel', 1.56)
        self.nodehandle.declare_parameter('gamma_max', 0.25)
        self.nodehandle.declare_parameter('kp_omega', 1.0)
        self.nodehandle.declare_parameter('kp_linear_vel', 1.0)
        self.nodehandle.declare_parameter('goal_tolerance', 0.1)

    def compute_cmd_vel(self, robot_transform: TransformStamped, goal_pose: PoseStamped) -> Twist | None:
        self.min_linear_vel = self.nodehandle.get_parameter('min_linear_vel').get_parameter_value().double_value
        self.max_linear_vel = self.nodehandle.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.min_angular_vel = self.nodehandle.get_parameter('min_angular_vel').get_parameter_value().double_value
        self.max_angular_vel = self.nodehandle.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.gamma_max = self.nodehandle.get_parameter('gamma_max').get_parameter_value().double_value
        self.kp_omega = self.nodehandle.get_parameter('kp_omega').get_parameter_value().double_value
        self.kp_linear_vel = self.nodehandle.get_parameter('kp_linear_vel').get_parameter_value().double_value
        self.goal_tolerance = self.nodehandle.get_parameter('goal_tolerance').get_parameter_value().double_value

        try:
            transform = self.tf_buffer.lookup_transform("base_link",
                                                        "map",
                                                        robot_transform.header.stamp)

            goal_in_base_link = tf2_geometry_msgs.do_transform_pose(goal_pose.pose, transform)
        except TransformException as e:
            self.nodehandle.get_logger().error(
                        f'Could not transform robot or goal pose to map in MAPFController: {e}', once=True)
            return None

        twist_msg = Twist()

        x_diff = robot_transform.transform.translation.x - goal_pose.pose.position.x
        y_diff = robot_transform.transform.translation.y - goal_pose.pose.position.y
        goal_dist = math.sqrt(x_diff**2 + y_diff**2)
        py_clip = lambda x, lower, upper: lower if x < lower else upper if x > upper else x
        # at the goal, turn to face it:
        if goal_dist < self.goal_tolerance:
            q = goal_pose.pose.orientation
            q = [q.w, q.x, q.y, q.z]
            goal_rpy = tf_transformations.euler_from_quaternion(q)
            q = robot_transform.transform.rotation
            q = [q.w, q.x, q.y, q.z]
            robot_rpy = tf_transformations.euler_from_quaternion(q)
            goal_yaw = goal_rpy[2]
            robot_yaw = robot_rpy[2]
            PI = 3.1416
            # figure out which way to turn: https://math.stackexchange.com/questions/110080/shortest-way-to-achieve-target-angle
            robot_yaw += PI
            goal_yaw += PI
            a = robot_yaw
            b = goal_yaw
            if a < b:
                if b-a <= PI: # positive
                    omega = py_clip(b-a, 0, self.max_angular_vel)
                else: # negative
                    omega = py_clip(-(b-a), -self.max_angular_vel, 0)
            else:
                if a-b <= PI: # negative
                    omega = py_clip(-(a-b), -self.max_angular_vel, 0)
                else: # positive
                    omega = py_clip(a-b, 0, self.max_angular_vel)

            if abs(omega) < self.gamma_max: # we are at correct orientation
                omega = 0.0 
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = omega*self.kp_omega
            return twist_msg

        else: # navigate towards the goal
        
            angular_error = math.atan2(goal_in_base_link.position.y, goal_in_base_link.position.x)
            # is facing the goal, within gamma_max
            if goal_in_base_link.position.x > 0 and abs(angular_error) < self.gamma_max:
                # proportional control
                omega = self.kp_omega * angular_error
                velocity = py_clip(goal_dist * self.kp_linear_vel, self.min_linear_vel, self.max_linear_vel)

                twist_msg.linear.x = velocity
                twist_msg.angular.z = omega
                return twist_msg
            
            else: # not facing the goal, turn to face it
                if goal_in_base_link.position.x > 0:
                    omega = angular_error
                else:
                    if goal_in_base_link.position.y > 0:
                        omega = self.max_angular_vel
                    else:
                        omega = -self.max_angular_vel

                twist_msg.linear.x = 0.0
                twist_msg.angular.z = omega
                return twist_msg

