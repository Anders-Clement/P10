#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include "spice_mapf_msgs/action/navigate_mapf.hpp"
#include "spice_mapf_msgs/srv/request_goal.hpp"
#include "spice_mapf_msgs/srv/join_planner.hpp"
#include "spice_mapf_msgs/msg/robot_pose.hpp"
#include "spice_mapf_msgs/msg/robot_poses.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/msg/robot_type.hpp"

#define AT_GOAL_THRESHOLD 0.1 // todo: make parameter
#define PARAM_min_linear_vel 0.1 // todo: make parameter
#define PARAM_max_linear_vel 0.4 // todo: make parameter
#define PARAM_min_angular_vel 0.4 // todo: make parameter
#define PARAM_max_angular_vel 1.56 // todo: make parameter
#define PARAM_gamma_max 0.25 // todo: make parameter
#define PARAM_kp_omega 1.0 // todo: make parameter
#define PARAM_kp_linear_vel 1.0 // todo: make parameter
#define PARAM_goal_tolerance 0.1 // todo: make parameter

double quat_to_yaw (geometry_msgs::msg::Quaternion quat)
{
    tf2::Quaternion q(
    quat.x,
    quat.y,
    quat.z,
    quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
};

class MAPFNavigatorActionServer;
class MAPFNavigator : public rclcpp::Node
{
public:
    MAPFNavigator();
    bool is_available_for_navigation();

    spice_msgs::msg::Id id;
    geometry_msgs::msg::TransformStamped current_transform;
private:
    void mapf_paths_cb(spice_mapf_msgs::msg::RobotPoses::SharedPtr msg);
    bool get_robot_transform();
    bool at_step_goal();
    void try_join_planner();
    void control_loop();
    std::optional<geometry_msgs::msg::Twist> compute_cmd_vel(geometry_msgs::msg::PoseStamped goal_pose);
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<MAPFNavigatorActionServer> navigator_action_server;
    rclcpp::Subscription<spice_mapf_msgs::msg::RobotPoses>::SharedPtr mapf_paths_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<spice_mapf_msgs::msg::RobotPose>::SharedPtr robot_pos_publisher;
    rclcpp::Client<spice_mapf_msgs::srv::JoinPlanner>::SharedPtr join_planner_client;
    rclcpp::TimerBase::SharedPtr try_join_planner_timer;
    rclcpp::TimerBase::SharedPtr control_loop_timer;

    std::unique_ptr<spice_mapf_msgs::msg::RobotPose> current_nav_step_goal;
    std::unique_ptr<spice_mapf_msgs::msg::RobotPose> next_nav_step_goal;
    bool joined_planner = false;
};

class MAPFNavigatorActionServer
{
public:
    MAPFNavigatorActionServer(MAPFNavigator* navigator);
private:
    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const spice_mapf_msgs::action::NavigateMapf::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<spice_mapf_msgs::action::NavigateMapf>> _goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<spice_mapf_msgs::action::NavigateMapf>> _goal_handle);
    void execute();
    bool at_goal();
    void end_execution();
public:
    std::shared_ptr<spice_mapf_msgs::msg::RobotPose> current_nav_goal;

private:
    rclcpp::Client<spice_mapf_msgs::srv::RequestGoal>::SharedPtr req_goal_client;
    rclcpp_action::Server<spice_mapf_msgs::action::NavigateMapf>::SharedPtr action_server;
    MAPFNavigator* navigator;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<spice_mapf_msgs::action::NavigateMapf>> goal_handle;

};