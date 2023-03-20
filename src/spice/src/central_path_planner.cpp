#include "spice/central_path_planner.hpp"


CentralPathPlanner::CentralPathPlanner() : Node("central_path_planner_node")
{
    m_planner_service = create_service<nav_msgs::srv::GetPlan>(
        "/get_plan", 
        std::bind(&CentralPathPlanner::get_plan_cb, this, std::placeholders::_1, std::placeholders::_2));
    m_global_frame = "map";
};

void CentralPathPlanner::get_plan_cb(nav_msgs::srv::GetPlan::Request::SharedPtr request, nav_msgs::srv::GetPlan::Response::SharedPtr response)
{
    // Checking if the goal and start state is in the global frame
    if (request->start.header.frame_id != m_global_frame) {
        RCLCPP_ERROR(
            get_logger(), "Planner will only except start position from %s frame",
            m_global_frame.c_str());
        return;
    }

    if (request->goal.header.frame_id != m_global_frame) {
        RCLCPP_INFO(
            get_logger(), "Planner will only except goal position from %s frame",
            m_global_frame.c_str());
        return;
    }

    double interpolation_resolution = request->tolerance;

    response->plan.poses.clear();
    response->plan.header.stamp = now();
    response->plan.header.frame_id = m_global_frame;
    // calculating the number of loops for current value of interpolation_resolution_
    int total_number_of_loop = std::hypot(
        request->goal.pose.position.x - request->start.pose.position.x,
        request->goal.pose.position.y - request->start.pose.position.y) /
        interpolation_resolution;
    double x_increment = (request->goal.pose.position.x - request->start.pose.position.x) / total_number_of_loop;
    double y_increment = (request->goal.pose.position.y - request->start.pose.position.y) / total_number_of_loop;

    for (int i = 0; i < total_number_of_loop; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = request->start.pose.position.x + x_increment * i;
        pose.pose.position.y = request->start.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = now();
        pose.header.frame_id = m_global_frame;
        response->plan.poses.push_back(pose);
    }

    response->plan.poses.push_back(request->goal);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<CentralPathPlanner>();
    rclcpp::spin(planner);
}