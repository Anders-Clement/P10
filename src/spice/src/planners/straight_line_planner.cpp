#include "spice/planners/straight_line_planner.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include <cmath>

nav_msgs::msg::Path StraightLinePlanner::get_plan(
    geometry_msgs::msg::PoseStamped start, 
    geometry_msgs::msg::PoseStamped goal,
    double goal_tolerance,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
    spice_msgs::msg::Id id)
{
    nav_msgs::msg::Path plan;
    plan.header.stamp = m_central_path_planner.now();
    plan.header.frame_id = m_global_frame;
    // calculating the number of loops for current value of interpolation_resolution_
    int total_number_of_loop = std::hypot(
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y) /
        goal_tolerance;
    double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

    for (int i = 0; i < total_number_of_loop; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = start.pose.position.x + x_increment * i;
        pose.pose.position.y = start.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = m_central_path_planner.now();
        pose.header.frame_id = m_global_frame;
        plan.poses.push_back(pose);
    }

    plan.poses.push_back(goal);

    return plan;
}