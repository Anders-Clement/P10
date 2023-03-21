#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spice_msgs/msg/id.hpp"

class GlobalPlanner
{
public:
    // Planning function, poses have been sanity checked
    // should return path from start pose to goal pose within the given tolerance
    virtual nav_msgs::msg::Path get_plan(
        geometry_msgs::msg::PoseStamped start, 
        geometry_msgs::msg::PoseStamped goal,
        double goal_tolerance,
        spice_msgs::msg::Id id) = 0;

protected:
    std::string m_global_frame = "map";
    rclcpp::Node::SharedPtr m_node;
};

#endif // GLOBAL_PLANNER_HPP