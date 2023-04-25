#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice/central_path_planner.hpp"

class CentralPathPlanner;

class GlobalPlanner
{
public:
    GlobalPlanner() = delete;
    GlobalPlanner(CentralPathPlanner& central_path_planner);
    // Planning function, poses have been sanity checked
    // should return path from start pose to goal pose within the given tolerance
    virtual nav_msgs::msg::Path get_plan(
        geometry_msgs::msg::PoseStamped start, 
        geometry_msgs::msg::PoseStamped goal,
        double goal_tolerance,
        std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
        spice_msgs::msg::Id id) = 0;

protected:
    std::string m_global_frame = "map";
    CentralPathPlanner& m_central_path_planner;
};

#endif // GLOBAL_PLANNER_HPP