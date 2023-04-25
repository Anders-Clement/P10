#ifndef STRAIGHT_LINE_PLANNER_HPP
#define STRAIGHT_LINE_PLANNER_HPP

#include "spice/planners/global_planner.hpp"

class StraightLinePlanner : public GlobalPlanner
{
public:
    StraightLinePlanner() = delete;
    StraightLinePlanner(CentralPathPlanner& central_path_planner) : GlobalPlanner(central_path_planner) {};

    nav_msgs::msg::Path get_plan(
        geometry_msgs::msg::PoseStamped start, 
        geometry_msgs::msg::PoseStamped goal,
        double goal_tolerance,
        std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
        spice_msgs::msg::Id id) override;
};

#endif // STRAIGHT_LINE_PLANNER_HPP