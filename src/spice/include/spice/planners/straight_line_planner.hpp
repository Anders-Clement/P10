#include "spice/planners/global_planner.hpp"

class StraightLinePlanner : public GlobalPlanner
{
public:
    StraightLinePlanner() = delete;
    StraightLinePlanner(rclcpp::Node::SharedPtr node) {m_node = node;};

    nav_msgs::msg::Path get_plan(
        geometry_msgs::msg::PoseStamped start, 
        geometry_msgs::msg::PoseStamped goal,
        double goal_tolerance,
        spice_msgs::msg::Id id) override;
};