#ifndef CENTRAL_PATH_PLANNER_HPP
#define CENTRAL_PATH_PLANNER_HPP

#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "spice_msgs/srv/get_plan.hpp"
#include "spice/planners/global_planner.hpp"
#include "spice/costmaps/costmap.hpp"

#define DEBUG_PUBLISH_TIME 1.0

struct robot_plan{
    nav_msgs::msg::Path plan;
    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped goal;
    rclcpp::Time timestamp;

};

class Costmap;
class GlobalPlanner;

class CentralPathPlanner : public rclcpp::Node
{
public:
    CentralPathPlanner();
    std::shared_ptr<nav2_costmap_2d::Costmap2D> get_costmap(spice_msgs::msg::Id id);
    robot_plan& get_last_plan_by_id(spice_msgs::msg::Id id);

private:
    void get_plan_cb(spice_msgs::srv::GetPlan::Request::SharedPtr request, spice_msgs::srv::GetPlan::Response::SharedPtr response);
    void debug_publish_timer_cb();
    std::unique_ptr<GlobalPlanner> m_straight_line_planner;
    std::unique_ptr<GlobalPlanner> m_a_star_planner;
    std::unique_ptr<Costmap> m_costmap;
    rclcpp::Service<spice_msgs::srv::GetPlan>::SharedPtr m_planner_service;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_array_publisher;
    rclcpp::TimerBase::SharedPtr m_debug_publish_timer;
    std::unordered_map<std::string, robot_plan> m_planned_paths;
    std::string m_global_frame;
    double m_tolerance;
};

#endif // CENTRAL_PATH_PLANNER_HPP