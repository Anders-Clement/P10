#ifndef CENTRAL_PATH_PLANNER_HPP
#define CENTRAL_PATH_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "spice_msgs/srv/get_plan.hpp"
#include "spice/planners/global_planner.hpp"


class CentralPathPlanner : public rclcpp::Node
{
public:
    CentralPathPlanner();

private:
    void get_plan_cb(spice_msgs::srv::GetPlan::Request::SharedPtr request, spice_msgs::srv::GetPlan::Response::SharedPtr response);

    std::unique_ptr<GlobalPlanner> planner;
    rclcpp::Service<spice_msgs::srv::GetPlan>::SharedPtr m_planner_service;
    std::string m_global_frame;
    double m_tolerance;
};

#endif