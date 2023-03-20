#ifndef CENTRAL_PATH_PLANNER_HPP
#define CENTRAL_PATH_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_plan.hpp"


class CentralPathPlanner : public rclcpp::Node
{
public:
    CentralPathPlanner();

private:
    void get_plan_cb(nav_msgs::srv::GetPlan::Request::SharedPtr request, nav_msgs::srv::GetPlan::Response::SharedPtr response);

    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr m_planner_service;
    std::string m_global_frame;
};

#endif