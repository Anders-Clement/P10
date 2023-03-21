#include "spice/central_path_planner.hpp"
#include "spice/planners/straight_line_planner.hpp"


CentralPathPlanner::CentralPathPlanner() : Node("central_path_planner_node")
{
    m_planner_service = create_service<spice_msgs::srv::GetPlan>(
        "/get_plan", 
        std::bind(&CentralPathPlanner::get_plan_cb, this, std::placeholders::_1, std::placeholders::_2));
    m_global_frame = "map";
    m_tolerance = 0.05;

    RCLCPP_INFO(get_logger(), "Central path planner is initialized");
};

void CentralPathPlanner::get_plan_cb(
    spice_msgs::srv::GetPlan::Request::SharedPtr request,
    spice_msgs::srv::GetPlan::Response::SharedPtr response)
{
    if(!planner)
    {
        planner = std::make_unique<StraightLinePlanner>(shared_from_this());
    }
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
    
    response->plan = planner->get_plan(request->start, request->goal, m_tolerance, request->id);

    RCLCPP_INFO(get_logger(), "Created a plan with %ld poses", response->plan.poses.size());
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<CentralPathPlanner>();
    rclcpp::spin(planner);
}