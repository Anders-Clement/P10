#include "spice/central_path_planner.hpp"
#include "spice/planners/a_star_planner.hpp"
#include "spice/costmaps/global_costmap.hpp"

CentralPathPlanner::CentralPathPlanner() : Node("central_path_planner_node")
{
    m_planner_service = create_service<spice_msgs::srv::GetPlan>(
        "/get_plan", 
        std::bind(&CentralPathPlanner::get_plan_cb, this, std::placeholders::_1, std::placeholders::_2));
    m_global_frame = "map";
    m_tolerance = 0.05;

    m_planner = std::make_unique<AStarPlanner>(*this);
    m_costmap = std::make_unique<GlobalCostmap>(*this);

    RCLCPP_INFO(get_logger(), "Central path planner is initialized");
};

void CentralPathPlanner::get_plan_cb(
    spice_msgs::srv::GetPlan::Request::SharedPtr request,
    spice_msgs::srv::GetPlan::Response::SharedPtr response)
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
    auto start_time = now();
    response->plan = m_planner->get_plan(request->start, request->goal, m_tolerance, request->id);
    auto duration = (now()-start_time).nanoseconds()*10e-9;

    m_planned_paths[request->id.id] = response->plan;

    RCLCPP_INFO(get_logger(), "Created a plan with %ld poses in %f ms", response->plan.poses.size(), duration);
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> CentralPathPlanner::get_costmap(spice_msgs::msg::Id id)
{
    return m_costmap->get_costmap(id);
}

nav_msgs::msg::Path& CentralPathPlanner::get_last_plan_by_id(spice_msgs::msg::Id id)
{
    return m_planned_paths[id.id];
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<CentralPathPlanner>();
    rclcpp::spin(planner);
}