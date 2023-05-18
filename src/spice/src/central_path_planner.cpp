#include "spice/central_path_planner.hpp"
#include "spice/planners/straight_line_planner.hpp"
#include "spice/planners/a_star_planner.hpp"
#include "spice/costmaps/prioritized_costmap.hpp"
#include "spice/costmaps/global_costmap.hpp"

CentralPathPlanner::CentralPathPlanner() : Node("central_path_planner_node")
{
    m_planner_service = create_service<spice_msgs::srv::GetPlan>(
        "/get_plan", 
        std::bind(&CentralPathPlanner::get_plan_cb, this, std::placeholders::_1, std::placeholders::_2));
    m_plan_pub = create_publisher<spice_msgs::msg::RobotPlan>("/robot_plans", 10);
    m_marker_array_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("/planned_paths", 10);
    m_debug_publish_timer = rclcpp::create_timer(this, 
                                                get_clock(), 
                                                rclcpp::Duration::from_seconds(DEBUG_PUBLISH_TIME),
                                                std::bind(&CentralPathPlanner::debug_publish_timer_cb, this));

    m_global_frame = "map";
    m_tolerance = 0.5;

    m_a_star_planner = std::make_unique<AStarPlanner>(*this);
    m_straight_line_planner = std::make_unique<StraightLinePlanner>(*this);
    m_global_costmap = std::make_unique<GlobalCostmap>(*this);
    m_prioritized_costmap = std::make_unique<PrioritizedCostmap>(*this);

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

    // set start and goal here, as it is used by some costmaps during planning    
    m_planned_paths[request->id.id].start = request->start;
    m_planned_paths[request->id.id].goal = request->goal;

    std::string planner_type = "Unknown";
    
    if(request->planner_type.type == spice_msgs::msg::PlannerType::PLANNER_PRIORITIZED)
    {
        auto prioritized_costmap = m_prioritized_costmap->get_costmap(request->id);
        auto prioritized_plan = m_a_star_planner->get_plan(request->start, request->goal, m_tolerance, prioritized_costmap, request->id);

        // failed to make prioritized plan, check if path is available in general
        if(prioritized_plan.poses.size() == 0)
        {
            auto global_costmap = m_global_costmap->get_costmap(request->id);
            auto a_star_plan = m_a_star_planner->get_plan(request->start, request->goal, m_tolerance, global_costmap, request->id);

            // plan is not possble in general
            if(a_star_plan.poses.size() == 0)
            {
                // empty path, with no wait, will make the navigation goal fail
                response->plan = nav_msgs::msg::Path();
                response->wait = false;
            }
            // robot is currently blocked by other robots
            // return valid path, but wait
            else
            {
                response->plan = a_star_plan;
                response->wait = true;
            }
        }
        else
        {
            response->plan = prioritized_plan;
            response->wait = false;
        }
        planner_type="Prioritized planner";
    }
    else if (request->planner_type.type == spice_msgs::msg::PlannerType::PLANNER_STRAIGHT_LINE)
    {
        response->plan = m_straight_line_planner->get_plan(request->start, request->goal, 0.05, nullptr, request->id);
        response->wait = false;
        planner_type = "Straight line planner";
    }
    else if(request->planner_type.type == spice_msgs::msg::PlannerType::PLANNER_A_STAR)
    {
        auto costmap = m_global_costmap->get_costmap(request->id);
        response->plan = m_a_star_planner->get_plan(request->start, request->goal, m_tolerance, costmap, request->id);
        response->wait = false;
        planner_type = "A*";
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Received request to plan with unknown planner: %d", request->planner_type.type);
        response->plan = nav_msgs::msg::Path();
    }
    
    auto duration = (now()-start_time).nanoseconds()*10e-9;
    m_planned_paths[request->id.id].plan = response->plan;
    m_planned_paths[request->id.id].timestamp = now();
    m_planned_paths[request->id.id].waiting = response->wait;

    RCLCPP_INFO(get_logger(), "Created a plan with %ld poses in %f ms using %s. Wait: %d", 
        response->plan.poses.size(), duration, planner_type.c_str(), response->wait);

    spice_msgs::msg::RobotPlan msg;
    msg.id = request->id;
    msg.plan = response->plan;
    m_plan_pub->publish(msg);
}


std::optional<robot_plan> CentralPathPlanner::get_last_plan_by_id(spice_msgs::msg::Id id)
{
    if(m_planned_paths.find(id.id) != m_planned_paths.end())
        return std::optional<robot_plan>{m_planned_paths.at(id.id)};
    else
        return {};
}


// convenience function to return color from hue in range 0-360
std_msgs::msg::ColorRGBA color_from_hue(double hue)
{
    double hh, p, q, t, ff;
    long i;
    std_msgs::msg::ColorRGBA out;
    out.a = 1.0;
    double saturation = 1.0;
    double value = 1.0;

    hh = hue;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = value * (1.0 - saturation);
    q = value * (1.0 - (saturation * ff));
    t = value * (1.0 - (saturation * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = value;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = value;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = value;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = value;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = value;
        break;
    case 5:
    default:
        out.r = value;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

void CentralPathPlanner::debug_publish_timer_cb()
{
    visualization_msgs::msg::MarkerArray msg;
    rclcpp::Time current_time = now();
    int num_paths = m_planned_paths.size();
    int i = 0; // for color
    for(auto& path : m_planned_paths)
    {
        auto age = current_time - path.second.timestamp;
        if(age.seconds() > DEBUG_PUBLISH_TIME*2.0)
        {   
            path.second.plan.poses.clear();
            continue;
        }
        auto color = color_from_hue((float)i++*(360.0/num_paths));
        visualization_msgs::msg::Marker marker;
        marker.action = 0;
        marker.type = 4;
        marker.scale.x = 0.05;
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        marker.header.stamp = now();
        marker.ns = path.first;
        marker.id = 0;
        marker.header.frame_id = path.second.plan.header.frame_id;
        marker.color = color;

        for(auto& pose : path.second.plan.poses)
        {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);

        visualization_msgs::msg::Marker goal_marker;
        goal_marker.action = 0;
        goal_marker.type = 2;
        goal_marker.scale.x = 0.1;
        goal_marker.scale.y = 0.1;
        goal_marker.scale.z = 0.1;
        goal_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        goal_marker.header.stamp = now();
        goal_marker.ns = path.first;
        goal_marker.id = 1;
        goal_marker.header.frame_id = path.second.plan.header.frame_id;
        goal_marker.color = color;
        goal_marker.pose = path.second.goal.pose;
        msg.markers.push_back(goal_marker);

        if(path.second.waiting)
        {
            visualization_msgs::msg::Marker wait_marker;
            wait_marker.type = 2; //sphere
            wait_marker.action = 0;
            wait_marker.scale.x = 0.2;
            wait_marker.pose = path.second.start.pose;
            wait_marker.id = 2;
            wait_marker.ns = path.first;
            wait_marker.header.frame_id = path.second.plan.header.frame_id;
            wait_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            std_msgs::msg::ColorRGBA yellow;
            yellow.a = 1.0;
            yellow.r = 0.0;
            yellow.g = 1.0;
            yellow.b = 1.0;
            wait_marker.color = yellow;
            msg.markers.push_back(wait_marker);
        }
    }

    m_marker_array_publisher->publish(std::move(msg));
}




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<CentralPathPlanner>();
    rclcpp::spin(planner);
}