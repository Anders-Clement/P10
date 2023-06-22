#include "spice_mapf/mapf_navigator.hpp"

MAPFNavigatorActionServer::MAPFNavigatorActionServer(MAPFNavigator* navigator) : navigator(navigator)
{
    using namespace std::placeholders;
    req_goal_client = navigator->create_client<spice_mapf_msgs::srv::RequestGoal>("/request_goal");
    action_server = rclcpp_action::create_server<spice_mapf_msgs::action::NavigateMapf>(
    navigator,
    "navigate_mapf",
    std::bind(&MAPFNavigatorActionServer::handle_goal, this, _1, _2),
    std::bind(&MAPFNavigatorActionServer::handle_cancel, this, _1),
    std::bind(&MAPFNavigatorActionServer::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse MAPFNavigatorActionServer::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const spice_mapf_msgs::action::NavigateMapf::Goal> goal)
{
    RCLCPP_INFO(navigator->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    if(goal_handle)
        return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MAPFNavigatorActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<spice_mapf_msgs::action::NavigateMapf>> _goal_handle)
{
    RCLCPP_INFO(navigator->get_logger(), "Received request to cancel goal, this is not possible");
    (void)_goal_handle;
    return rclcpp_action::CancelResponse::REJECT;
}

void MAPFNavigatorActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<spice_mapf_msgs::action::NavigateMapf>> _goal_handle)
{
    goal_handle = _goal_handle;
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MAPFNavigatorActionServer::execute, this)}.detach();
}

void MAPFNavigatorActionServer::execute()
{
    RCLCPP_INFO(navigator->get_logger(), "Executing goal");
    auto action_result = std::make_shared<spice_mapf_msgs::action::NavigateMapf::Result>();
    auto const goal = goal_handle->get_goal();
    current_nav_goal = std::make_shared<spice_mapf_msgs::msg::RobotPose>();
    current_nav_goal->id = navigator->id;
    current_nav_goal->position.x = goal->goal_pose.pose.position.x;
    current_nav_goal->position.y = goal->goal_pose.pose.position.y;

    // request the goal at the planner
    spice_mapf_msgs::srv::RequestGoal::Request::SharedPtr goal_request = std::make_shared<spice_mapf_msgs::srv::RequestGoal::Request>();
    goal_request->robot_pose = *current_nav_goal;
    goal_request->workcell_id = goal->workcell_id;

    using namespace std::chrono_literals;
    while(!req_goal_client->wait_for_service(1s))
    {
        RCLCPP_INFO(navigator->get_logger(), "Timeout on /request_goal, retrying...");
    }

    while(1)
    {
        auto future = req_goal_client->async_send_request(goal_request);
        future.wait();
        auto result = future.get();
        
        if(!result->success && result->currently_occupied)
        {
            RCLCPP_INFO(navigator->get_logger(), "Requested goal is currently blocked by other robot. Waiting 1 second...");
            std::this_thread::sleep_for(1s);
        }
        else if(!result->success)
        {
            RCLCPP_INFO(navigator->get_logger(), "Requested goal is invalid, or navigator has not joined planner yet, aborting action");
            action_result->success = false;
            goal_handle->abort(action_result);
            end_execution();
            return;
        }
        else if(result->success && !result->currently_occupied)
        {
            current_nav_goal->position = result->goal_position;
            break;
        }
    }
    // got a plan, wait for it to execute

    while(!at_goal())
    {
        std::this_thread::sleep_for(10ms);
    }

    // Check if goal is done
    if (rclcpp::ok()) {
        action_result->success = true;
        goal_handle->succeed(action_result);
        RCLCPP_INFO(navigator->get_logger(), "Goal succeeded");
    }
    goal_handle.reset();
}

bool MAPFNavigatorActionServer::at_goal()
{
    // no nav goal, assume we are there
    if(!current_nav_goal)
        return true;

    double x_diff = current_nav_goal->position.x - navigator->current_transform.transform.translation.x;
    double y_diff = current_nav_goal->position.y - navigator->current_transform.transform.translation.y;
    double distance_to_goal = std::sqrt(x_diff*x_diff + y_diff*y_diff);
    
    double goal_yaw = quat_to_yaw(current_nav_goal->heading);
    double robot_yaw = quat_to_yaw(navigator->current_transform.transform.rotation);

    double rotation_to_goal = std::abs(goal_yaw - robot_yaw); 
    return distance_to_goal < AT_GOAL_THRESHOLD && 0.0 < AT_GOAL_THRESHOLD;
}

void MAPFNavigatorActionServer::end_execution()
{
    current_nav_goal.reset();
}

MAPFNavigator::MAPFNavigator() : Node("mapf_navigator")
{
    std:: string ns = std::string(this->get_namespace()).substr(1,std::string(this->get_namespace()).rfind('/')-1);
    if(ns == "")
    {
        RCLCPP_ERROR(this->get_logger(), 
            "ROBOT_NAMESPACE is empty");
    }
    id.id = ns;
    id.robot_type.type = spice_msgs::msg::RobotType::CARRIER_ROBOT;
    RCLCPP_INFO(get_logger(), "Found robot namespace: %s", id.id.c_str());

    navigator_action_server = std::make_unique<MAPFNavigatorActionServer>(this);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    int QUEUE_DEPTH = 1;
    rclcpp::QoS qos_best_effort = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, QUEUE_DEPTH), qos_profile);
    cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_best_effort);
    robot_pos_publisher = create_publisher<spice_mapf_msgs::msg::RobotPose>("/robot_pos", qos_best_effort);
    mapf_paths_subscriber = create_subscription<spice_mapf_msgs::msg::RobotPoses>(
        "/mapf_paths", 
        10,
        std::bind(&MAPFNavigator::mapf_paths_cb, this, std::placeholders::_1));

    join_planner_client = create_client<spice_mapf_msgs::srv::JoinPlanner>("/join_planner");
    try_join_planner_timer = rclcpp::create_timer(this, get_clock(), 
        rclcpp::Duration::from_seconds(1.0),
        std::bind(&MAPFNavigator::try_join_planner, this));
    try_join_planner_timer->cancel(); // only used for retries
    try_join_planner();

    control_loop_timer = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(0.1),
        std::bind(&MAPFNavigator::control_loop, this));
}

void MAPFNavigator::control_loop()
{
    if(!get_robot_transform())
    {
        return;
    }
    if(joined_planner)
    {
        auto robot_pose_msg = spice_mapf_msgs::msg::RobotPose();
        robot_pose_msg.position.x = current_transform.transform.translation.x;
        robot_pose_msg.position.y = current_transform.transform.translation.y;
        robot_pose_msg.heading = current_transform.transform.rotation;
        robot_pose_msg.id = id;
        robot_pos_publisher->publish(robot_pose_msg);
    }
    if(at_step_goal())
    {
        if(next_nav_step_goal)
        {
            *current_nav_step_goal = *next_nav_step_goal;
            next_nav_step_goal.reset();
        }
    }

    // TODO: pass goal pose and current transform to controller, and compute cmd_vel
    auto goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.header.stamp = current_transform.header.stamp;
    goal_pose.header.frame_id = "map";
    if(current_nav_step_goal)
    {
        goal_pose.pose.position.x = current_nav_step_goal->position.x;
        goal_pose.pose.position.y = current_nav_step_goal->position.y;
        goal_pose.pose.orientation = current_nav_step_goal->heading;
    }
    else
    {
        goal_pose.pose.position.x = current_transform.transform.translation.x;
        goal_pose.pose.position.y = current_transform.transform.translation.y;
        goal_pose.pose.orientation = current_transform.transform.rotation;
    }
    auto cmd_vel = compute_cmd_vel(goal_pose);
    if(cmd_vel)
    {
        cmd_vel_publisher->publish(cmd_vel.value());
    }
    else
    {
        cmd_vel_publisher->publish(geometry_msgs::msg::Twist());
    }
}

std::optional<geometry_msgs::msg::Twist> MAPFNavigator::compute_cmd_vel(geometry_msgs::msg::PoseStamped goal_pose)
{
    std::string target_frame = id.id + "_base_link";
    std::string from_frame = "map";
    geometry_msgs::msg::TransformStamped map_to_base_link;
    try {
        map_to_base_link = tf_buffer->lookupTransform(
        target_frame, from_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) 
    {
        RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
            target_frame.c_str(), from_frame.c_str(), ex.what());
        return {};
    }

    double x_diff = current_transform.transform.translation.x - goal_pose.pose.position.x;
    double y_diff = current_transform.transform.translation.y - goal_pose.pose.position.y;
    double goal_distance = std::sqrt(x_diff*x_diff + y_diff*y_diff);
    geometry_msgs::msg::Twist twist_msg;
    // at the goal, turn to face it
    if(goal_distance < PARAM_goal_tolerance)
    {
        double goal_yaw = quat_to_yaw(goal_pose.pose.orientation);
        double robot_yaw = quat_to_yaw(current_transform.transform.rotation);
        double PI = 3.1415;
        robot_yaw += PI;
        goal_yaw += PI;
        double omega;
        if(robot_yaw < goal_yaw)
        {
            if(goal_yaw-robot_yaw <= PI)
            {
                omega = std::clamp(goal_yaw-robot_yaw, 0.0, PARAM_max_angular_vel);
            }
            else
            {
                omega = std::clamp(-(goal_yaw-robot_yaw), -PARAM_max_angular_vel, 0.0);
            }
        }
        else
        {
            if(robot_yaw-goal_yaw <= PI)
            {
                omega = std::clamp(-(robot_yaw-goal_yaw), -PARAM_max_angular_vel, 0.0);
            }
            else
            {
                omega = std::clamp(robot_yaw-goal_yaw, 0.0, PARAM_max_angular_vel);
            }

        }
        if(std::abs(omega) < PARAM_gamma_max)
        {
            omega = 0.0;
        }
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = omega*PARAM_kp_omega;
        return {twist_msg};
    }
    else
    {
        geometry_msgs::msg::PoseStamped goal_in_base_link = 
            tf_buffer->transform(goal_pose, current_transform.child_frame_id);

        double angular_error = std::atan2(goal_in_base_link.pose.position.y, goal_in_base_link.pose.position.x);
        double omega, velocity;
        if(goal_in_base_link.pose.position.x > 0 && std::abs(angular_error) < PARAM_gamma_max)
        {
            omega = angular_error*PARAM_kp_omega;
            velocity = std::clamp(goal_distance*PARAM_kp_linear_vel, PARAM_min_linear_vel, PARAM_max_linear_vel);
        }
        else
        {
            velocity = 0.0;
            if(goal_in_base_link.pose.position.x > 0)
            {
                omega = angular_error*PARAM_kp_omega;
            }
            else
            {
                if(goal_in_base_link.pose.position.y > 0)
                {
                    omega = PARAM_max_angular_vel;
                }
                else
                {
                    omega = -PARAM_max_angular_vel;
                }
            }
        }
        twist_msg.linear.x = velocity;
        twist_msg.angular.z = omega;
        return {twist_msg};
    }
}

void MAPFNavigator::mapf_paths_cb(spice_mapf_msgs::msg::RobotPoses::SharedPtr msg)
{
    for(auto pose : msg->poses)
    {
        // only consider message for this navigator
        if(pose.id == id)
        {
            if(pose.rejoin)
            {
                joined_planner = false;
                try_join_planner();
                next_nav_step_goal.reset();
                navigator_action_server->current_nav_goal.reset();
                return;
            }

            if(pose != *current_nav_step_goal)
            {
                RCLCPP_INFO(get_logger(), "Got new pose from path: %f,%f", pose.position.x, pose.position.y);
                if(at_step_goal())
                {
                    current_nav_step_goal = std::make_unique<spice_mapf_msgs::msg::RobotPose>(pose);
                }
                else
                {
                    next_nav_step_goal = std::make_unique<spice_mapf_msgs::msg::RobotPose>(pose);
                }
            }   
        }
    }
}

bool MAPFNavigator::get_robot_transform()
{
    std::string from_frame_rel = id.id + "_base_link";
    std::string to_frame_rel = "map";
    try {
        current_transform = tf_buffer->lookupTransform(
        to_frame_rel, from_frame_rel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) 
    {
        RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
            to_frame_rel.c_str(), from_frame_rel.c_str(), ex.what());
        return false;
    }

    return true;
}

bool MAPFNavigator::is_available_for_navigation()
{
    return get_robot_transform() && joined_planner;
}

bool MAPFNavigator::at_step_goal()
{
    // if no goal is given, assume robot is at step goal
    if(!current_nav_step_goal)
        return true;
    double x_diff = current_nav_step_goal->position.x - current_transform.transform.translation.x;
    double y_diff = current_nav_step_goal->position.y - current_transform.transform.translation.y;
    double distance_to_step_goal = std::sqrt(x_diff*x_diff + y_diff*y_diff);

    return distance_to_step_goal < AT_GOAL_THRESHOLD;
}

void MAPFNavigator::try_join_planner()
{
    try_join_planner_timer->cancel();
    if(joined_planner)
    {
        RCLCPP_WARN(get_logger(), "Trying to join planner, but is already joined!");
        return;
    }
    if(!get_robot_transform())
    {
        try_join_planner_timer->reset();
        return;
    }
    using namespace std::chrono_literals;
    while(!join_planner_client->wait_for_service(1s))
    {
        RCLCPP_INFO(get_logger(), "timeout on join_planner service");
    }
    // to ensure up to date transform after waiting for service
    get_robot_transform();

    auto request_msg = std::make_shared<spice_mapf_msgs::srv::JoinPlanner::Request>();
    request_msg->robot_pose.id = id;
    request_msg->robot_pose.heading = current_transform.transform.rotation;

    if(current_nav_step_goal)
    {
        request_msg->robot_pose.position.x = current_nav_step_goal->position.x;
        request_msg->robot_pose.position.y = current_nav_step_goal->position.y;
    }
    else
    {
        request_msg->robot_pose.position.x = current_transform.transform.translation.x;
        request_msg->robot_pose.position.y = current_transform.transform.translation.y;
        // controller will move robot to position in request
        current_nav_step_goal = std::make_unique<spice_mapf_msgs::msg::RobotPose>(request_msg->robot_pose);
    }

    join_planner_client->async_send_request(request_msg, 
        [this](const rclcpp::Client<spice_mapf_msgs::srv::JoinPlanner>::SharedFuture future)
        {
            std::shared_ptr<spice_mapf_msgs::srv::JoinPlanner_Response> result = future.get();

            if(result->success)
            {
                joined_planner = true;
                RCLCPP_INFO(get_logger(), "Agent %s joined mapf planner", id.id.c_str());
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Agent %s failed to join mapf planner, retrying in 1 seconds",
                    id.id.c_str());
                try_join_planner_timer->reset();
            }
        } 
    );
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto executor = rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 4);
    auto navigator = MAPFNavigator();
    executor.add_node(navigator.get_node_base_interface());
    executor.spin();
    return 0;
}