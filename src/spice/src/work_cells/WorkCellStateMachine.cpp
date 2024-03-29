#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include <chrono>
#include "tf2/LinearMath/Matrix3x3.h"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"

using namespace std::chrono_literals;

WorkCellStateMachine::WorkCellStateMachine(std::string work_cell_name, rclcpp::Node& node_handle, 
        geometry_msgs::msg::Transform transform, spice_msgs::msg::RobotType::_type_type robot_type)
:  m_nodehandle(node_handle), m_work_cell_name(work_cell_name), m_robot_type(robot_type), m_transform(transform)
{
    // m_nodehandle.declare_parameter("robot_radius",rclcpp::ParameterValue(0.25));
    // m_nodehandle.declare_parameter("workcell_radius",rclcpp::ParameterValue(0.25));
    // ROBOT_RADIUS = m_nodehandle.get_parameter("robot_radius").as_double();
    // WORKCELL_RADIUS = m_nodehandle.get_parameter("workcell_radius").as_double();
    m_work_cell_queue_manager = std::make_unique<WorkCellQueuePositionManager>(*this);
    m_register_work_service = m_nodehandle.create_service<spice_msgs::srv::RegisterWork>(
        m_work_cell_name + "/register_work", 
        std::bind(&WorkCellStateMachine::on_register_work, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_ready_for_processing_service = m_nodehandle.create_service<std_srvs::srv::Trigger>(
        m_work_cell_name + "/robot_ready_for_processing", 
        std::bind(&WorkCellStateMachine::on_robot_ready_for_processing, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_ready_in_queue_service = m_nodehandle.create_service<spice_msgs::srv::RobotReady>(
        m_work_cell_name + "/robot_ready_in_queue", 
        std::bind(&WorkCellStateMachine::on_robot_ready_in_queue, this, std::placeholders::_1, std::placeholders::_2));
    m_heartbeat_service = m_nodehandle.create_service<spice_msgs::srv::Heartbeat>(
        m_work_cell_name + "/heartbeat",
        std::bind(&WorkCellStateMachine::on_robot_heartbeat, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_exited_service = m_nodehandle.create_service<std_srvs::srv::Trigger>(m_work_cell_name + "/robot_exited",
        std::bind(&WorkCellStateMachine::on_robot_exited, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_heartbeat_timer = rclcpp::create_timer(&m_nodehandle, m_nodehandle.get_clock(), 
        rclcpp::Duration::from_seconds(ROBOT_HEARTBEAT_TIMEOUT_PERIOD),
        std::bind(&WorkCellStateMachine::check_robot_heartbeat_cb, this));

   

    

    auto qos_profile_TL = rmw_qos_profile_default;
    qos_profile_TL.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    m_state_transition_event_pub = m_nodehandle.create_publisher<spice_msgs::msg::RobotStateTransition>(
        m_work_cell_name + "/robot_state_transition_event",
        rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_TL.history, 10), qos_profile_TL)
    );

    m_tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_nodehandle);
    
    m_entry_transform.translation.x = -STEP_DISTANCE;
    m_exit_transform.translation.x = STEP_DISTANCE;
    double time = m_nodehandle.get_clock()->now().seconds();
    m_queue_manager = std::make_unique<QueueManager>(m_nodehandle, m_work_cell_name);
    m_queue_manager->initialize_points(3, time);

    m_current_state = WORK_CELL_STATE::STARTUP;
    m_states = {
        std::make_shared<StartupState>(*this),
        std::make_shared<ReadyForRobotState>(*this),
        std::make_shared<RobotEnteringState>(*this),
        std::make_shared<ProcessingState>(*this),
        std::make_shared<RobotExitingState>(*this)
    };
    m_states[static_cast<int>(m_current_state)]->init();
}

void WorkCellStateMachine::change_state(WORK_CELL_STATE new_state)
{
    if(new_state == m_current_state)
    {
        RCLCPP_WARN(m_nodehandle.get_logger(), "TRYING TO GO TO SAME STATE AS CURRENT STATE");
        return;
    }
    RCLCPP_INFO(m_nodehandle.get_logger(), "%s state transition to %d from %d", m_work_cell_name.c_str(),
            static_cast<int>(new_state), static_cast<int>(m_current_state));

    auto robot_state_transition_msg = std::make_unique<spice_msgs::msg::RobotStateTransition>();
    robot_state_transition_msg->old_state = internal_state_to_robot_state(m_current_state);
    robot_state_transition_msg->new_state = internal_state_to_robot_state(new_state);
    robot_state_transition_msg->id = get_work_cell_id();
    m_state_transition_event_pub->publish(std::move(robot_state_transition_msg));

    m_states[static_cast<int>(m_current_state)]->deinit();
    m_current_state = new_state;
    m_states[static_cast<int>(m_current_state)]->init();
}

tf2::Matrix3x3 q_to_mat(geometry_msgs::msg::Quaternion q)
{
    tf2::Quaternion q_tf2;
    q_tf2.setX(q.x); 
    q_tf2.setY(q.y); 
    q_tf2.setZ(q.z); 
    q_tf2.setW(q.w); 
    return tf2::Matrix3x3(q_tf2);
}

geometry_msgs::msg::Pose transform_to_map(geometry_msgs::msg::Transform& target_transform, geometry_msgs::msg::Transform& cell_transform)
{
    auto from_rot = q_to_mat(target_transform.rotation);
    auto to_rot = q_to_mat(cell_transform.rotation);

    tf2::Matrix3x3 from_to_rot_mat = from_rot * to_rot;
    tf2::Quaternion from_to_rot;
    from_to_rot_mat.getRotation(from_to_rot);

    tf2::Vector3 from_translation;
    from_translation.setX(target_transform.translation.x);
    from_translation.setY(target_transform.translation.y);
    from_translation.setZ(target_transform.translation.z);
    tf2::Vector3 to_translation;
    to_translation.setX(cell_transform.translation.x);
    to_translation.setY(cell_transform.translation.y);
    to_translation.setZ(cell_transform.translation.z);

    tf2::Vector3 from_to_trans = to_rot*from_translation + to_translation;

    geometry_msgs::msg::Pose map_pose;
    map_pose.position.x = from_to_trans.getX();
    map_pose.position.y = from_to_trans.getY();
    map_pose.position.z = from_to_trans.getZ();
    map_pose.orientation.x = from_to_rot.getX();
    map_pose.orientation.y = from_to_rot.getY();
    map_pose.orientation.z = from_to_rot.getZ();
    map_pose.orientation.w = from_to_rot.getW();
    return map_pose;
}

// bool WorkCellStateMachine::enqueue_robot(spice_msgs::srv::RegisterWork::Request::SharedPtr request)
// {
//     // TODO: add check that there is space in queue, if not, return false
//     // TODO: get queue pose here
//     carrier_robot robot(geometry_msgs::msg::Pose(), request->work, request->robot_id);
//     m_enqueued_robots.push_back(robot);
//     return true;
// }

void WorkCellStateMachine::on_register_work(
    const std::shared_ptr<spice_msgs::srv::RegisterWork::Request> request, 
    std::shared_ptr<spice_msgs::srv::RegisterWork::Response> response)
{
    RCLCPP_INFO(m_nodehandle.get_logger(), "On register work from %s", request->robot_id.id.c_str());
    // TODO: do we want to implement simulated checks for work compatibility, queue length etc?

    auto queue_point_opt = m_queue_manager->get_queue_point();
    if(!queue_point_opt)
    {
        RCLCPP_INFO(m_nodehandle.get_logger(), "Failed to register work due to no space in queue");
        response->work_is_enqueued = false;
        return;
    }

    QueuePoint* queue_point = queue_point_opt.value();
    queue_point->queued_robot = request->robot_id;
    carrier_robot robot(queue_point, request->work, request->robot_id, m_nodehandle.now());
    m_enqueued_robots.push_back(robot);
    response->work_is_enqueued = true;
    response->queue_id = queue_point->id;
    
    response->queue_pose.pose = transform_to_map(queue_point->transform, m_transform);
    response->queue_pose.header.frame_id = "map";
    response->queue_pose.header.stamp = m_nodehandle.get_clock()->now();

    response->entry_pose.pose = transform_to_map(m_entry_transform, m_transform);
    response->entry_pose.header.frame_id = "map";
    response->entry_pose.header.stamp = m_nodehandle.get_clock()->now();
    
    response->processing_pose.header.frame_id = "map";
    response->processing_pose.header.stamp = m_nodehandle.get_clock()->now();
    response->processing_pose.pose.position.x = m_transform.translation.x;
    response->processing_pose.pose.position.y = m_transform.translation.y;
    response->processing_pose.pose.position.z = m_transform.translation.z;
    response->processing_pose.pose.orientation.x = m_transform.rotation.x;
    response->processing_pose.pose.orientation.y = m_transform.rotation.y;
    response->processing_pose.pose.orientation.z = m_transform.rotation.z;
    response->processing_pose.pose.orientation.w = m_transform.rotation.w;
    
    response->exit_pose.pose = transform_to_map(m_exit_transform, m_transform);
    response->exit_pose.header.frame_id = "map";
    response->exit_pose.header.stamp = m_nodehandle.get_clock()->now();
    
    RCLCPP_INFO(m_nodehandle.get_logger(), "Enqueued robot: %s", request->robot_id.id.c_str());
}

void WorkCellStateMachine::on_robot_ready_in_queue(
    const std::shared_ptr<spice_msgs::srv::RobotReady::Request> request,
    std::shared_ptr<spice_msgs::srv::RobotReady::Response> response)
{
    for(auto& robot : m_enqueued_robots)
    {
        if(robot.robot_id.id == request->robot_id.id)
        {
            robot.ready_in_queue = true;
            response->success = true;
            return;
        }
    }
    RCLCPP_WARN(get_logger(), "On_robot_ready_in_queue got request for unknown robot");
    response->success = false; // return false if we do now know the robot
}

void WorkCellStateMachine::on_robot_ready_for_processing(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_states[static_cast<int>(m_current_state)]->on_robot_ready_for_processing(request, response);
}

void WorkCellStateMachine::on_robot_heartbeat(
    const std::shared_ptr<spice_msgs::srv::Heartbeat::Request> request,
    std::shared_ptr<spice_msgs::srv::Heartbeat::Response> response)
{
    // check for current robot in cell
    if(m_current_robot_work)
    {
        if(m_current_robot_work->robot_id.id == request->id.id)
        {
            response->restart_robot = false;
            m_current_robot_work->last_heartbeat_time = m_nodehandle.now();
            return;
        }
    }

    // check for enqueued robots
    for(auto& robot : m_enqueued_robots)
    {
        if(robot.robot_id.id == request->id.id)
        {
            response->restart_robot = false;
            robot.last_heartbeat_time = m_nodehandle.now();
            return;
        }
    }

    response->restart_robot = true; // if robot is unknown
}

void WorkCellStateMachine::check_robot_heartbeat_cb()
{
    rclcpp::Time current_time = m_nodehandle.now();

    auto it = m_enqueued_robots.begin();
    while(it != m_enqueued_robots.end())
    {
        auto timeout_period = (current_time-it->last_heartbeat_time).seconds();
        if(timeout_period > ROBOT_HEARTBEAT_TIMEOUT_PERIOD)
        {
            RCLCPP_WARN(get_logger(), "Heartbeat timeout on enqueued robot: %s", it->robot_id.id.c_str());
            auto to_delete = it;
            it++;
            m_queue_manager->free_queue_point(to_delete->queue_point);
            m_enqueued_robots.erase(to_delete);
        }
        else {
            it++; // to avoid pointer invalidation due to deletion above
        }
    }

    // check robot in cell (or in its way into the cell)
    // TODO: consider adding a service call between cell and carrier once exited
    // Here, we omit heartbeat when carrier robot is exiting, due to lack of this last synchronization
    if(m_current_robot_work && m_current_state != WORK_CELL_STATE::ROBOT_EXITING)
    {
        auto timeout_period = (current_time-m_current_robot_work->last_heartbeat_time).seconds();
        if(timeout_period > ROBOT_HEARTBEAT_TIMEOUT_PERIOD)
        {
            // timeout on robot in cell
            RCLCPP_WARN(get_logger(), "Heartbeat timeout on current robot in cell: %s", m_current_robot_work->robot_id.id.c_str());
            release_robot();
            if(m_current_state != WORK_CELL_STATE::READY_FOR_ROBOT && m_current_state != WORK_CELL_STATE::STARTUP)
            {
                change_state(WORK_CELL_STATE::READY_FOR_ROBOT);
            }
        }
    }
}

std::optional<carrier_robot> WorkCellStateMachine::get_enqueued_robot()
{
    for(auto enqueued_robot = m_enqueued_robots.begin(); enqueued_robot != m_enqueued_robots.end(); enqueued_robot++)
    {
        if(enqueued_robot->ready_in_queue)
        {
            std::optional<carrier_robot> next_robot(*enqueued_robot);
            m_enqueued_robots.erase(enqueued_robot);
            return next_robot;
        }
    }
    return {};
}

void WorkCellStateMachine::activate_heartbeat()
{
    //RCLCPP_INFO(this->m_nodehandle.get_logger(), "%s activating heartbeat", m_work_cell_name.c_str());
    if (!m_heartbeat_client)
    {
        m_heartbeat_client = m_nodehandle.create_client<spice_msgs::srv::Heartbeat>("heartbeat");
    }
    if (!m_heartbeat_timer)
    {
        m_heartbeat_timer = rclcpp::create_timer(
            &m_nodehandle,
            m_nodehandle.get_clock(),
            rclcpp::Duration::from_seconds(5),
            [this]() -> void {
                if (!this->m_heartbeat_client->wait_for_service(1s))
                {
                    RCLCPP_WARN(this->m_nodehandle.get_logger(), "Timeout on heartbeat service");
                }
                auto request = std::make_shared<spice_msgs::srv::Heartbeat::Request>();
                request->id = get_work_cell_id();
                this->m_heartbeat_client->async_send_request(request,
                [this](rclcpp::Client<spice_msgs::srv::Heartbeat>::SharedFuture future) -> void{
                    auto response = future.get();
                    if(response->restart_robot)
                    {
                        RCLCPP_WARN(this->m_nodehandle.get_logger(), 
                            "%s was asked to restart, but is is not implemented", 
                            this->m_work_cell_name.c_str());
                    }
                });
            }
        );
    }
    m_heartbeat_timer->reset();
}

void WorkCellStateMachine::deactivate_heartbeat()
{
    if(m_heartbeat_timer)
    {
        m_heartbeat_timer->cancel();
    }
}

void WorkCellStateMachine::publish_transform()
{
    geometry_msgs::msg::TransformStamped t;
    // publish center of cell
    t.header.stamp = m_nodehandle.get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = get_work_cell_id().id;
    t.transform = m_transform;
    m_tf_static_broadcaster->sendTransform(t);

    // publish transform for entry to cell
    t.transform = m_entry_transform;
    t.header.frame_id = get_work_cell_id().id;
    t.child_frame_id = get_work_cell_id().id + "_entry";
    m_tf_static_broadcaster->sendTransform(t);

    // publish transform for exit of cell
    t.transform = m_exit_transform;
    t.header.frame_id = get_work_cell_id().id;
    t.child_frame_id = get_work_cell_id().id + "_exit";
    m_tf_static_broadcaster->sendTransform(t);

    // publish queue positions
    auto queue_transforms = m_queue_manager->get_queue_point_transforms();
    for (size_t i = 0; i < queue_transforms.size(); i++)
    {   
        t.transform = queue_transforms[i];
        t.header.frame_id = get_work_cell_id().id;
        t.child_frame_id = get_work_cell_id().id + "_q" + std::to_string(i);
        m_tf_static_broadcaster->sendTransform(t);
    }
}

spice_msgs::msg::RobotState WorkCellStateMachine::internal_state_to_robot_state(WORK_CELL_STATE state)
{
    spice_msgs::msg::RobotState robot_state;
    if(state == WORK_CELL_STATE::STARTUP)
    {
        robot_state.state = spice_msgs::msg::RobotState::STARTUP;
    }
    else if (state == WORK_CELL_STATE::READY_FOR_ROBOT ||
        state == WORK_CELL_STATE::ROBOT_ENTERING ||
        state == WORK_CELL_STATE::PROCESSING ||
        state == WORK_CELL_STATE::ROBOT_EXITING)
    {
        robot_state.state = spice_msgs::msg::RobotState::WC_READY_FOR_ROBOTS;
    }
    int num_queue_points = m_queue_manager->m_queue_points.size();
    int used_points = 0;
    for(auto it = m_queue_manager->m_queue_points.begin(); it != m_queue_manager->m_queue_points.end(); it++)
    {
        if(it->occupied)
            used_points++;
    }
    std::string info = ", " + std::to_string(used_points) + "/" + std::to_string(num_queue_points);
    robot_state.internal_state = WORK_CELL_STATE_NAMES[static_cast<uint8_t>(state)] + info;
    return robot_state;
}

spice_msgs::msg::Id WorkCellStateMachine::get_work_cell_id() 
{ 
    spice_msgs::msg::Id id;
    id.id = m_work_cell_name;
    id.robot_type.type = m_robot_type;
    return id; 
}

void WorkCellStateMachine::release_robot()
{
    if(m_current_robot_work)
    {
        m_queue_manager->free_queue_point(m_current_robot_work->queue_point);
        m_current_robot_work.reset();
    }
    else
    {
        RCLCPP_WARN(m_nodehandle.get_logger(), "Tried to release robot, but did not have one");
    }
}

void WorkCellStateMachine::on_robot_exited(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_states[static_cast<int>(m_current_state)]->on_robot_exited(request, response);
}