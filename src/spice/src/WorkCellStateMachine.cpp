#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include "tf2/LinearMath/Matrix3x3.h"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice/work_cell_state_machine.hpp"

using namespace std::chrono_literals;

WorkCellStateMachine::WorkCellStateMachine(std::string work_cell_name, spice_msgs::msg::RobotType::_type_type robot_type, 
    rclcpp::Node& node_handle, geometry_msgs::msg::Transform transform)
:  m_nodehandle(node_handle), m_work_cell_name(work_cell_name), m_robot_type(robot_type), m_transform(transform)
{
    m_register_work_service = m_nodehandle.create_service<spice_msgs::srv::RegisterWork>(
        m_work_cell_name + "/register_work", 
        std::bind(&WorkCellStateMachine::on_register_robot, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_ready_for_processing_service = m_nodehandle.create_service<std_srvs::srv::Trigger>(
        m_work_cell_name + "/robot_ready_for_processing", 
        std::bind(&WorkCellStateMachine::on_robot_ready_for_processing, this, std::placeholders::_1, std::placeholders::_2));

    auto qos_profile_TL = rmw_qos_profile_default;
    qos_profile_TL.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    m_state_transition_event_pub = m_nodehandle.create_publisher<spice_msgs::msg::RobotStateTransition>(
        m_work_cell_name + "/robot_state_transition_event",
        rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_TL.history, 10), qos_profile_TL)
    );
    m_tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_nodehandle);

    const double STEP_DISTANCE = .5;
    
    m_entry_transform.translation.x = -STEP_DISTANCE;
    m_exit_transform.translation.x = STEP_DISTANCE;    

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

void WorkCellStateMachine::on_register_robot(
    const std::shared_ptr<spice_msgs::srv::RegisterWork::Request> request, 
    std::shared_ptr<spice_msgs::srv::RegisterWork::Response> response)
{
    RCLCPP_INFO(m_nodehandle.get_logger(), "On register robot");
    // TODO: do we want to implement simulated checks for work compatibility, queue length etc?
    m_enqueued_robots.emplace(*request);
    response->work_is_enqueued = true;
    
    // TODO: transform entry and exit poses to map frame before sending
    response->processing_pose.pose.position.x = m_transform.translation.x;
    response->processing_pose.pose.position.y = m_transform.translation.y;
    response->processing_pose.pose.position.z = m_transform.translation.z;
    response->processing_pose.pose.orientation.x = m_transform.rotation.x;
    response->processing_pose.pose.orientation.y = m_transform.rotation.y;
    response->processing_pose.pose.orientation.z = m_transform.rotation.z;
    response->processing_pose.pose.orientation.w = m_transform.rotation.w;
    
    // calculate exit transform to map frame, using m_transform
    auto from_rot = q_to_mat(m_exit_transform.rotation);
    auto to_rot = q_to_mat(m_transform.rotation);

    tf2::Matrix3x3 from_to_rot_mat = from_rot * to_rot;
    tf2::Quaternion from_to_rot;
    from_to_rot_mat.getRotation(from_to_rot);

    tf2::Vector3 from_translation;
    from_translation.setX(m_exit_transform.translation.x);
    from_translation.setY(m_exit_transform.translation.y);
    from_translation.setZ(m_exit_transform.translation.z);
    tf2::Vector3 to_translation;
    to_translation.setX(m_transform.translation.x);
    to_translation.setY(m_transform.translation.y);
    to_translation.setZ(m_transform.translation.z);

    tf2::Vector3 from_to_trans = to_rot*from_translation + to_translation;

    geometry_msgs::msg::Pose exit_pose;
    exit_pose.position.x = from_to_trans.getX();
    exit_pose.position.y = from_to_trans.getY();
    exit_pose.position.z = from_to_trans.getZ();
    exit_pose.orientation.x = from_to_rot.getX();
    exit_pose.orientation.y = from_to_rot.getY();
    exit_pose.orientation.z = from_to_rot.getZ();
    exit_pose.orientation.w = from_to_rot.getW();

    response->exit_pose.pose = exit_pose;

    response->processing_pose.header.frame_id = "map";
    response->processing_pose.header.stamp = m_nodehandle.get_clock()->now();
    response->exit_pose.header.frame_id = "map";
    response->exit_pose.header.stamp = m_nodehandle.get_clock()->now();
    

    RCLCPP_INFO(m_nodehandle.get_logger(), "Enqueued robot: %s", request->robot_id.id.c_str());
}

void WorkCellStateMachine::on_robot_ready_for_processing(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_states[static_cast<int>(m_current_state)]->on_robot_ready_for_processing(request, response);
}

std::optional<spice_msgs::srv::RegisterWork::Request> WorkCellStateMachine::get_enqueued_robot()
{
    if (m_enqueued_robots.empty()) return {};

    std::optional<spice_msgs::srv::RegisterWork::Request> next_robot(m_enqueued_robots.front());
    m_enqueued_robots.pop();
    return next_robot;
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
    robot_state.internal_state = WORK_CELL_STATE_NAMES[static_cast<uint8_t>(state)];
    return robot_state;
}

spice_msgs::msg::Id WorkCellStateMachine::get_work_cell_id() 
{ 
    spice_msgs::msg::Id id;
    id.id = m_work_cell_name;
    id.robot_type.type = m_robot_type;
    return id; 
}


// enum class WORK_CELL_STATE : uint8_t{
    // STARTUP = 0,
    // READY_FOR_ROBOT,
    // ROBOT_ENTERING,
    // PROCESSING,
    // ROBOT_EXITING,
    // NUM_STATES
// };