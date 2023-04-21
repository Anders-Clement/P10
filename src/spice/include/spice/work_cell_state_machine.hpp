#ifndef WORK_CELL_STATE_MACHINE_HPP
#define WORK_CELL_STATE_MACHINE_HPP

#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include <list>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/msg/work.hpp"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/robot_state_transition.hpp"
#include "spice_msgs/srv/register_work.hpp"
#include "spice_msgs/srv/heartbeat.hpp"
#include "spice_msgs/srv/robot_ready.hpp"
#include "spice/work_cell.hpp"

enum class WORK_CELL_STATE : uint8_t{
    STARTUP = 0,
    READY_FOR_ROBOT,
    ROBOT_ENTERING,
    PROCESSING,
    ROBOT_EXITING,
    NUM_STATES
};

static const std::string WORK_CELL_STATE_NAMES[static_cast<uint8_t>(WORK_CELL_STATE::NUM_STATES)] = {
    "STARTUP",
    "READY_FOR_ROBOT",
    "ROBOT_ENTERING",
    "PROCESSING",
    "ROBOT_EXITING"
};

// all data for a specific client carrier robot
struct carrier_robot
{
    geometry_msgs::msg::Pose queue_pose;
    spice_msgs::msg::Work work;
    spice_msgs::msg::Id robot_id;
    bool ready_in_queue;

    carrier_robot(geometry_msgs::msg::Pose _queue_pose, 
            spice_msgs::msg::Work _work, spice_msgs::msg::Id _id)
            : queue_pose(_queue_pose),
            work(_work), robot_id(_id), ready_in_queue(false) {};
};

class WorkCellState;

class WorkCellStateMachine
{
public:
    WorkCellStateMachine(std::string work_cell_name, spice_msgs::msg::RobotType::_type_type robot_type, 
        rclcpp::Node& node_handle, geometry_msgs::msg::Transform transform);
    void change_state(WORK_CELL_STATE new_state);
    void activate_heartbeat();
    void deactivate_heartbeat();
    void publish_transform();
    bool enqueue_robot(spice_msgs::srv::RegisterWork::Request::SharedPtr request);
    std::optional<carrier_robot> get_enqueued_robot();
    float get_processing_time() {return 5.0;} // TODO: add and use processing time in Work msg
    spice_msgs::msg::Id get_work_cell_id();
    rclcpp::Logger get_logger() { return m_nodehandle.get_logger(); }

    rclcpp::Node& m_nodehandle;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_call_robot_client;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_done_processing_client;
    std::unique_ptr<carrier_robot> m_current_robot_work;
private:

    void on_register_work(
        const std::shared_ptr<spice_msgs::srv::RegisterWork::Request> request, 
        std::shared_ptr<spice_msgs::srv::RegisterWork::Response> response);
    void on_robot_ready_for_processing(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_robot_ready_in_queue(
        const std::shared_ptr<spice_msgs::srv::RobotReady::Request> request,
        std::shared_ptr<spice_msgs::srv::RobotReady::Response> response
    );
    spice_msgs::msg::RobotState internal_state_to_robot_state(WORK_CELL_STATE state);

    std::string m_work_cell_name;
    spice_msgs::msg::RobotType::_type_type m_robot_type;
    geometry_msgs::msg::Transform m_transform;
    geometry_msgs::msg::Transform m_entry_transform;
    geometry_msgs::msg::Transform m_exit_transform;
    std::vector<std::shared_ptr<WorkCellState>> m_states;
    WORK_CELL_STATE m_current_state;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_static_broadcaster;
    std::shared_ptr<rclcpp::Client<spice_msgs::srv::Heartbeat>> m_heartbeat_client;
    rclcpp::TimerBase::SharedPtr m_heartbeat_timer;
    std::shared_ptr<rclcpp::Service<spice_msgs::srv::RegisterWork>> m_register_work_service;
    std::shared_ptr<rclcpp::Service<spice_msgs::srv::RobotReady>> m_robot_ready_in_queue_service;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_robot_ready_for_processing_service;
    std::shared_ptr<rclcpp::Publisher<spice_msgs::msg::RobotStateTransition>> m_state_transition_event_pub;
    std::list<carrier_robot> m_enqueued_robots;
};

#endif