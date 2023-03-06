#ifndef WORK_CELL_STATE_MACHINE_HPP
#define WORK_CELL_STATE_MACHINE_HPP

#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include <queue>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "spice_msgs/msg/processing_type.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/srv/register_work.hpp"
#include "spice_msgs/srv/heartbeat.hpp"
#include "spice/work_cell.hpp"

enum class WORK_CELL_STATE : uint8_t{
    STARTUP = 0,
    READY_FOR_ROBOT,
    ROBOT_ENTERING,
    PROCESSING,
    ROBOT_EXITING,
    NUM_STATES
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
    std::optional<spice_msgs::srv::RegisterWork::Request> get_enqueued_robot();
    float get_processing_time() {return 5.0;} // TODO: add and use processing time in Work msg
    std::string get_work_cell_id() { return m_work_cell_name; }
    rclcpp::Logger get_logger() { return m_nodehandle.get_logger(); }

    rclcpp::Node& m_nodehandle;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_call_robot_client;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_done_processing_client;
    spice_msgs::srv::RegisterWork::Request m_current_robot_work;
private:

    void on_register_robot(
        const std::shared_ptr<spice_msgs::srv::RegisterWork::Request> request, 
        std::shared_ptr<spice_msgs::srv::RegisterWork::Response> response);
    void on_robot_ready_for_processing(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::string m_work_cell_name;
    spice_msgs::msg::RobotType::_type_type m_robot_type;
    geometry_msgs::msg::Transform m_transform;
    std::vector<std::shared_ptr<WorkCellState>> m_states;
    WORK_CELL_STATE m_current_state;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_static_broadcaster;
    std::shared_ptr<rclcpp::Client<spice_msgs::srv::Heartbeat>> m_heartbeat_client;
    rclcpp::TimerBase::SharedPtr m_heartbeat_timer;
    std::shared_ptr<rclcpp::Service<spice_msgs::srv::RegisterWork>> m_register_work_service;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_robot_ready_for_processing_service;
    std::queue<spice_msgs::srv::RegisterWork::Request> m_enqueued_robots;
};

#endif