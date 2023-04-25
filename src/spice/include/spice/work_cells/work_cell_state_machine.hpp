#ifndef WORK_CELL_STATE_MACHINE_HPP
#define WORK_CELL_STATE_MACHINE_HPP

#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include <list>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "spice_msgs/msg/task.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/msg/work.hpp"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/robot_state_transition.hpp"
#include "spice_msgs/srv/register_work.hpp"
#include "spice_msgs/srv/heartbeat.hpp"
#include "spice_msgs/srv/robot_ready.hpp"
#include "spice/work_cells/queue_manager.hpp"
#include "spice/work_cells/work_cell.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "spice/work_cells/work_cell_queue_position_manager.hpp"

#define ROBOT_RADIUS 0.25
#define WORKCELL_RADIUS 0.25
#define STEP_DISTANCE 0.5

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
    QueuePoint* queue_point;
    spice_msgs::msg::Work work;
    spice_msgs::msg::Id robot_id;
    bool ready_in_queue;

    carrier_robot(QueuePoint* _queue_point, 
            spice_msgs::msg::Work _work, spice_msgs::msg::Id _id)
            : queue_point(_queue_point),
            work(_work), robot_id(_id), ready_in_queue(false) {};
};

class WorkCellState;
class QueueManager;
class WorkCellQueuePositionManager;

class WorkCellStateMachine
{
public:
    WorkCellStateMachine(std::string work_cell_name, spice_msgs::msg::RobotType::_type_type robot_type, 
        rclcpp::Node& node_handle, geometry_msgs::msg::Transform transform);
    void change_state(WORK_CELL_STATE new_state);
    void activate_heartbeat();
    void deactivate_heartbeat();
    void publish_transform();
    std::optional<carrier_robot> get_enqueued_robot();
    void release_robot();
    float get_processing_time() {return 5.0;} // TODO: add and use processing time in Work msg
    spice_msgs::msg::Id get_work_cell_id();
    rclcpp::Logger get_logger() { return m_nodehandle.get_logger(); }

    rclcpp::Node& m_nodehandle;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_call_robot_client;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_done_processing_client;
    std::unique_ptr<carrier_robot> m_current_robot_work;
    std::string m_work_cell_name;
    QueueManager m_queue_manager;

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
    std::unique_ptr<WorkCellQueuePositionManager> m_work_cell_queue_manager;


    spice_msgs::msg::RobotType::_type_type m_robot_type;
    geometry_msgs::msg::Transform m_transform;
    geometry_msgs::msg::Transform m_entry_transform;
    geometry_msgs::msg::Transform m_exit_transform;

    std::vector<spice_msgs::msg::Robot> workcell_list; //list of all workcells
    std::vector<spice_msgs::msg::Robot> carrier_list; //list of all carrier robots
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_workcells_cli;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_carriers_cli;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::vector<std::pair<float,float>> world_corners;
    std::vector<float> world_corners_x;
    std::vector<float> world_corners_y;
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
    rclcpp::TimerBase::SharedPtr m_timer{nullptr};
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_global_costmap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmap_subscriber;
    
};

#endif