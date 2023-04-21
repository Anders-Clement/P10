#ifndef WORK_CELL_STATE_MACHINE_HPP
#define WORK_CELL_STATE_MACHINE_HPP

#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include <queue>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/robot_state_transition.hpp"
#include "spice_msgs/srv/register_work.hpp"
#include "spice_msgs/srv/heartbeat.hpp"
#include "spice/work_cell.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "spice_msgs/srv/get_robots_by_type.hpp"

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
    spice_msgs::msg::Id get_work_cell_id();
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
    spice_msgs::msg::RobotState internal_state_to_robot_state(WORK_CELL_STATE state);

    void update_q_location();
    void update_robots_lists();
    void global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    int pnpoly(int nvert, std::vector<float> vertx, std::vector<float> verty, float testx, float testy);
    void inflateCostMap(int current_loop, std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
        std::vector<std::pair<unsigned int, unsigned int>> costpositions, float slope);
    void publish_costmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap);
    void preprocessing_q_costmap();

    std::string m_work_cell_name;
    spice_msgs::msg::RobotType::_type_type m_robot_type;
    geometry_msgs::msg::Transform m_transform;
    geometry_msgs::msg::Transform m_entry_transform;
    geometry_msgs::msg::Transform m_exit_transform;
    std::vector<geometry_msgs::msg::Transform> m_q_transforms;
    double ROBOT_RADIUS = 0.25;
    double WORKCELL_RADIUS = 0.25;
    int q_num = 3;
    bool gotCostmap = false;
    std::vector<std::pair<unsigned int, unsigned int>> viable_points;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmapPub;
    rclcpp::TimerBase::SharedPtr m_timer_robots_lists{nullptr};
    rclcpp::TimerBase::SharedPtr m_timer_q{nullptr};
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
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_robot_ready_for_processing_service;
    std::shared_ptr<rclcpp::Publisher<spice_msgs::msg::RobotStateTransition>> m_state_transition_event_pub;
    std::queue<spice_msgs::srv::RegisterWork::Request> m_enqueued_robots;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_global_costmap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmap_subscriber;
    
};

#endif