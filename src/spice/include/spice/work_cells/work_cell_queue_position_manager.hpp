#ifndef WORK_CELL_QUEUE_POSITION_MANAGER_HPP
#define WORK_CELL_QUEUE_POSITION_MANAGER_HPP

#include <string>
#include "spice_msgs/msg/robot_type.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"

class WorkCellStateMachine;
class WorkCellQueuePositionManager{
    public:
    WorkCellQueuePositionManager() = delete;
    WorkCellQueuePositionManager(WorkCellStateMachine& workCellStateMachine);
    
    void global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg); // check
    void timer_update_robots_lists();//check
    void timer_update_q_locations(); // check
    void update_workcell_costmap();// check


    int pnpoly(int nvert, std::vector<float> vertx, std::vector<float> verty, float testx, float testy); // check
    void inflateCostMap(int current_loop, std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope); // check
    void publish_costmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap); //check
    
    void attraction(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope, std::pair<unsigned int, unsigned int> wc_center_point); // check

    rclcpp::Logger get_logger();    

    private:
    
    WorkCellStateMachine& m_workCellStateMachine;

    rclcpp::TimerBase::SharedPtr m_timer_robots_lists{nullptr};
    rclcpp::TimerBase::SharedPtr m_timer_q{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmapPub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_workcells_cli;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_carriers_cli;


    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_global_costmap;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> workcell_costmap;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmap_subscriber;
    std::vector<std::pair<unsigned int, unsigned int>> costpoints;
    std::mutex m_mutex;
    float MAX_Q_VEL = 0.30;
    std::chrono::_V2::system_clock::time_point lastTime;
    std::pair<unsigned int, unsigned int> map_coord_entry;
    std::vector<std::pair<unsigned int, unsigned int>> viable_points;
    std::vector<std::pair<float,float>> world_corners;
    std::vector<float> world_corners_x;
    std::vector<float> world_corners_y;
    std::vector<spice_msgs::msg::Robot> workcell_list; //list of all workcells
    std::vector<spice_msgs::msg::Robot> carrier_list; //list of all carrier robots
};

#endif