#ifndef WORK_CELL_QUEUE_POSITION_MANAGER_HPP
#define WORK_CELL_QUEUE_POSITION_MANAGER_HPP

#include <string>
#include "spice_msgs/msg/robot_type.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"
#include "spice_msgs/msg/robot_plan.hpp"
#include "spice_msgs/msg/param.hpp"

class WorkCellStateMachine;
class WorkCellQueuePositionManager{
    public:
    WorkCellQueuePositionManager() = delete;
    WorkCellQueuePositionManager(WorkCellStateMachine& workCellStateMachine);
    
    void global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg); // check
    void plans_cb(spice_msgs::msg::RobotPlan::SharedPtr msg);
    void updateRobotPlanCost();
    void timer_update_robots_lists();//check
    void timer_update_q_locations(); // check
    void update_workcell_costmap();// check
    void update_static_map_cost();


    int pnpoly(int nvert, std::vector<float> vertx, std::vector<float> verty, float testx, float testy); // check
    void inflateCostMap(int current_loop, std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope, std::vector<std::pair<unsigned int, unsigned int>> cost_points); // check
    void publish_costmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap); //check
    void param_cb(spice_msgs::msg::Param::SharedPtr msg);
    
    void attraction(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope, std::pair<unsigned int, unsigned int> wc_center_point); // check

    rclcpp::Logger get_logger();    

    private:


    WorkCellStateMachine& m_workCellStateMachine;

    rclcpp::TimerBase::SharedPtr m_timer_robots_lists{nullptr};
    rclcpp::TimerBase::SharedPtr m_timer_q{nullptr};
    rclcpp::TimerBase::SharedPtr m_timer_static_map{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmapPub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_workcells_cli;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_carriers_cli;
    //float WORK_CELL_REP_SLOPE, CARRIER_BOT_REP_SLOPE, WALL_REP_SLOPE, QUEUE_REP_SLOPE, PLAN_REP_SLOPE;
    //float WORK_CELL_ATT_SLOPE, QUEUE_ATT_SLOPE;
    //int MIN_MOVE_DIST = 1;  
    //float MAX_Q_VEL = 0.30;
    std::vector<std::pair<unsigned int, unsigned int>> static_map_cost_points;
    std::string MAP_NAME;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_global_costmap;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> workcell_costmap;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_robot_plan_costmap;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> static_costmap;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmap_subscriber;
    rclcpp::Subscription<spice_msgs::msg::RobotPlan>::SharedPtr m_plans_subscriber;
    rclcpp::Subscription<spice_msgs::msg::Param>::SharedPtr m_param_subcriber;
    
    std::map<std::string, nav_msgs::msg::Path> m_all_robot_plans;
    std::chrono::_V2::system_clock::time_point lastTime;
    std::pair<unsigned int, unsigned int> map_coord_entry;
    std::vector<std::pair<unsigned int, unsigned int>> viable_points;
    std::vector<std::pair<float,float>> world_corners;
    std::vector<float> world_corners_x;
    std::vector<float> world_corners_y;
    std::vector<spice_msgs::msg::Robot> workcell_list; //list of all workcells
    std::vector<spice_msgs::msg::Robot> carrier_list; //list of all carrier robots
    std::map<int,float> param_map;
};

#endif