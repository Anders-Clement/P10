#ifndef DYNAMIC_OBSTACLE_LAYER_HPP
#define DYNAMIC_OBSTACLE_LAYER_HPP
#include "math.h"
#include "map"
#include "string"
#include "vector"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_costmap_2d
{

class DynamicObstacleLayer : public nav2_costmap_2d::Layer, public nav2_costmap_2d::Costmap2D
{
public:
  DynamicObstacleLayer();

  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable()
  {
    return false;
  }
  virtual void matchSize();

  virtual void TFCallback(tf2_msgs::msg::TFMessage::SharedPtr msg);

  virtual void get_robots_on_timer_cb();

  virtual void inflateDynObs(int loopsLeft, int maxLoops, std::vector<std::vector<unsigned int>> costpositions, double* min_x, double* min_y,
										double* max_x, double* max_y);

private:
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    float obstacle_points_;
    float update_;
    std::string topic_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    std::map<std::string, geometry_msgs::msg::TransformStamped> messageBuffer;
    rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_robots_cli;
    std::vector<spice_msgs::msg::Robot> robot_list; //list of all robots including workcell
    tf2::Duration transform_tolerance_;
    std::string robot_name;
    double ROBOT_RADIUS = 0.15;
    double ANGLE_INCREMENT;
    double TF_TOLERANCE = 10.0;
    double inflation_radius;
    unsigned char obstacle_cost;

    std::string global_frame_;
    
};
}  // namespace nav2_costmap_2d
#endif