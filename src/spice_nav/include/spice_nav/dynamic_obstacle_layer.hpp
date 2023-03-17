#ifndef DYNAMIC_OBSTACLE_LAYER_HPP
#define DYNAMIC_OBSTACLE_LAYER_HPP

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

  virtual void DynamicObstacleCallback(geometry_msgs::msg::PoseArray::SharedPtr msg);

private:
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    float update_;
    std::string topic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    std::map<std::string, geometry_msgs::msg::PoseArray> messageBuffer;
    rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}  // namespace nav2_costmap_2d
#endif