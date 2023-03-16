#include <chrono>
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "spice/dynamic_obstacle_layer.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace nav2_costmap_2d
{

DynamicObstacleLayer::DynamicObstacleLayer()
  : last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
}

void DynamicObstacleLayer::onInitialize()
{
  nh_ = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue("dynamic_obstacle"));
  nh_->get_parameter(name_ + "." + "enabled", enabled_);
  nh_->get_parameter(name_ + "." + "topic", topic_);

  subscription_ = nh_->create_subscription<geometry_msgs::msg::PoseArray>(
      topic_, 10, std::bind(&DynamicObstacleLayer::DynamicObstacleCallback, this, _1));

  need_recalculation_ = false;
  current_ = true;
  RCLCPP_INFO(logger_, "[DYNAMIC OBSTACLE PLUGIN] initialized and subribed to topic: %s", topic_);
}

void DynamicObstacleLayer::DynamicObstacleCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  messageBuffer[msg->header.frame_id] = *msg;
}

void DynamicObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{
  if (need_recalculation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void DynamicObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                       int max_j)
{
  if (!enabled_)
  {
    return;
  }

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(),
               size_y = master_grid.getSizeInCellsY();  // size of map if needed later on
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  //
  unsigned int span = master_grid.getSizeInCellsX();

  for (auto obstaclePoints : messageBuffer)
  {  // for msg in buffer update cost map
    RCLCPP_INFO(logger_, "obstacle points proccesed with frame id: %s",obstaclePoints.second.header.frame_id);
    
    if (nh_->now().seconds() - obstaclePoints.second.header.stamp.sec > 10.0)
    {  // check if msg time is within threshold
      continue;
    }

    for (auto pose : obstaclePoints.second.poses)
    {  // get pose index in master_grid
      RCLCPP_INFO(logger_, "obstacle point at world position: x: %f y:%f z:%f",pose.position.x, pose.position.y, pose.position.z);
      unsigned int mx, my;
      if (master_grid.worldToMap(pose.position.x, pose.position.y, mx, my))
      {
        RCLCPP_INFO(logger_, "obstacle was succesfully transformed to costmap");
        int index = master_grid.getIndex(mx, my);
        master_array[index] = LETHAL_OBSTACLE;
      }
      else RCLCPP_INFO(logger_, "obstacle failed to transform to costmap");
    }
  }
}

void DynamicObstacleLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
               layered_costmap_->getFootprint().size());
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DynamicObstacleLayer, nav2_costmap_2d::Layer)