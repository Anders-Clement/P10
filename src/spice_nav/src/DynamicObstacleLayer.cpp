#include <chrono>
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "spice_nav/dynamic_obstacle_layer.hpp"

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
  declareParameter("topic", rclcpp::ParameterValue("/tf"));
  declareParameter("obstacle_points", rclcpp::ParameterValue(8.0));
  nh_->get_parameter(name_ + "." + "enabled", enabled_);
  nh_->get_parameter(name_ + "." + "topic", topic_);
  nh_->get_parameter(name_ + "." + "obstacle_points", obstacle_points_);

  robot_name = getenv("ROBOT_NAMESPACE");

  subscription_ = nh_->create_subscription<tf2_msgs::msg::TFMessage>(
	  topic_, 10, std::bind(&DynamicObstacleLayer::TFCallback, this, _1));

  timer_ = nh_->create_wall_timer(5s, std::bind(&DynamicObstacleLayer::get_robots_on_timer_cb, this));
  get_robots_cli = nh_->create_client<spice_msgs::srv::GetRobotsByType>("/get_robots_by_type");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  ANGLE_INCREMENT = 2.0 * M_PI / obstacle_points_;
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  RCLCPP_INFO(logger_, "[DYNAMIC OBSTACLE PLUGIN] initialized");
}

void DynamicObstacleLayer::get_robots_on_timer_cb()
{
  if (!get_robots_cli->wait_for_service(1s))
  {
	RCLCPP_WARN(logger_, "Timeout on Swarm manager get_robots_by_type");
	return;
  }

  auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
  get_robots_request->type.type = spice_msgs::msg::RobotType::CARRIER_ROBOT;

  using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

  auto get_robots_cb = [this](ServiceResponseFuture future) { robot_list = future.get()->robots; };

  auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_robots_cb);
}

void DynamicObstacleLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
			master->getOriginY());
}

void DynamicObstacleLayer::TFCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (auto& tf : msg->transforms)
  {
	if (tf.header.frame_id != "map")
	{
	  continue;
	}
	messageBuffer[tf.child_frame_id] = tf;
  }
}

void DynamicObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
										double* max_x, double* max_y)
{
  double wx, wy;
  matchSize();
  geometry_msgs::msg::TransformStamped robot_tf;
  for (auto const& robot : robot_list)
  {
	if (robot.id.id == robot_name)
	{
	  continue;
	}

	unsigned int mx, my;
	try
	{
	  robot_tf = tf_buffer_->transform(messageBuffer[robot.id.id + "_base_link"], "odom");
	  wx = robot_tf.transform.translation.x;
	  wy = robot_tf.transform.translation.y;

	  if (worldToMap(wx, wy, mx, my))
	  {
		setCost(mx, my, LETHAL_OBSTACLE);
		*min_x = std::min(wx, *min_x);
		*min_y = std::min(wy, *min_y);
		*max_x = std::max(wx, *max_x);
		*max_y = std::max(wy, *max_y);
	  }

	  // put in additional points
	  for (int i = 0; i < obstacle_points_; i++)
	  {
		wx = robot_tf.transform.translation.x + (ROBOT_RADIUS * std::cos(ANGLE_INCREMENT * i));
		wy = robot_tf.transform.translation.y + (ROBOT_RADIUS * std::sin(ANGLE_INCREMENT * i));

		if (worldToMap(wx, wy, mx, my))
		{
		  setCost(mx, my, LETHAL_OBSTACLE);

		  *min_x = std::min(wx, *min_x);
		  *min_y = std::min(wy, *min_y);
		  *max_x = std::max(wx, *max_x);
		  *max_y = std::max(wy, *max_y);
		}
	  }
	}
	catch (const tf2::TransformException& ex)
	{
	  RCLCPP_INFO(logger_, "Could not transform to odom reason %s", ex);
	  return;
	}
  }
}

void DynamicObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
									   int max_j)
{
  if (!enabled_)
  {
	return;
  }

  for (int j = min_j; j < max_j; j++)
  {
	for (int i = min_i; i < max_i; i++)
	{
	  int index = getIndex(i, j);
	  if (costmap_[index] == NO_INFORMATION)
	  {
		continue;
	  }
	  master_grid.setCost(i, j, costmap_[index]);
	}
  }
}

void DynamicObstacleLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "onFootprintChanged(): num footprint points: %lu",
			   layered_costmap_->getFootprint().size());
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DynamicObstacleLayer, nav2_costmap_2d::Layer)