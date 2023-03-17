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
	declareParameter("topic", rclcpp::ParameterValue("dynamic_obstacle"));
	nh_->get_parameter(name_ + "." + "enabled", enabled_);
	nh_->get_parameter(name_ + "." + "topic", topic_);

	subscription_ = nh_->create_subscription<geometry_msgs::msg::PoseArray>(
		topic_, 10, std::bind(&DynamicObstacleLayer::DynamicObstacleCallback, this, _1));
	
	tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    
	tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	current_ = true;
	default_value_ = NO_INFORMATION;
	matchSize();
	RCLCPP_INFO(logger_, "[DYNAMIC OBSTACLE PLUGIN] initialized and subribed to topic: %s", topic_);
}

void DynamicObstacleLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
				master->getOriginY());
}

void DynamicObstacleLayer::DynamicObstacleCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
	geometry_msgs::msg::PoseArray poseArray = *msg;

	if (messageBuffer.find(msg->header.frame_id) == messageBuffer.end())
	{	 // frame id does not exist in map

		messageBuffer.insert({ msg->header.frame_id, poseArray });
	}
	else
	{	 // replace existing value;
		messageBuffer[msg->header.frame_id] = poseArray;
	}
	try{
	tf_buffer_->lookupTransform("polybot04_base_link","polybot07_base_link", tf2::TimePointZero);
	RCLCPP_INFO(logger_, "[DYNAMIC OBSTACLE PLUGIN]weei transformed stuff");
	}	
	catch(const tf2::TransformException & ex) {
          RCLCPP_INFO(logger_, "[DYNAMIC OBSTACLE PLUGIN]did not transformed stuff because: %s",ex.what());
          
        }

	
}	

void DynamicObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
										double* max_x, double* max_y)
	{
	double maxmx, minmx, minmy, maxmy;
	matchSize();

	for (auto const& obstaclePoints : messageBuffer)
	{
		if (nh_->now().seconds() - obstaclePoints.second.header.stamp.sec > 10.0)
		{  // check if msg time is within threshold
		continue;
		}

		for (auto pose : obstaclePoints.second.poses)
		{
		unsigned int mx, my;
		if (worldToMap(pose.position.x, pose.position.y, mx, my))
		{
			setCost(mx, my, LETHAL_OBSTACLE);
		}

		minmx = std::min(pose.position.x, minmx);
		minmy = std::min(pose.position.y, minmy);
		maxmx = std::max(pose.position.x, maxmx);
		maxmy = std::max(pose.position.y, maxmy);
		}
	}
	*min_x = std::min(*min_x, minmx);
	*min_y = std::min(*min_y, minmy);
	*max_x = std::max(*max_x, maxmx);
	*max_y = std::max(*max_y, maxmy);
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