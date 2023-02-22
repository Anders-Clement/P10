#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SetRobotTaskPoses : public rclcpp::Node
{

public:
    SetRobotTaskPoses()
  : Node("set_robot_task_poses")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", 100, std::bind(&SetRobotTaskPoses::PointSet_callback, this, _1));

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker",10);

    jobLocationPublisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/job_locations",1);
  }

  
private:

    void PointSet_callback(const geometry_msgs::msg::PointStamped msg) {
        

        visualization_msgs::msg::Marker marker;

        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";
        
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = msg.point;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1;
        marker.color.b = 0;
        marker.color.g = 1;
        marker.color.r = 0;

        markers.markers.push_back(marker);
        id ++;
       
        publisher_->publish(markers);
        PublishRvizLocations(markers);
 
    }

    void PublishRvizLocations(visualization_msgs::msg::MarkerArray locations){
        
        geometry_msgs::msg::PoseArray jobLocations;
        geometry_msgs::msg::Pose jobLocation;

        for(auto location : locations.markers){
            
            jobLocation = location.pose;

            jobLocations.poses.push_back(jobLocation); 
        }

        jobLocationPublisher_->publish(jobLocations);

    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr jobLocationPublisher_{nullptr};
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetRobotTaskPoses>());
  rclcpp::shutdown();
  return 0;
}