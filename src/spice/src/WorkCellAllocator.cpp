#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "spice_msgs/srv/alloc_task.hpp"

using namespace std::chrono_literals;

class WorckCellAllocator : public rclcpp::Node
{
public:
  WorckCellAllocator()
  : Node("robot_pose_relayer")
  {
        tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        service = 
        create_service<spice_msgs::srv::AllocTask>("allocate task", std::bind(&WorckCellAllocator::OnAllocTask, this,
                                std::placeholders::_1, std::placeholders::_2));

  }

private:

  void OnAllocTask(const std::shared_ptr<spice_msgs::srv::AllocTask::Request>request,
          std::shared_ptr<spice_msgs::srv::AllocTask::Response>response){
    
    std::string fromFrameRel = request.get()->id + "_base_link";
    
    //std::string fromFrameRel =request.get()->goal_pose;
    geometry_msgs::msg::TransformStamped t;
    float minDist = INFINITY;
    geometry_msgs::msg::TransformStamped goal; 

    for(auto toFrameRel : request.get()->process_type){
        
        try{      
            t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
            float dist = sqrt(std::pow(t.transform.translation.x,2) + std::pow(t.transform.translation.y,2) + std::pow(t.transform.translation.z,2));
            
            if(dist < minDist){
                minDist = dist;
                goal = tf_buffer_->lookupTransform("map",toFrameRel,tf2::TimePointZero);
            }
        }
         catch (const tf2::TransformException & ex) {
          /*RCLCPP_INFO(
            //this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());*/
          continue;
        }
    }

    //SEND RESPONSE
    geometry_msgs::msg::PoseStamped goalPose;
    goalPose.header = goal.header;
    goalPose.pose.position.x = goal.transform.translation.x;
    goalPose.pose.position.x = goal.transform.translation.y;
    goalPose.pose.position.x = goal.transform.translation.z;
    goalPose.pose.orientation = goal.transform.rotation; 
    response->goal_pose;
   
  }
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<spice_msgs::srv::AllocTask>::SharedPtr service;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorckCellAllocator>());
  rclcpp::shutdown();
  return 0;
}