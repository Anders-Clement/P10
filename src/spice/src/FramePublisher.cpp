#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("global_tf_relay")
  {
    //target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

    //tf_buffer_ =
      //std::make_unique<tf2_ros::Buffer>(this->get_clock());
    //tf_listener_ =
     // std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    prefix = getenv("ROBOT_NAMESPACE");
     
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      prefix+"/to_tf_global", 100, std::bind(&FramePublisher::tf_callback, this, _1));
    
    subscription_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      prefix+"/to_tf_static_global", 100, std::bind(&FramePublisher::tf_static_callback, this, _1));

    publisher_ =
      this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);

    publisher_static_ =
      this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", 100);

    
      
      //call on_timer at 10hz
      /*timer_ = this->create_wall_timer(
      100ms, std::bind(&FramePublisher::on_timer, this));*/
    
  }        

private:
  
  void tf_callback(const tf2_msgs::msg::TFMessage msg) const{
      
    tf2_msgs::msg::TFMessage output_msg;

    for (const auto & transform : msg.transforms) {
      // Append prefix to frame names
      geometry_msgs::msg::TransformStamped output_tf = transform;
      
      output_tf.header.frame_id = prefix+"/" + transform.header.frame_id;
      output_tf.child_frame_id = prefix+"/" + transform.child_frame_id;


      output_msg.transforms.push_back(output_tf);
    }
    publisher_->publish(output_msg);
  }

  void tf_static_callback(const tf2_msgs::msg::TFMessage msg) const{
    
    tf2_msgs::msg::TFMessage output_msg_static;

    for (const auto & transform : msg.transforms) {
      
      // Append prefix to frame names
      geometry_msgs::msg::TransformStamped output_tf = transform;
      
      output_tf.header.frame_id = prefix +"/" + transform.header.frame_id;
      output_tf.child_frame_id = prefix + "/" +transform.child_frame_id;

      output_msg_static.transforms.push_back(output_tf);
    }
    publisher_static_->publish(output_msg_static);
  }

  /*void on_timer(){
    std::string fromFrameRel = "base_link"; //change
    std::string toFrameRel = target_frame_.c_str();
    geometry_msgs::msg::TransformStamped t;
    try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
            
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
    
    publisher_->publish(t);
  }*/

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_static_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_{nullptr};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_static_{nullptr};
  std::string prefix;

  //std::string target_frame_;
  //std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  //std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  //rclcpp::TimerBase::SharedPtr timer_{nullptr};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}