#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class PoseRelay : public rclcpp::Node
{
public:
  PoseRelay()
  : Node("robot_pose_relayer")
  {
    prefix_ = this->declare_parameter<std::string>("prefix", getenv("ROBOT_NAMESPACE"));

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<tf2_msgs::msg::TFMessage>("to_tf_global", 100);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PoseRelay::on_timer, this));
  }

private:
  void on_timer()
  {
    std::string toFrameRel = "map";
    std::string fromFrameRel = "base_link";
    tf2_msgs::msg::TFMessage output_msg;
    geometry_msgs::msg::TransformStamped t;
    
    try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
            output_msg.transforms.push_back(t);

            publisher_->publish(output_msg);

        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
  }


  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string prefix_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseRelay>());
  rclcpp::shutdown();
  return 0;
}