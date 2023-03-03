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
#include "spice_msgs/srv/alloc_work_cell.hpp"
#include "spice_msgs/srv/get_robots.hpp"
#include "spice_msgs/msg/robot.hpp"

using namespace std::chrono_literals;

class WorckCellAllocator : public rclcpp::Node
{
public:
  WorckCellAllocator() : Node("work_cell_allocator")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    service = create_service<spice_msgs::srv::AllocWorkCell>("allocate_work_cell", std::bind(&WorckCellAllocator::OnWorkCell, this,
                                                                                             std::placeholders::_1, std::placeholders::_2));
    get_ready_robots_timer = create_wall_timer(5s, std::bind(&WorckCellAllocator::get_robots_on_timer_cb, this));

    get_robots_cli = create_client<spice_msgs::srv::GetRobots>("get_robots");
  }

private:
  void get_robots_on_timer_cb()
  {
    auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobots::Request>();

    using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobots>::SharedFuture;

    auto get_robots_cb = [this](ServiceResponseFuture future)
    {
      this->robots = future.get()->robots;
    };

    auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_robots_cb);
  }

  // made client to get all ready robots and merges so type is avaialble e.g. work cell
  // todo: be able to get a list of workcelltype.msg from robotstate and return what workcell the robot should go to.

  void OnWorkCell(const std::shared_ptr<spice_msgs::srv::AllocWorkCell::Request> request,
                  const std::shared_ptr<spice_msgs::srv::AllocWorkCell::Response> response)
  {

    std::string fromFrameRel = request.get()->robot_id.id + "_base_link";
    geometry_msgs::msg::TransformStamped t;
    float minDist = INFINITY;
    geometry_msgs::msg::TransformStamped goal;

    // for (auto toFrameRel : request.get()->work_cell_types)
    // {
    //   try
    //   {
    //     t = tf_buffer_->lookupTransform(
    //         toFrameRel, fromFrameRel,
    //         tf2::TimePointZero);
    //     float dist = sqrt(std::pow(t.transform.translation.x, 2) + std::pow(t.transform.translation.y, 2) + std::pow(t.transform.translation.z, 2));

    //     if (dist < minDist)
    //     {
    //       minDist = dist;
    //       goal = tf_buffer_->lookupTransform("map", toFrameRel, tf2::TimePointZero);
    //     }
    //   }
    //   catch (const tf2::TransformException &ex)
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    //     continue;
    //   }
    // }

    if (goal.header.frame_id == "")
    {
      response->found_job = false;
      return;
    }
    // SEND RESPONSE
    geometry_msgs::msg::PoseStamped goalPose;
    geometry_msgs::msg::Pose pose;

    goalPose.header = goal.header;
    goalPose.pose.position.x = goal.transform.translation.x;
    goalPose.pose.position.y = goal.transform.translation.y;
    goalPose.pose.position.z = goal.transform.translation.z;
    goalPose.pose.orientation = goal.transform.rotation;
    response->goal_pose = goalPose;
    response->found_job = true;
  }
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<spice_msgs::srv::AllocWorkCell>::SharedPtr service;
  rclcpp::TimerBase::SharedPtr get_ready_robots_timer{nullptr};
  rclcpp::Client<spice_msgs::srv::GetRobots>::SharedPtr get_robots_cli;
  std::vector<spice_msgs::msg::Robot> robots;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorckCellAllocator>());
  rclcpp::shutdown();
  return 0;
}