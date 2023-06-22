#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/qos.hpp"
#include "spice_msgs/srv/alloc_work_cell.hpp"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "spice_msgs/msg/robot.hpp"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/queue_occupancy.hpp"
#include "rclcpp/qos.hpp"

#define WORKCELL_QUEUE_SIZE 4

using namespace std::chrono_literals;

class WorckCellAllocator : public rclcpp::Node
{
public:
  WorckCellAllocator() : Node("work_cell_allocator")
  {

    tf2_ros::DynamicListenerQoS QoS;
     QoS.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    tf_buffer_ = 
     std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ =
     std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true, QoS);
    
    
    service = 
    create_service<spice_msgs::srv::AllocWorkCell>("allocate_work_cell", std::bind(&WorckCellAllocator::OnWorkCell, this,
    std::placeholders::_1, std::placeholders::_2));
    get_ready_robots_timer = 
    create_wall_timer(5s, std::bind(&WorckCellAllocator::get_robots_on_timer_cb, this));

    get_robots_cli = 
    create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");


    auto qos_profile_TL = rmw_qos_profile_default;
    qos_profile_TL.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    nrEnqueued_sub = this->create_subscription<spice_msgs::msg::QueueOccupancy>(
      "/workstations_occupancy",
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_TL.history, 10), qos_profile_TL),
      std::bind(&WorckCellAllocator::nrEnqueued_cb, this, std::placeholders::_1)
    );


  }

private:
  void get_robots_on_timer_cb()
  {
    auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
    get_robots_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

    using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

    auto get_workcells_cb = [this](ServiceResponseFuture future)
    {
      this->workcells = future.get()->robots;
    };

    auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_workcells_cb);
  }

  // made client to get all ready robots and merges so type is avaialble e.g. work cell
  // todo: be able to get a list of workcelltype.msg from robotstate and return what workcell the robot should go to.

  void nrEnqueued_cb(spice_msgs::msg::QueueOccupancy::SharedPtr msg)
  {
    occupancy_dict.insert_or_assign(msg->id.id, msg->enqueued);
    allocated_occupancy_dict.insert_or_assign(msg->id.id, 0);
  }

  void OnWorkCell(const std::shared_ptr<spice_msgs::srv::AllocWorkCell::Request> request,
                  const std::shared_ptr<spice_msgs::srv::AllocWorkCell::Response> response)
  {
    // RCLCPP_INFO(this->get_logger(), "Allocating task to robot: %s", request->robot_id.id.c_str());

    std::string toFrameRel = request.get()->robot_id.id + "_base_link";
    geometry_msgs::msg::TransformStamped t;
    float minDist = INFINITY;
    uint8_t minEnqueued = UINT8_MAX;
    // geometry_msgs::msg::TransformStamped goal;
    spice_msgs::msg::Id workcellType;

    for(auto workcell : workcells){
      if(occupancy_dict.at(workcell.id.id) > WORKCELL_QUEUE_SIZE)
        continue;
      for(auto type : request.get()->robot_types ){ // check if robot is of requested type
        if(type.type == workcell.id.robot_type.type){
          
          std::string fromFrameRel = workcell.id.id + "_entry";
          try
          {
            t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
          }
          catch (const tf2::TransformException &ex)
          {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            continue;
          }

          float dist = sqrt(std::pow(t.transform.translation.x, 2) + std::pow(t.transform.translation.y, 2) + std::pow(t.transform.translation.z, 2));
          uint8_t enqueued = occupancy_dict.at(workcell.id.id) + allocated_occupancy_dict.at(workcell.id.id);

          if (enqueued < minEnqueued )
          {
            workcellType = workcell.id;
            minEnqueued = enqueued;
            minDist = dist;
          }
          else if (enqueued == minEnqueued && dist < minDist)
          {
            workcellType = workcell.id;
            minEnqueued = enqueued;
            minDist = dist;
          }
        }  
        // RCLCPP_INFO(this->get_logger(), "WS [%s] has enqueue [%d]", workcell.id.id.c_str(), occupancy_dict.at(workcell.id.id)); 
      }
    }
    if (!workcellType.id.empty()){
      RCLCPP_INFO(this->get_logger(), "Allocating WS [%s] with queue [%d] and distance [%.2f] to robot [%s]", workcellType.id.c_str(), minEnqueued, round(minDist), request->robot_id.id.c_str()); 
      allocated_occupancy_dict.at(workcellType.id) += 1;
    }
    else{
      RCLCPP_WARN(this->get_logger(), "Could not find a viable WS to allocate to robot [%s]", request->robot_id.id.c_str()); 
    }


    if(workcellType == spice_msgs::msg::Id {}){
      response->found_job = false;
      return;
    }
    response->workcell_id = workcellType;
    response->found_job = true;
  }

  u_int8_t occupied_in_queue;
  std::shared_ptr<rclcpp::Subscription<spice_msgs::msg::QueueOccupancy>> nrEnqueued_sub;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<spice_msgs::srv::AllocWorkCell>::SharedPtr service;
  rclcpp::TimerBase::SharedPtr get_ready_robots_timer{nullptr};
  rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_robots_cli;
  std::vector<spice_msgs::msg::Robot> workcells; //list of all robots including workcells
  std::map<std::string,uint8_t> occupancy_dict;
  std::map<std::string,uint8_t> allocated_occupancy_dict;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorckCellAllocator>());
  rclcpp::shutdown();
  return 0;
}