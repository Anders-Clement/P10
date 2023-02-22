#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <time.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spice_msgs/srv/get_ready_robots.hpp"
#include "spice_msgs/msg/robot.hpp"
#include "spice_msgs/srv/robot_task.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class TaskAllocator : public rclcpp::Node
{

public:
    TaskAllocator()
  : Node("task_allocator")
  {

    this->declare_parameter("use_rviz", false);
    rclcpp::Parameter use_rviz = this->get_parameter("use_rviz");

    if(!use_rviz.as_bool()){ //use deafualt Job points;
    PopulateLocations();
    }

    readyRobotsCli_ = this->create_client<spice_msgs::srv::GetReadyRobots>("get_ready_robots");

    jobSubscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/job_locations", 1, std::bind(&TaskAllocator::JobLocation_callback, this, _1));
    
    GetReadyRobot();

   // srand(time(NULL)); introduce RANDOMNESS WOWOWOWOWOWOWOWWOWO

    timerReadyBots_ = this->create_wall_timer(
      10s, std::bind(&TaskAllocator::GetReadyRobot, this));
  }

  
  void GetReadyRobot()
  {
    robotsRecieved = false;
    
    if (!readyRobotsCli_->wait_for_service(1s)) {
      
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for Swarm Manager service. Exiting.");
        return;
      }
      RCLCPP_WARN(this->get_logger(), "Swarm Manager service not available");
      
    }


    auto request = std::make_shared<spice_msgs::srv::GetReadyRobots::Request>();

    // We give the async_send_request() method a callback that will get executed once the response
    // is received.
    // This way we can return immediately from this method and allow other work to be done by the
    // executor in `spin` while waiting for the response.

    using ServiceResponseFuture =
      rclcpp::Client<spice_msgs::srv::GetReadyRobots>::SharedFuture;
    
    auto response_received_callback = [this](ServiceResponseFuture future) {
        
        auto result = future.get();
        robots_ = result->robots;
        robotsRecieved = true;
        AllocateTasks();
      };
    
    auto futureResult = readyRobotsCli_->async_send_request(request, response_received_callback);
  }




  void AllocateTasks(){

    if(!robotsRecieved){
      return;
    }
    else if (locations.size() <= 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run set_robot_task_poses and send poses from rviz");
        return;
    }
    

    for (const auto & robot : robots_) {

        allocTaskCli_ = this->create_client<spice_msgs::srv::RobotTask>(robot.id + "/allocate_task");
                
        if(!allocTaskCli_->wait_for_service(1s)) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "could not find %s task service ",robot.id.c_str());
        continue;
        }
        
        auto jobRequest = std::make_shared<spice_msgs::srv::RobotTask::Request>();
        
        jobRequest->goal_pose = locations[rand()%5];
        
        jobRequest->process_time = rand()%20; 
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Allocating task to %s",robot.id.c_str());
        
        using ServiceResponseFuture =
        rclcpp::Client<spice_msgs::srv::RobotTask>::SharedFuture;
    
        auto response_received_callback = [this](ServiceResponseFuture future) {
        
          auto result = future.get();
          
          std::string tookJob;
          if(result->job_accepted){
            tookJob = "took the job";
          }
          else{
            tookJob = "did not take the job";
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot %s ",tookJob);

        };
    
        auto futureResult = allocTaskCli_->async_send_request(jobRequest,response_received_callback);
     }
  } 

  

private:

  void PopulateLocations(){

        
        geometry_msgs::msg::Pose poses[5];
        
        poses[0].orientation.w = -2.5352063179016113;
        poses[0].orientation.x =0.0 ;
        poses[0].orientation.y = 0.0;
        poses[0].orientation.z = 0.9775979723076776;
        poses[0].position.x = -0.7799386978149414;
        poses[0].position.y = -3.2567038536071777;
        poses[0].position.z = 0.0;
        
        poses[1].orientation.w = 0.9765170182634035;
        poses[1].orientation.x = 0;
        poses[1].orientation.y =0 ;
        poses[1].orientation.z = -0.21544027720449957;
        poses[1].position.x = -6.155909538269043;
        poses[1].position.y = -0.9745041728019714;
        poses[1].position.z =0 ;

        poses[2].orientation.w = 0.38131979411579997;
        poses[2].orientation.x = 0;
        poses[2].orientation.y = 0;
        poses[2].orientation.z = -0.9244431916648441;
        poses[2].position.x = -2.2982521057128906;
        poses[2].position.y = 0.5658056139945984;
        poses[2].position.z = 0;

        poses[3].orientation.w = 0.9861226417049496;
        poses[3].orientation.x = 0;
        poses[3].orientation.y = 0;
        poses[3].orientation.z = 0.16601847944386092;
        poses[3].position.x = -14.981329917907715;
        poses[3].position.y = 3.0031487941741943;
        poses[3].position.z = 0;

        poses[4].orientation.w = 0.2830619845919701;
        poses[4].orientation.x = 0;
        poses[4].orientation.y = 0;
        poses[4].orientation.z = 0.9591016175978723;
        poses[4].position.x = -10.81999397277832;
        poses[4].position.y = 1.1732733249664307;
        poses[4].position.z = 0;

        geometry_msgs::msg::PoseStamped location;

        for(auto i: poses){
        
        location.header.stamp = this->get_clock()->now();
        location.header.frame_id = "map";
        location.pose = i;
        
        locations.push_back(location);
        }
  }



void JobLocation_callback(const geometry_msgs::msg::PoseArray msg){
      
      geometry_msgs::msg::PoseStamped location;
      
      for(auto i : msg.poses){

        location.header = msg.header;
        location.pose = i;
        locations.push_back(location);
      }
  }


  std::vector<geometry_msgs::msg::PoseStamped, std::allocator<geometry_msgs::msg::PoseStamped>> locations;
  rclcpp::Client<spice_msgs::srv::GetReadyRobots>::SharedPtr readyRobotsCli_;
  rclcpp::Client<spice_msgs::srv::RobotTask>::SharedPtr allocTaskCli_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr jobSubscription_;
  bool robotsRecieved = false;
  rclcpp::TimerBase::SharedPtr timerReadyBots_{nullptr};
  std::shared_ptr<spice_msgs::srv::GetReadyRobots_Request> request;
  std::vector<spice_msgs::msg::Id, std::allocator<spice_msgs::msg::Id>> robots_;
  
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskAllocator>());
  rclcpp::shutdown();
  return 0;
}