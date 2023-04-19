/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "spice_nav/central_planner_plugin.hpp"

namespace spice_nav
{
    using namespace std::chrono_literals;

void CentralPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
        0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".tolerance", rclcpp::ParameterValue(
        0.1));
    node_->get_parameter(name_ + ".tolerance", goal_tolerance_);

    robot_namespace_ = std::string(node_->get_namespace()).substr(std::string(node_->get_namespace()).rfind('/')+1);
    // default to prioritized planner
    planner_type.type = spice_msgs::msg::PlannerType::PLANNER_PRIORITIZED;

    if(robot_namespace_ == "")
    {
      RCLCPP_ERROR(node_->get_logger(), 
        "ROBOT_NAMESPACE is empty, planner will be unable to fetch correct plans");
    }

    central_planner_client = node_->create_client<spice_msgs::srv::GetPlan>("/get_plan");

    set_planner_type_server = node_->create_service<spice_msgs::srv::SetPlannerType>("set_planner_type", 
        std::bind(&CentralPlanner::set_planner_type_cb, this, std::placeholders::_1, std::placeholders::_2));

    wait_publisher = node_->create_publisher<std_msgs::msg::Bool>("wait_topic", 10);
}

void CentralPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
    
}

void CentralPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void CentralPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path CentralPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
    if(!central_planner_client->wait_for_service(1s))
    {
        RCLCPP_ERROR(node_->get_logger(), "Timeout on wait for central planner service");
        return nav_msgs::msg::Path();
    }

    spice_msgs::srv::GetPlan::Request::SharedPtr request = std::make_shared<spice_msgs::srv::GetPlan::Request>();
    request->start = start;
    request->goal = goal;
    request->tolerance = goal_tolerance_;
    request->id.id = robot_namespace_;
    request->planner_type = planner_type;

    auto start_time = node_->now();
    
    auto future = central_planner_client->async_send_request(request);
    auto status = future.wait_for(1s);
    if(status == std::future_status::timeout)
    {
      RCLCPP_ERROR(node_->get_logger(), "Central planner took too long! Aborting planning...");
      return nav_msgs::msg::Path();
    }

    spice_msgs::srv::GetPlan::Response::SharedPtr response = future.get(); 
         
    auto duration = (node_->now()-start_time).nanoseconds()*10e-9;
    RCLCPP_INFO(node_->get_logger(), 
      "Got path from central planner with %ld poses in %f seconds", 
      response->plan.poses.size(), 
      duration);

    // always publish wait condition, in order to keep in sync
    std_msgs::msg::Bool msg;
    msg.data = response->wait;
    wait_publisher->publish(msg);

    return response->plan;
}

void CentralPlanner::set_planner_type_cb(spice_msgs::srv::SetPlannerType::Request::SharedPtr req, 
    spice_msgs::srv::SetPlannerType::Response::SharedPtr res)
{
    planner_type = req->planner_type;
    res->success = true;
}

}  // namespace spice_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spice_nav::CentralPlanner, nav2_core::GlobalPlanner)