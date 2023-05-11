/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "spice_control/controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/convert.h"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace nav2_pure_pursuit_controller
{

  /**
   * Find element in iterator with the minimum calculated value
   */
  template <typename Iter, typename Getter>
  Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end)
    {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it)
    {
      auto comp = getCompareVal(*it);
      if (comp < lowest)
      {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }

  void SpiceController::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".gamma_max", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.56));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".kp_omega", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_angular_vel", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".kp_linear_vel", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.1));

    get_params();

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  }

  void SpiceController::get_params()
  {
    auto node = node_.lock();
    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".gamma_max", gamma_max_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    node->get_parameter(plugin_name_ + ".kp_omega", kp_omega_);
    node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
    node->get_parameter(plugin_name_ + ".min_angular_vel", min_angular_vel_);
    node->get_parameter(plugin_name_ + ".kp_linear_vel", kp_linear_vel_);
    node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);
  }

  void SpiceController::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type pure_pursuit_controller::SpiceController",
        plugin_name_.c_str());
    global_pub_.reset();
  }

  void SpiceController::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type pure_pursuit_controller::SpiceController",
        plugin_name_.c_str());
    global_pub_->on_activate();
  }

  void SpiceController::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Dectivating controller: %s of type pure_pursuit_controller::SpiceController",
        plugin_name_.c_str());
    global_pub_->on_deactivate();
  }

  geometry_msgs::msg::TwistStamped SpiceController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &,
      nav2_core::GoalChecker *)
  {
    auto node = node_.lock();

    geometry_msgs::msg::PoseStamped last_plan_pose_in_map = tf_->transform(global_plan_.poses.back(), "map");
    auto robot_in_map = tf_->transform(pose, "map");
    auto goal = global_plan_.poses.back();
    goal.header.stamp = pose.header.stamp;
    auto goal_in_odom = tf_->transform(goal, "odom");
    auto goal_in_base_link = tf_->transform(goal, "base_link");
    // double robot_to_goal_in_odom_x = goal_in_odom.pose.position.x - pose.pose.position.x;
    // double robot_to_goal_in_odom_y = goal_in_odom.pose.position.y - pose.pose.position.y;

    // double gamma = std::atan2(robot_to_goal_in_odom_y, robot_to_goal_in_odom_x);
    // auto tf_quat = tf2::Quaternion(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    // tf2::Matrix3x3 m(tf_quat);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // double robot_rot = yaw;
    // double error = robot_rot - gamma;
    // RCLCPP_INFO(node->get_logger(), "goal_in_map x,y: %f,%f", last_plan_pose_in_map.pose.position.x, last_plan_pose_in_map.pose.position.y);
    // RCLCPP_INFO(node->get_logger(), "goal_in_base_link x,y: %f,%f", goal_in_base_link.pose.position.x, goal_in_base_link.pose.position.y);
    // RCLCPP_INFO(node->get_logger(), "goal_in_odom x,y: %f,%f", goal_in_odom.pose.position.x, goal_in_odom.pose.position.y);
    // RCLCPP_INFO(node->get_logger(), "pose x,y: %f,%f, frame: %s", pose.pose.position.x, pose.pose.position.y, pose.header.frame_id.c_str());
    // RCLCPP_INFO(node->get_logger(), "gamma: %f, robot_rot: %f, error: %f", gamma, robot_rot, error);

    geometry_msgs::msg::TwistStamped output;
    output.header.frame_id = pose.header.frame_id;
    output.header.stamp = node->now();

    double x_diff = robot_in_map.pose.position.x - last_plan_pose_in_map.pose.position.x;
    double y_diff = robot_in_map.pose.position.y - last_plan_pose_in_map.pose.position.y;
    double goal_dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);

    // at the goal, now face it
    if (goal_dist < goal_tolerance_)
    {
      tf2::Quaternion tf_quat;
      tf2::fromMsg(goal_in_odom.pose.orientation, tf_quat); //Assume quat_msg is a quaternion ros msg

      tf2::Matrix3x3 m(tf_quat);
      double _, __, goal_yaw;
      m.getRPY(_, __, goal_yaw);

      tf2::fromMsg(pose.pose.orientation, tf_quat); //Assume quat_msg is a quaternion ros msg
      m = tf2::Matrix3x3(tf_quat);
      double robot_yaw;
      m.getRPY(_, __, robot_yaw);
      // RCLCPP_INFO(node->get_logger(), "robot_yaw: %f, goal_yaw: %f", robot_yaw, goal_yaw);
      double omega;
      // δ=(T−C+540°)mod360°−180°
      double PI = 3.1415;
      robot_yaw += PI;
      goal_yaw += PI;

      double a = robot_yaw;
      double b = goal_yaw;
      // figure out which way to turn: https://math.stackexchange.com/questions/110080/shortest-way-to-achieve-target-angle
      if(a < b)
      {
        if(b-a <= PI)
        {
          // positive
          omega = std::clamp(b-a, min_angular_vel_, max_angular_vel_);
        }
        else
        {
          // negative
          omega = std::clamp(-(b-a), -max_angular_vel_, -min_angular_vel_);
        }
      }
      else
      {
        if(a-b <= PI)
        {
          // negative
          omega = std::clamp(-(a-b), -max_angular_vel_, -min_angular_vel_);
        }
        else
        {
          // positive
          omega = std::clamp(a-b, min_angular_vel_, max_angular_vel_);
        }
      }

      // double robot_to_goal_yaw = goal_yaw - robot_yaw;
      // if(std::abs(robot_to_goal_yaw) > 0.05)
      // {
      //   omega = max_angular_vel_;
      // }
      // else
      // {
      //   omega = 0.0;
      // }
      output.twist.linear.x = 0.0;
      output.twist.angular.z = omega;
      return output;
    }
    else // navigate towards the goal
    {
      double error = std::atan2(goal_in_base_link.pose.position.y, goal_in_base_link.pose.position.x);
      if (goal_in_base_link.pose.position.x > 0 && std::abs(error) < gamma_max_)
      {
        // proportional control
        double omega = kp_omega_ * error;
        double v = std::clamp(goal_dist * kp_linear_vel_, min_linear_vel_, desired_linear_vel_);
        // RCLCPP_INFO(node->get_logger(), "PROPORTIONAL CONTROL, e: %f, v: %f, w: %f", error, v, omega);

        output.twist.linear.x = v;
        output.twist.angular.z = omega;
        return output;
      }
      else
      {
        double omega;
        // turn on spot until goal is faced
        if (goal_in_base_link.pose.position.x > 0)
        {
          omega = error;
        }
        else
        {
          if (goal_in_base_link.pose.position.y > 0)
          {
            omega = max_angular_vel_;
          }
          else
          {
            omega = -max_angular_vel_;
          }
        }

        output.twist.linear.x = 0.0;
        output.twist.angular.z = omega;
        // RCLCPP_INFO(node->get_logger(), "ROTATION CONTROL, e: %f, v: %f, w: %f", error, 0.0, omega);
        return output;
      }
    }

    // auto transformed_plan = transformGlobalPlan(pose);

    // // Find the first pose which is at a distance greater than the specified lookahed distance
    // auto goal_pose_it = std::find_if(
    //     transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
    //     { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_; });

    // // If the last pose is still within lookahed distance, take the last pose
    // if (goal_pose_it == transformed_plan.poses.end())
    // {
    //   goal_pose_it = std::prev(transformed_plan.poses.end());
    // }
    // auto goal_pose = goal_pose_it->pose;

    // double linear_vel, angular_vel;

    // // If the goal pose is in front of the robot then compute the velocity using the pure pursuit
    // // algorithm, else rotate with the max angular velocity until the goal pose is in front of the
    // // robot
    // if (goal_pose.position.x > 0)
    // {
    //   auto curvature = 2.0 * goal_pose.position.y /
    //                    (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
    //   linear_vel = desired_linear_vel_;
    //   angular_vel = desired_linear_vel_ * curvature;
    // }
    // else
    // {
    //   linear_vel = 0.0;
    //   angular_vel = max_angular_vel_;
    // }

    // // Create and publish a TwistStamped message with the desired velocity
    // geometry_msgs::msg::TwistStamped cmd_vel;
    // cmd_vel.header.frame_id = pose.header.frame_id;
    // cmd_vel.header.stamp = clock_->now();
    // cmd_vel.twist.linear.x = linear_vel;
    // cmd_vel.twist.angular.z = max(
    //     -1.0 * abs(max_angular_vel_), min(
    //                                       angular_vel, abs(
    //                                                        max_angular_vel_)));

    // return cmd_vel;
  }

  void SpiceController::setPlan(const nav_msgs::msg::Path &path)
  {
    global_pub_->publish(path);
    global_plan_ = path;
    get_params();
  }

  nav_msgs::msg::Path
  SpiceController::transformGlobalPlan(
      const geometry_msgs::msg::PoseStamped &pose)
  {
    // Original mplementation taken fron nav2_dwb_controller

    if (global_plan_.poses.empty())
    {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
            tf_, global_plan_.header.frame_id, pose,
            robot_pose, transform_tolerance_))
    {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                            costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
            {
              return euclidean_distance(robot_pose, ps);
            });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto &global_plan_pose)
        {
          return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
    {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
          tf_, costmap_ros_->getBaseFrameID(),
          stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty())
    {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  bool SpiceController::transformPose(
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::string frame,
      const geometry_msgs::msg::PoseStamped &in_pose,
      geometry_msgs::msg::PoseStamped &out_pose,
      const rclcpp::Duration &transform_tolerance) const
  {
    // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    if (in_pose.header.frame_id == frame)
    {
      out_pose = in_pose;
      return true;
    }

    try
    {
      tf->transform(in_pose, out_pose, frame);
      return true;
    }
    catch (tf2::ExtrapolationException &ex)
    {
      auto transform = tf->lookupTransform(
          frame,
          in_pose.header.frame_id,
          tf2::TimePointZero);
      if (
          (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
          transform_tolerance)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Transform data too old when converting from %s to %s",
            in_pose.header.frame_id.c_str(),
            frame.c_str());
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Data time: %ds %uns, Transform time: %ds %uns",
            in_pose.header.stamp.sec,
            in_pose.header.stamp.nanosec,
            transform.header.stamp.sec,
            transform.header.stamp.nanosec);
        return false;
      }
      else
      {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Exception in transformPose: %s",
          ex.what());
      return false;
    }
    return false;
  }

  void SpiceController::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    if (!percentage)
    {
      desired_linear_vel_ = speed_limit;
      RCLCPP_INFO(
          logger_,
          "Received new speed_limit: %f", speed_limit);
    }
    else
    {
      RCLCPP_WARN(
          logger_,
          "Tried to set speed as percentage, but it is not supported");
    }
  }

} // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::SpiceController, nav2_core::Controller)