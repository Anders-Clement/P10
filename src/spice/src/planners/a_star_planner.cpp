// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// Based on the nav2_navfn_planner by Navigation 2


#include "spice/planners/a_star_planner.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <cmath>

AStarPlanner::AStarPlanner(CentralPathPlanner& central_path_planner) : GlobalPlanner(central_path_planner)
{
}

nav_msgs::msg::Path AStarPlanner::get_plan(
    geometry_msgs::msg::PoseStamped start, 
    geometry_msgs::msg::PoseStamped goal,
    double goal_tolerance,
    spice_msgs::msg::Id id)
{
    m_costmap = m_central_path_planner.get_costmap(id);
    if(!m_navfn_planner)
    {
        m_navfn_planner = std::make_unique<NavFn>(
            m_costmap->getSizeInCellsX(),
            m_costmap->getSizeInCellsY());
    }
        
    try
    {
        auto plan = create_plan(start, goal, goal_tolerance);
        return plan;
    }
    catch(const std::exception& e)
    {
        return nav_msgs::msg::Path();
    }
}

// derivative work of navfn_planner of navigation2
nav_msgs::msg::Path AStarPlanner::create_plan(
    geometry_msgs::msg::PoseStamped start, 
    geometry_msgs::msg::PoseStamped goal,
    double goal_tolerance)
{
    auto costmap = m_central_path_planner.get_costmap(spice_msgs::msg::Id());

    unsigned int mx_start, my_start, mx_goal, my_goal;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
        std::string error = {
                "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
                std::to_string(start.pose.position.y) + ") was outside bounds"};
        RCLCPP_WARN(m_central_path_planner.get_logger(), error.c_str());
        throw std::exception();
    }

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
        std::string error = {
                "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
                std::to_string(goal.pose.position.y) + ") was outside bounds"};
        RCLCPP_WARN(m_central_path_planner.get_logger(), error.c_str());
        throw std::exception();
    }

    if (costmap->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        std::string error =  {
                "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
                std::to_string(start.pose.position.y) + ") was in lethal cost"};
        RCLCPP_WARN(m_central_path_planner.get_logger(), error.c_str());
        throw std::exception();
    }

    if (goal_tolerance == 0 && costmap->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        std::string error = {
                "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
                std::to_string(goal.pose.position.y) + ") was in lethal cost"};
        RCLCPP_WARN(m_central_path_planner.get_logger(), error.c_str());
        throw std::exception();
    }

    // If costmap has changed, update planner based on the new costmap size
    if (m_navfn_planner->nx != static_cast<int>(costmap->getSizeInCellsX()) ||
        m_navfn_planner->ny != static_cast<int>(costmap->getSizeInCellsY()))
    {
        m_navfn_planner->setNavArr(
        costmap->getSizeInCellsX(),
        costmap->getSizeInCellsY());
    }

    nav_msgs::msg::Path path;

    // Corner case of the start(x,y) = goal(x,y)
    if (start.pose.position.x == goal.pose.position.x &&
        start.pose.position.y == goal.pose.position.y)
    {
        path.header.stamp = m_central_path_planner.now();
        path.header.frame_id = m_global_frame;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.z = 0.0;

        pose.pose = start.pose;
        // if we have a different start and goal orientation, set the unique path pose to the goal
        // orientation
        if (start.pose.orientation != goal.pose.orientation) {
        pose.pose.orientation = goal.pose.orientation;
        }
        path.poses.push_back(pose);
        return path;
    }

    if (!makePlan(start.pose, goal.pose, goal_tolerance, path)) {
        std::string error = {
                "Failed to create plan with tolerance of: " + std::to_string(goal_tolerance)};
        RCLCPP_WARN(m_central_path_planner.get_logger(), error.c_str());
        throw std::exception();
    }

  return path;
}

bool AStarPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = m_central_path_planner.now();
  plan.header.frame_id = m_global_frame;

  // TODO(orduno): add checks for start and goal reference frame -- should be in global frame

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    m_central_path_planner.get_logger(), "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(m_costmap->getMutex()));

  // make sure to resize the underlying array that Navfn uses
  m_navfn_planner->setNavArr(
    m_costmap->getSizeInCellsX(),
    m_costmap->getSizeInCellsY());

  m_navfn_planner->setCostmap(m_costmap->getCharMap(), true, true);

  lock.unlock();

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  worldToMap(wx, wy, mx, my);
  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  m_navfn_planner->setStart(map_goal);
  m_navfn_planner->setGoal(map_start);
  if (true) {
    m_navfn_planner->calcNavFnAstar();
  } else {
    m_navfn_planner->calcNavFnDijkstra(true);
  }

  double resolution = m_costmap->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  p = goal;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
  } else {
    // Goal is not reachable. Trying to find nearest to the goal
    // reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);

      // If use_final_approach_orientation=true, interpolate the last pose orientation from the
      // previous pose to set the orientation to the 'final approach' orientation of the robot so
      // it does not rotate.
      // And deal with corner case of plan of length 1
      if (false) {
        size_t plan_size = plan.poses.size();
        if (plan_size == 1) {
          plan.poses.back().pose.orientation = start.orientation;
        } else if (plan_size > 1) {
          double dx, dy, theta;
          auto last_pose = plan.poses.back().pose.position;
          auto approach_pose = plan.poses[plan_size - 2].pose.position;
          // Deal with the case of NavFn producing a path with two equal last poses
          if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
            std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
          {
            approach_pose = plan.poses[plan_size - 3].pose.position;
          }
          dx = last_pose.x - approach_pose.x;
          dy = last_pose.y - approach_pose.y;
          theta = atan2(dy, dx);
          plan.poses.back().pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(theta);
        }
      }
    } else {
      RCLCPP_ERROR(
        m_central_path_planner.get_logger(),
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

void AStarPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}

bool AStarPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  m_navfn_planner->setStart(map_goal);

  const int & max_cycles = (m_costmap->getSizeInCellsX() >= m_costmap->getSizeInCellsY()) ?
    (m_costmap->getSizeInCellsX() * 4) : (m_costmap->getSizeInCellsY() * 4);

  int path_len = m_navfn_planner->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = m_navfn_planner->getLastPathCost();
  RCLCPP_DEBUG(
    m_central_path_planner.get_logger(),
    "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = m_navfn_planner->getPathX();
  float * y = m_navfn_planner->getPathY();
  int len = m_navfn_planner->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double AStarPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * m_navfn_planner->nx + mx;
  return m_navfn_planner->potarr[index];
}

bool AStarPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < m_costmap->getOriginX() || wy < m_costmap->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - m_costmap->getOriginX()) / m_costmap->getResolution()));
  my = static_cast<int>(
    std::round((wy - m_costmap->getOriginY()) / m_costmap->getResolution()));

  if (mx < m_costmap->getSizeInCellsX() && my < m_costmap->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    m_central_path_planner.get_logger(),
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    m_costmap->getSizeInCellsX(), m_costmap->getSizeInCellsY());

  return false;
}

void AStarPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = m_costmap->getOriginX() + mx * m_costmap->getResolution();
  wy = m_costmap->getOriginY() + my * m_costmap->getResolution();
}

void AStarPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  m_costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}
