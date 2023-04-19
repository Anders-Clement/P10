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

// Based on the nav2_navfn_planner of Navigation 2


#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include "spice/planners/global_planner.hpp"
#include "spice/planners/navfn.hpp"

class AStarPlanner : public GlobalPlanner
{
public:
    AStarPlanner() = delete;
    AStarPlanner(CentralPathPlanner& central_path_planner);

    nav_msgs::msg::Path get_plan(
        geometry_msgs::msg::PoseStamped start, 
        geometry_msgs::msg::PoseStamped goal,
        double goal_tolerance,
        std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
        spice_msgs::msg::Id id) override;

private:
    nav_msgs::msg::Path create_plan(
        geometry_msgs::msg::PoseStamped start, 
        geometry_msgs::msg::PoseStamped goal,
        double goal_tolerance
    );
    bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav_msgs::msg::Path & plan);

    /**
     * @brief Compute the navigation function given a seed point in the world to start from
     * @param world_point Point in world coordinate frame
     * @return true if can compute
     */
    bool computePotential(const geometry_msgs::msg::Point & world_point);

    /**
     * @brief Compute a plan to a goal from a potential - must call computePotential first
     * @param goal Goal pose
     * @param plan Path to be computed
     * @return true if can compute a plan path
     */
    bool getPlanFromPotential(
        const geometry_msgs::msg::Pose & goal,
        nav_msgs::msg::Path & plan);

    /**
     * @brief Remove artifacts at the end of the path - originated from planning on a discretized world
     * @param goal Goal pose
     * @param plan Computed path
     */
    void smoothApproachToGoal(
        const geometry_msgs::msg::Pose & goal,
        nav_msgs::msg::Path & plan);

    /**
     * @brief Compute the potential, or navigation cost, at a given point in the world
     *        must call computePotential first
     * @param world_point Point in world coordinate frame
     * @return double point potential (navigation cost)
     */
    double getPointPotential(const geometry_msgs::msg::Point & world_point);

    // Check for a valid potential value at a given point in the world
    // - must call computePotential first
    // - currently unused
    // bool validPointPotential(const geometry_msgs::msg::Point & world_point);
    // bool validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

    /**
     * @brief Compute the squared distance between two points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return double squared distance between two points
     */
    inline double squared_distance(
        const geometry_msgs::msg::Pose & p1,
        const geometry_msgs::msg::Pose & p2)
    {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;
        return dx * dx + dy * dy;
    }

    /**
     * @brief Transform a point from world to map frame
     * @param wx double of world X coordinate
     * @param wy double of world Y coordinate
     * @param mx int of map X coordinate
     * @param my int of map Y coordinate
     * @return true if can transform
     */
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

    /**
     * @brief Transform a point from map to world frame
     * @param mx double of map X coordinate
     * @param my double of map Y coordinate
     * @param wx double of world X coordinate
     * @param wy double of world Y coordinate
     */
    void mapToWorld(double mx, double my, double & wx, double & wy);

    /**
     * @brief Set the corresponding cell cost to be free space
     * @param mx int of map X coordinate
     * @param my int of map Y coordinate
     */
    void clearRobotCell(unsigned int mx, unsigned int my);

    std::unique_ptr<NavFn> m_navfn_planner;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_costmap;
};

#endif // A_STAR_PLANNER_HPP