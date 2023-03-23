#include <vector>
#include <algorithm>
#include "spice/costmaps/prioritized_costmap.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

PrioritizedCostmap::PrioritizedCostmap(CentralPathPlanner& central_path_planner) : Costmap(central_path_planner)
{
  get_robots_cli = m_central_path_planner.create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");
  get_ready_robots_timer =
      m_central_path_planner.create_wall_timer(1s, std::bind(&PrioritizedCostmap::get_robots_on_timer_cb, this));
};

// get current full costmap, of map + any other layers added for a robot Id
std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::get_costmap(spice_msgs::msg::Id id)
{
  auto it = prevRobotCostMaps.find(id);

  if (it != prevRobotCostMaps.end())

    return prevRobotCostMaps[id];
  else
  {
    RCLCPP_WARN(m_central_path_planner.get_logger(), "Asked to get costmap, but do not have a costmap yet");
    return std::make_shared<nav2_costmap_2d::Costmap2D>();
  }
}

void PrioritizedCostmap::get_robots_on_timer_cb()
{
  auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
  get_robots_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

  using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

  auto get_robots_cb = [this](ServiceResponseFuture future) { robots = future.get()->robots; };

  auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_robots_cb);
}

void PrioritizedCostmap::calcPrioritizedCostMap(spice_msgs::msg::Id robotId)
{
  // update avialable robots in map
  std::vector<spice_msgs::msg::Robot> robotsCopy = robots;

  for (std::map<spice_msgs::msg::Id, std::shared_ptr<nav2_costmap_2d::Costmap2D>>::iterator i = robotCostMaps.begin();
       i != robotCostMaps.end(); i++)
  {
    auto it = std::find(robotsCopy.begin(), robotsCopy.end(), i->first);
    if (it != robotsCopy.end())
    {
      robotsCopy.erase(it);
    }
    else
    {
      robotCostMaps.erase(i);
    }
  }
  for (auto i : robotsCopy)
  {
    prevRobotCostMaps[i.id] = nullptr;
  }

    //create blank costmap of same dimensions as global_costmap
  nav2_costmap_2d::Costmap2D costmap;
  costmap.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  costmap.resizeMap(m_global_costmap->getSizeInCellsX(), m_global_costmap->getSizeInCellsY(),
                    m_global_costmap->getResolution(), m_global_costmap->getOriginX(), m_global_costmap->getOriginY());


  for (auto it : robotCostMaps)  // robotCostMap ordered for priority
  {
    if(it.first == robotId){
        prevRobotCostMaps = robotCostMaps;
        return;
    }
    
    nav2_costmap_2d::Costmap2D prioritized_cost_map;


    //TODO: combine costmap and m_global_costmap

    *robotCostMaps[it.first] = costmap;

    nav_msgs::msg::Path robotPath;  // m_central_path_planner.get_robot_plan(it.first);
    std::vector<std::vector<unsigned int>> costpositions;

    for (auto pose : robotPath.poses)
    {
      unsigned int mx, my;
      if (costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
      {
        if (costmap.getCost(mx, my) <= nav2_costmap_2d::LETHAL_OBSTACLE)
          costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        costpositions.push_back({ mx, my });
      }
    }
    inflateCostMap(5, 5, costmap, costpositions);
  }
}

void PrioritizedCostmap::inflateCostMap(int loopsLeft, int maxLoops, nav2_costmap_2d::Costmap2D& costmap,
                                        std::vector<std::vector<unsigned int>> costpositions)
{
  std::vector<std::vector<unsigned int>> nextcosts;
  unsigned int mx, my;
  unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE / (maxLoops - loopsLeft);

  if (loopsLeft > 0)
  {
    for (auto it : costpositions)
    {
      for (int i = -1; i <= 1; i += 2)
      {
        for (int j = -1; j <= 1; j += 2)
        {
          mx = it[0] + i;
          my = it[1] + j;
          if (costmap.getCost(mx, my) < cost)
          {
            costmap.setCost(mx, my, cost);
            nextcosts.push_back({ mx, my });
          }
        }
      }
    }
  }

  loopsLeft--;
  inflateCostMap(loopsLeft, maxLoops, costmap, nextcosts);
  return;
}
