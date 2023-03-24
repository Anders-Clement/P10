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
	m_costmapPub = m_central_path_planner.create_publisher<nav_msgs::msg::OccupancyGrid>("/prioritized_costmap", 10);
};

// get current full costmap, of map + any other layers added for a robot Id
std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::get_costmap(spice_msgs::msg::Id id)
{
  if (std::find(robots.begin(), robots.end(), id.id) != robots.end())
  {
	return calcPrioritizedCostMap(id);
  }
  else
  {
	RCLCPP_WARN(m_central_path_planner.get_logger(), "Asked to get costmap, but do not have a costmap yet");
	return std::make_shared<nav2_costmap_2d::Costmap2D>();
  }
}

void PrioritizedCostmap::get_robots_on_timer_cb()
{
  auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
  get_robots_request->type.type = spice_msgs::msg::RobotType::CARRIER_ROBOT;

  using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

  auto get_robots_cb = [this](ServiceResponseFuture future) {
	for (auto robot : future.get()->robots)
	{
	  robots.push_back(robot.id.id);
	  std::sort(robots.begin(), robots.end());
	}
  };

  auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_robots_cb);
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::calcPrioritizedCostMap(spice_msgs::msg::Id robotId)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*m_global_costmap);

  for (auto it : robots)  // robots ordered according to priority
  {
	if (it == robotId.id)
	{

		nav_msgs::msg::OccupancyGrid occGrid;
		occGrid.header.frame_id = "map";
		occGrid.header.stamp = m_central_path_planner.now();
		occGrid.info.width = costmap->getSizeInCellsX();
		occGrid.info.height = costmap->getSizeInCellsY();
		occGrid.info.origin.position.x = costmap->getOriginX();
		occGrid.info.origin.position.y = costmap->getOriginY();
		occGrid.info.resolution = costmap->getResolution();
		occGrid.info.map_load_time = m_central_path_planner.now();
		occGrid.data.resize(costmap->getSizeInCellsX()*costmap->getSizeInCellsY());
		unsigned char* grid = costmap->getCharMap();
		for(unsigned int i = 0; i < costmap->getSizeInCellsX()*costmap->getSizeInCellsY(); i++){
			occGrid.data[i] = *grid++;
		}
		m_costmapPub->publish(occGrid);

	  return costmap;
	}

	nav_msgs::msg::Path robotPath = m_central_path_planner.get_last_plan_by_id(robotId);
	std::vector<std::vector<unsigned int>> costpositions;

	for (auto pose : robotPath.poses)
	{
	  unsigned int mx, my;
	  if (costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
	  {
		if (costmap->getCost(mx, my) <= nav2_costmap_2d::LETHAL_OBSTACLE)
		{
		  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
		  costpositions.push_back({ mx, my });
		}
	  }

	}

	int number_of_loops = ceil(INFLATION_RADIOUS / MAP_RESOLUTION);
	inflateCostMap(number_of_loops, number_of_loops, *costmap, costpositions);
  }
  	return nullptr; // if loop through all robots without returning map
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
	loopsLeft--;
  	inflateCostMap(loopsLeft, maxLoops, costmap, nextcosts);
  }


  return;
}
