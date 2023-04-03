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

  m_central_path_planner.declare_parameter("priority_scheme", 0);
  PRIORITY_SCHEME = m_central_path_planner.get_parameter("priority_scheme").get_parameter_value().get<int>();

  if(PRIORITY_SCHEME > PRIORITY_OPTIONS){
	RCLCPP_WARN(m_central_path_planner.get_logger(), "[PRIORITIZED COSTMAP] Priority_scheme: %d  does not exists, maximum is: %d defaulting to 0", PRIORITY_SCHEME, PRIORITY_OPTIONS);
	PRIORITY_SCHEME = 0;
	
  }

  m_central_path_planner.declare_parameter("future_lookup", 0);
  FUTURE_LOOKUP = m_central_path_planner.get_parameter("future_lookup").get_parameter_value().get<int>();

  RCLCPP_WARN(m_central_path_planner.get_logger(), "[PRIORITIZED COSTMAP] init with priority_scheme: %d ", PRIORITY_SCHEME);
  RCLCPP_WARN(m_central_path_planner.get_logger(), "[PRIORITIZED COSTMAP] init with future_lookup: %d", FUTURE_LOOKUP);
};

// get current full costmap, of map + any other layers added for a robot Id
std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::get_costmap(spice_msgs::msg::Id id)
{
  for (auto& robot : robots)
  {
	if (robot.id == id.id)
	  return calcPrioritizedCostMap(id);
  }
  RCLCPP_WARN(m_central_path_planner.get_logger(), "Cant find costmap for robot with id: %s", id.id.c_str());
  return std::make_shared<nav2_costmap_2d::Costmap2D>();
}

void PrioritizedCostmap::get_robots_on_timer_cb()
{
  auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
  get_robots_request->type.type = spice_msgs::msg::RobotType::CARRIER_ROBOT;

  using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

  auto get_robots_cb = [this](ServiceResponseFuture future) {
	robots.clear();
	auto result = future.get();
	for (auto robot : result->robots)
	{
	  robots.push_back(robot.id);
	}
	calcRobotPriorities();
  };

  auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_robots_cb);
}

void PrioritizedCostmap::PlanFailed(spice_msgs::msg::Id robot)
{
  if (PRIORITY_SCHEME == 2)
  {
	firstPrioRobot = robot;
	calcRobotPriorities();
  }
}

void PrioritizedCostmap::calcRobotPriorities()
{
  std::vector<std::pair<spice_msgs::msg::Id, int>> robotPathPriorities;

  switch (PRIORITY_SCHEME)
  {
	case 0:	 // static priorities, sort after robot ID
	  std::sort(robots.begin(), robots.end(),
				[](const spice_msgs::msg::Id& a, const spice_msgs::msg::Id& b) { return a.id < b.id; });
	  break;

	case 1:	 // dynamic priorities based on shortest path
	  for (auto robot : robots)
	  {
		robot_plan robotPlan = m_central_path_planner.get_last_plan_by_id(robot);
		robotPathPriorities.push_back(std::pair<spice_msgs::msg::Id, int>{ robot, robotPlan.plan.poses.size() });
	  }
	  std::sort(robotPathPriorities.begin(), robotPathPriorities.end(),
				[=](std::pair<spice_msgs::msg::Id, int>& a, std::pair<spice_msgs::msg::Id, int>& b) {
				  return a.second < b.second;
				});
	  for (int i = 0; i < robots.size(); i++)
	  {
		robots[i] = robotPathPriorities[i].first;
	  }
	  break;

	case 2:	 // dynamic priorities based on failed planning
	  for (auto itr = robots.begin(); itr != robots.end(); itr++)
	  {
		if (itr->id == firstPrioRobot.id)
		{
		  robots.erase(itr);
		  robots.insert(robots.begin(), firstPrioRobot);
		  break;
		}
	  }
	  RCLCPP_WARN(m_central_path_planner.get_logger(),
				  "[PRIORITIZED COSTMAP] could not find robot %s in order to change its priority",
				  firstPrioRobot.id.c_str());
	  break;

	default:
	  break;
  }
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::calcPrioritizedCostMap(spice_msgs::msg::Id robotId)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*m_global_costmap);
  for (auto it : robots)  // robots ordered according to priority
  {
	robot_plan cur_robot_plan = m_central_path_planner.get_last_plan_by_id(it);
	if (it.id == robotId.id)
	{
	  geometry_msgs::msg::PoseStamped robot_pose = cur_robot_plan.start;
	  unsigned int r_mx, r_my;

	  // get current robot w pose in order to clear costmap around it
	  if (costmap->worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, r_mx, r_my))
	  {
		// ensure cost map is not accessed outside of bounds (less than 0, more than x,y max)
		unsigned int start_x;
		if (r_mx < ceil(ROBOT_RADIUS / MAP_RESOLUTION))
		  start_x = -r_mx;
		else
		  start_x = r_mx - ceil(ROBOT_RADIUS / MAP_RESOLUTION);

		unsigned int start_y;
		if (r_my < ceil(ROBOT_RADIUS / MAP_RESOLUTION))
		  start_y = -r_my;
		else
		  start_y = r_my - ceil(ROBOT_RADIUS / MAP_RESOLUTION);

		unsigned int end_x;
		if (costmap->getSizeInCellsX() - r_mx < ceil(ROBOT_RADIUS / MAP_RESOLUTION))
		  end_x = costmap->getSizeInCellsX() - r_mx;
		else
		  end_x = r_mx + ceil(ROBOT_RADIUS / MAP_RESOLUTION);

		unsigned int end_y;
		if (costmap->getSizeInCellsY() - r_my < ceil(ROBOT_RADIUS / MAP_RESOLUTION))
		  end_y = costmap->getSizeInCellsY() - r_my;
		else
		  end_y = r_my + ceil(ROBOT_RADIUS / MAP_RESOLUTION);

		// clear cost map around the robot

		for (unsigned int i = start_x; i < end_x; i++)
		{
		  for (unsigned int j = start_y; j < end_y; j++)
		  {
			costmap->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
		  }
		}
	  }

	  else
	  {
		RCLCPP_WARN(m_central_path_planner.get_logger(),
					"[PRIORITIZED COSTMAP] could not transform from w space to m space, robot: %s at pose x= %f, y= %f",
					robotId.id.c_str(), robot_pose.pose.position.x, robot_pose.pose.position.y);
	  }

	  // publish prioritized costmap result
	  nav_msgs::msg::OccupancyGrid occGrid;
	  occGrid.header.frame_id = "map";
	  occGrid.header.stamp = m_central_path_planner.now();
	  occGrid.info.width = costmap->getSizeInCellsX();
	  occGrid.info.height = costmap->getSizeInCellsY();
	  occGrid.info.origin.position.x = costmap->getOriginX();
	  occGrid.info.origin.position.y = costmap->getOriginY();
	  occGrid.info.resolution = costmap->getResolution();
	  occGrid.info.map_load_time = m_central_path_planner.now();
	  occGrid.data.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY());
	  unsigned char* grid = costmap->getCharMap();
	  for (unsigned int i = 0; i < costmap->getSizeInCellsX() * costmap->getSizeInCellsY(); i++)
	  {
		occGrid.data[i] = *grid++;
	  }
	  m_costmapPub->publish(occGrid);

	  return costmap;
	}

	std::vector<std::vector<unsigned int>> costpositions;
	int index = 0;
	for (auto pose : cur_robot_plan.plan.poses)
	{
	  if (index > FUTURE_LOOKUP && FUTURE_LOOKUP != 0)
	  {
		break;
	  }
	  unsigned int mx, my;
	  if (costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
	  {
		if (costmap->getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE)
		{
		  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
		  costpositions.push_back({ mx, my });
		}
	  }
	  index++;
	}

	int number_of_loops = ceil(INFLATION_RADIOUS / MAP_RESOLUTION);
	inflateCostMap(number_of_loops, number_of_loops, *costmap, costpositions);
  }
  RCLCPP_WARN(m_central_path_planner.get_logger(), "went through all robots and got no machting id");
  return std::make_shared<nav2_costmap_2d::Costmap2D>();  // if loop through all robots without returning map
}

void PrioritizedCostmap::inflateCostMap(int loopsLeft, int maxLoops, nav2_costmap_2d::Costmap2D& costmap,
										std::vector<std::vector<unsigned int>> costpositions)
{
  std::vector<std::vector<unsigned int>> nextcosts;
  unsigned int mx, my;
  unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE / (maxLoops + 1 - loopsLeft);

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
