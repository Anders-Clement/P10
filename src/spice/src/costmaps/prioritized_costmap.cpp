#include <vector>
#include <algorithm>
#include "spice/costmaps/prioritized_costmap.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

PrioritizedCostmap::PrioritizedCostmap(CentralPathPlanner &central_path_planner) : Costmap(central_path_planner)
{
	get_robots_cli = m_central_path_planner.create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");
	get_ready_robots_timer =
		m_central_path_planner.create_wall_timer(1s, std::bind(&PrioritizedCostmap::get_robots_on_timer_cb, this));
	m_costmapPub = m_central_path_planner.create_publisher<nav_msgs::msg::OccupancyGrid>("/prioritized_costmap", 10);

	m_central_path_planner.declare_parameter("priority_scheme", 0);
	PRIORITY_SCHEME = m_central_path_planner.get_parameter("priority_scheme").get_parameter_value().get<int>();

	m_central_path_planner.declare_parameter("cost", nav2_costmap_2d::LETHAL_OBSTACLE);
	m_cost = m_central_path_planner.get_parameter("cost").get_parameter_value().get<unsigned char>();

	m_tf_buffer = std::make_unique<tf2_ros::Buffer>(m_central_path_planner.get_clock());
	m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

	if (PRIORITY_SCHEME > PRIORITY_OPTIONS)
	{
		RCLCPP_WARN(m_central_path_planner.get_logger(),
					"[PRIORITIZED COSTMAP] Priority_scheme: %d  does not exists, maximum is: %d defaulting to 0",
					PRIORITY_SCHEME, PRIORITY_OPTIONS);
		PRIORITY_SCHEME = 0;
	}

	m_central_path_planner.declare_parameter("future_lookup", 0);
	FUTURE_LOOKUP = m_central_path_planner.get_parameter("future_lookup").get_parameter_value().get<int>();

  RCLCPP_INFO(m_central_path_planner.get_logger(), "[PRIORITIZED COSTMAP] init with priority_scheme: %d ", PRIORITY_SCHEME);
  RCLCPP_INFO(m_central_path_planner.get_logger(), "[PRIORITIZED COSTMAP] init with future_lookup: %d", FUTURE_LOOKUP);
};

// get current full costmap, of map + any other layers added for a robot Id
std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::get_costmap(spice_msgs::msg::Id id)
{
	for (auto &robot : robots)
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

	auto get_robots_cb = [this](ServiceResponseFuture future)
	{
		robots.clear();
		auto result = future.get();
		for (auto robot : result->robots)
		{
			robots.push_back(robot.id);
		}
		if (PRIORITY_SCHEME == 2)
		{
			return;
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
	case 0: // static priorities, sort after robot ID
		std::sort(robots.begin(), robots.end(),
				  [](const spice_msgs::msg::Id &a, const spice_msgs::msg::Id &b)
				  { return a.id < b.id; });
		break;

	case 1: // dynamic priorities based on shortest path
		for (auto robot : robots)
		{
			std::optional<robot_plan> cur_robot_plan_opt = m_central_path_planner.get_last_plan_by_id(robot);
			if (!cur_robot_plan_opt)
			{
				robotPathPriorities.push_back(std::pair<spice_msgs::msg::Id, int>{robot, 0});
				continue;
			}
			robot_plan &robotPlan = cur_robot_plan_opt.value();
			robotPathPriorities.push_back(std::pair<spice_msgs::msg::Id, int>{robot, robotPlan.plan.poses.size()});
		}
		std::sort(robotPathPriorities.begin(), robotPathPriorities.end(),
				  [=](std::pair<spice_msgs::msg::Id, int> &a, std::pair<spice_msgs::msg::Id, int> &b)
				  {
					  return a.second < b.second;
				  });
		for (long unsigned int i = 0; i < robots.size(); i++)
		{
			robots[i] = robotPathPriorities[i].first;
		}
		break;

	case 2: // dynamic priorities based on failed planning
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

//   int priority = 0;
//   for(auto robot : robots){
// 	RCLCPP_INFO(m_central_path_planner.get_logger(),
// 				  "[PRIORITIZED COSTMAP] robot %s has priority %d",
// 				  robot.id.c_str(), priority);
// 	priority++;
//   }
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> PrioritizedCostmap::calcPrioritizedCostMap(spice_msgs::msg::Id robotId)
{
	std::shared_ptr<nav2_costmap_2d::Costmap2D> master_costmap =
		std::make_shared<nav2_costmap_2d::Costmap2D>(*m_global_costmap);

	nav2_costmap_2d::Costmap2D costmap;
	costmap.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
	costmap.resizeMap(master_costmap->getSizeInCellsX(), master_costmap->getSizeInCellsY(),
					  master_costmap->getResolution(), master_costmap->getOriginX(), master_costmap->getOriginY());
	current_robot_id = robotId;
	for (auto it : robots) // robots ordered according to priority
	{
		std::optional<robot_plan> cur_robot_plan_opt = m_central_path_planner.get_last_plan_by_id(it);
		if (it.id == current_robot_id.id)
		{
			if (!cur_robot_plan_opt)
			{
				RCLCPP_ERROR(m_central_path_planner.get_logger(), "Did not get plan for robot which costmap is calculated for");
			}
			robot_plan cur_robot_plan = cur_robot_plan_opt.value();
			geometry_msgs::msg::PoseStamped robot_pose = cur_robot_plan.start;
			unsigned int r_mx, r_my;

			// add other robots to costmap
			AddRobotsToGlobalCostmap(costmap);
			
			// get current robot w pose in order to clear costmap around it
			double angle_increment = 2.0 * M_PI / m_robot_points;
			for (float i = robot_pose.pose.position.x - ROBOT_RADIUS; i < robot_pose.pose.position.x + ROBOT_RADIUS; i += costmap.getResolution()/2)
			{
				for(float j = robot_pose.pose.position.y - ROBOT_RADIUS; j < robot_pose.pose.position.y + ROBOT_RADIUS; j += costmap.getResolution()/2){
					
					if (costmap.worldToMap(i, j, r_mx, r_my))
					{
						costmap.setCost(r_mx, r_my, nav2_costmap_2d::FREE_SPACE);
					}
				}
			}

			for (unsigned int i = 0; i < master_costmap->getSizeInCellsX(); i++)
			{
				for (unsigned int j = 0; j < master_costmap->getSizeInCellsY(); j++)
				{
					unsigned char cost = costmap.getCost(i, j);
					if (cost == nav2_costmap_2d::FREE_SPACE)
					{
						continue;
					}
					master_costmap->setCost(i, j, cost);
				}
			}

			nav_msgs::msg::OccupancyGrid occGrid;
			occGrid.header.frame_id = "map";
			occGrid.header.stamp = m_central_path_planner.now();
			occGrid.info.width = master_costmap->getSizeInCellsX();
			occGrid.info.height = master_costmap->getSizeInCellsY();
			occGrid.info.origin.position.x = master_costmap->getOriginX();
			occGrid.info.origin.position.y = master_costmap->getOriginY();
			occGrid.info.resolution = master_costmap->getResolution();
			occGrid.info.map_load_time = m_central_path_planner.now();
			occGrid.data.resize(master_costmap->getSizeInCellsX() * master_costmap->getSizeInCellsY());
			unsigned char *grid = master_costmap->getCharMap();
			for (unsigned int i = 0; i < master_costmap->getSizeInCellsX() * master_costmap->getSizeInCellsY(); i++)
			{
				occGrid.data[i] = *grid++;
			}
			m_costmapPub->publish(occGrid);

			return master_costmap;
		}

		std::vector<std::vector<unsigned int>> costpositions;
		if (!cur_robot_plan_opt)
			continue;
		robot_plan &cur_robot_plan = cur_robot_plan_opt.value();
		int index = 0;
		for (auto pose : cur_robot_plan.plan.poses)
		{
			if (index > FUTURE_LOOKUP && FUTURE_LOOKUP != 0)
			{
				break;
			}
			unsigned int mx, my;
			if (costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
			{
				if (costmap.getCost(mx, my) < m_cost)
				{
					costmap.setCost(mx, my, m_cost);
					costpositions.push_back({mx, my});
				}
			}
			index++;
		}

		int number_of_loops = round(INFLATION_RADIOUS / MAP_RESOLUTION);
		inflateCostMap(number_of_loops, number_of_loops, costmap, costpositions);
	}
	RCLCPP_WARN(m_central_path_planner.get_logger(), "went through all robots and got no machting id");
	return std::make_shared<nav2_costmap_2d::Costmap2D>(); // if loop through all robots without returning map
}

void PrioritizedCostmap::inflateCostMap(int loopsLeft, int maxLoops, nav2_costmap_2d::Costmap2D &costmap,
										std::vector<std::vector<unsigned int>> costpositions)
{
	std::vector<std::vector<unsigned int>> nextcosts;
	unsigned int mx, my;
	unsigned char cost = m_cost; // / ((maxLoops + 1 - loopsLeft)*0.25);

	if (loopsLeft > 0)
	{
		for (auto it : costpositions)
		{
			for (int i = -1; i <= 1; i ++)
			{
				for (int j = -1; j <= 1; j ++)
				{
					signed int checkx = it[0] + i;
					signed int checky = it[1] + j;
					if(checkx > costmap.getSizeInCellsX() || checkx < 0 || checky > costmap.getSizeInCellsY() || checky < 0){
						continue;
					}
					mx = it[0] + i;
					my = it[1] + j;
					if (costmap.getCost(mx, my) < cost)
					{
						costmap.setCost(mx, my, cost);
						nextcosts.push_back({mx, my});
					}
				}
			}
		}
		loopsLeft--;
		inflateCostMap(loopsLeft, maxLoops, costmap, nextcosts);
	}
	return;
}

void PrioritizedCostmap::AddRobotsToGlobalCostmap(nav2_costmap_2d::Costmap2D& costmap)
{
	for (auto robot : robots)
	{
		if (robot.id == current_robot_id.id)
		{
			continue;
		}

		try
		{
			auto robot_obs_tf = m_tf_buffer->lookupTransform("map", robot.id + "_base_link", tf2::TimePointZero);
			unsigned int mx, my;
			if (costmap.worldToMap(robot_obs_tf.transform.translation.x, robot_obs_tf.transform.translation.y, mx, my))
			{
				costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
			}

			double angle_increment = 2.0 * M_PI / m_robot_points;
			for (int i = 0; i < m_robot_points; i++)
			{

				float wx = robot_obs_tf.transform.translation.x + (ROBOT_RADIUS*2.0 * std::cos(angle_increment * i));
				float wy = robot_obs_tf.transform.translation.y + (ROBOT_RADIUS*2.0 * std::sin(angle_increment * i));

				if (costmap.worldToMap(wx, wy, mx, my))
				{
					costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
				}
			}
		}
		catch (const tf2::TransformException &ex)
		{
			RCLCPP_INFO(m_central_path_planner.get_logger(), "Could not transform %s to %s: %s", "map",
						(robot.id + "_base_link").c_str(), ex.what());
			continue;
		}
	}

	return;
}