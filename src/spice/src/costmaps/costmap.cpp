#include "spice/costmaps/costmap.hpp"

Costmap::Costmap(CentralPathPlanner& central_path_planner) : m_central_path_planner(central_path_planner)
{
    m_costmap_subscriber = m_central_path_planner.create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/polybot05/global_costmap/costmap",
        10,
        std::bind(&Costmap::global_costmap_cb, this, std::placeholders::_1));
};

void Costmap::global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    m_global_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*msg);
    auto x_cells = m_global_costmap->getSizeInCellsX();
    auto y_cells = m_global_costmap->getSizeInCellsY();
    RCLCPP_INFO(m_central_path_planner.get_logger(), "got a costmap with size in cells x: %d, y: %d", x_cells, y_cells);
}