#include "rclcpp/rclcpp.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "spice_msgs/msg/id.hpp"

class Costmap
{
public:
    Costmap() = delete;
    Costmap(rclcpp::Node::SharedPtr node) : m_node(node) {};
    // get current full costmap, of map + any other layers added for a robot Id
    virtual std::shared_ptr<nav2_costmap_2d::Costmap2D> get_costmap(spice_msgs::msg::Id id) = 0;

protected:
    // map + inflation
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_global_costmap;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmap_subscriber;

    // central planner node
    rclcpp::Node::SharedPtr m_node;
};
