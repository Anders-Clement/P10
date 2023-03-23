#include "spice/costmaps/global_costmap.hpp"

GlobalCostmap::GlobalCostmap(rclcpp::Node::SharedPtr node) : Costmap(node) {

};

// get current full costmap, of map + any other layers added for a robot Id
std::shared_ptr<nav2_costmap_2d::Costmap2D> GlobalCostmap::get_costmap(spice_msgs::msg::Id id)
{
    if(m_global_costmap)
        return m_global_costmap;
    else
    {
        RCLCPP_WARN(m_node->get_logger(), "Asked to get costmap, but do not have a costmap yet");
        return std::make_shared<nav2_costmap_2d::Costmap2D>();
    }
}
