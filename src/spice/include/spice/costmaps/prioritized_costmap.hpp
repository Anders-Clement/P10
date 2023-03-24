#include "map.hpp"
#include "spice/costmaps/costmap.hpp"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "spice_msgs/msg/robot.hpp"



class PrioritizedCostmap : public Costmap
{
public:
    PrioritizedCostmap() = delete;
    PrioritizedCostmap(CentralPathPlanner& central_path_planner);

    virtual void get_robots_on_timer_cb();

    virtual std::shared_ptr<nav2_costmap_2d::Costmap2D> calcPrioritizedCostMap(spice_msgs::msg::Id robotId);

    virtual void inflateCostMap(int loopsLeft, int maxLoops, nav2_costmap_2d::Costmap2D& costmap, std::vector<std::vector<unsigned int>> costpositions);
    

    // get current full costmap, of map + any other layers added for a robot Id
    std::shared_ptr<nav2_costmap_2d::Costmap2D> get_costmap(spice_msgs::msg::Id id) override;
private:
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_robots_cli;
    rclcpp::TimerBase::SharedPtr get_ready_robots_timer{nullptr};
    std::vector<std::string> robots;
    double MAP_RESOLUTION = 0.05;
    double INFLATION_RADIOUS = 0.35;
};