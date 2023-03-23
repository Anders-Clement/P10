#ifndef GLOBAL_COSTMAP_HPP
#define GLOBAL_COSTMAP_HPP

#include "spice/costmaps/costmap.hpp"

class GlobalCostmap : public Costmap
{
public:
    GlobalCostmap() = delete;
    GlobalCostmap(CentralPathPlanner& central_path_planner);
    // get current full costmap, of map + any other layers added for a robot Id
    std::shared_ptr<nav2_costmap_2d::Costmap2D> get_costmap(spice_msgs::msg::Id id) override;
private:

};

#endif // GLOBAL_COSTMAP_HPP