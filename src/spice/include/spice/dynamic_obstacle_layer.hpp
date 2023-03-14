#ifndef DYNAMIC_OBSTACLE_LAYER_HPP
#define DYNAMIC_OBSTACLE_LAYER_HPP

#include "string"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "spice_msgs/srv/get_robots_by_type.hpp"


class DynamicObstacleLayer : public nav2_costmap_2d::Layer
{   
    public:
    DynamicObstacleLayer();
    
    virtual void onInitialize();
    
    virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

    virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

    virtual void reset(){
        
        return;
    }

    virtual void onFootprintChanged();

    virtual bool isClearable() {return false;}

    virtual void DynamicObstacleCallback();


private:

    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
    bool need_recalculation_;

  // Size of gradient in cells
    int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
    int GRADIENT_FACTOR = 10;

    std::string topic_;  
    rclcpp::Subscription<spice_msgs::msg::DynamicObstacle>::SharedPtr subscription_;
    
};

#endif