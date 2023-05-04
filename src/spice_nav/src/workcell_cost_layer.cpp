#include <chrono>
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "spice_nav/workcell_cost_layer.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace nav2_costmap_2d
{

    WorkcellCostLayer::WorkcellCostLayer()
        : last_min_x_(-std::numeric_limits<float>::max()), last_min_y_(-std::numeric_limits<float>::max()), last_max_x_(std::numeric_limits<float>::max()), last_max_y_(std::numeric_limits<float>::max())
    {
    }

    void WorkcellCostLayer::onInitialize()
    {
        nh_ = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("shape", rclcpp::ParameterValue(0));
        declareParameter("cost", rclcpp::ParameterValue(254));
        declareParameter("OFFSET_ENTRY", rclcpp::ParameterValue(0.25));
        declareParameter("OFFSET_EXIT", rclcpp::ParameterValue(0.25));
        declareParameter("robot_radius", rclcpp::ParameterValue(0.25));
        declareParameter("workcell_radius", rclcpp::ParameterValue(ROBOT_RADIUS));
        nh_->get_parameter(name_ + "." + "enabled", enabled_);
        nh_->get_parameter(name_ + "." + "shape", shape_);
        nh_->get_parameter(name_ + "." + "cost", cost_);
        nh_->get_parameter(name_ + "." + "OFFSET_ENTRY", OFFSET_ENTRY);
        nh_->get_parameter(name_ + "." + "OFFSET_EXIT", OFFSET_EXIT);
        nh_->get_parameter(name_ + "." + "robot_radius", ROBOT_RADIUS);
        nh_->get_parameter(name_ + "." + "workcell_radius", WORKCELL_RADIUS);

        transform_tolerance_ = tf2::durationFromSec(TF_TOLERANCE);

        rclcpp::QoS QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        QoS.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        global_frame_ = layered_costmap_->getGlobalFrameID();

        timer_ = nh_->create_wall_timer(5s, std::bind(&WorkcellCostLayer::get_robots_on_timer_cb, this));
        get_robots_cli = nh_->create_client<spice_msgs::srv::GetRobotsByType>("/get_robots_by_type");

        tf2_ros::DynamicListenerQoS tfQoS;
        QoS.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(nh_->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, nh_, true, tfQoS);

        current_ = true;
        default_value_ = NO_INFORMATION;
        matchSize();
        RCLCPP_INFO(logger_, "[WorkcellCostLayer] initialized");
    }

    void WorkcellCostLayer::get_robots_on_timer_cb()
    {
        if (!get_robots_cli->wait_for_service(1s))
        {
            RCLCPP_WARN(logger_, "Timeout on Swarm manager get_robots_by_type");
            return;
        }
        auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
        get_robots_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

        using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

        auto get_robots_cb = [this](ServiceResponseFuture future)
        { workcell_list = future.get()->robots; };

        auto futureResult = get_robots_cli->async_send_request(get_robots_request, get_robots_cb);
    }

    void WorkcellCostLayer::matchSize()
    {
        Costmap2D *master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
                  master->getOriginY());
    }

    void WorkcellCostLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x, double *min_y,
                                         double *max_x, double *max_y)
    {
        matchSize();
        geometry_msgs::msg::TransformStamped workcell_entry, workcell_exit;

        for (auto const &workcell : workcell_list)
        {
            try
            {
                workcell_entry = tf_buffer_->lookupTransform(global_frame_, workcell.id.id + "_entry", tf2::TimePointZero);
                workcell_exit = tf_buffer_->lookupTransform(global_frame_, workcell.id.id + "_exit", tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(nh_->get_logger(), "[WorkcellCostLayer]failed transform");
                continue;
            }

            double entry_wx, entry_wy, exit_wx, exit_wy;
            

            tf2::Quaternion q(workcell_entry.transform.rotation.x, workcell_entry.transform.rotation.y, workcell_entry.transform.rotation.z, workcell_entry.transform.rotation.w);
            tf2::Matrix3x3 rot_m(q);
            tf2::Vector3 entry_pos(workcell_entry.transform.translation.x, workcell_entry.transform.translation.y, workcell_entry.transform.translation.z);
            tf2::Vector3 exit_pos(workcell_exit.transform.translation.x, workcell_exit.transform.translation.y, workcell_exit.transform.translation.z);
            auto inv_m = rot_m.inverse();
            auto zero_rot_entry = inv_m * entry_pos;
            auto zero_rot_exit = inv_m * exit_pos;
            entry_wx = zero_rot_entry.getX() + OFFSET_ENTRY;
            entry_wy = zero_rot_entry.getY();
            exit_wx = zero_rot_exit.getX() - OFFSET_EXIT;
            exit_wy = zero_rot_exit.getY();


            

            unsigned int mx, my;
            std::vector<std::vector<double>> obstacle_points;
            if (shape_ == 0)
            { // tunnel
                double it_wx = entry_wx;

                while (it_wx < exit_wx)
                {
                    obstacle_points.push_back({it_wx, entry_wy + WORKCELL_RADIUS});
                    obstacle_points.push_back({it_wx, entry_wy - WORKCELL_RADIUS});
                    it_wx += 0.02; // obstacle resolution less than costmap resolution to prevent gaps
                                        
                }
            }
            else if (shape_ == 1)
            { // square:
                double it_wx = entry_wx;
                double it_wy = entry_wy - WORKCELL_RADIUS;
                
                while (it_wx < exit_wx)
                {
                    obstacle_points.push_back({it_wx, entry_wy + WORKCELL_RADIUS});
                    obstacle_points.push_back({it_wx, entry_wy - WORKCELL_RADIUS});
                    it_wx += 0.02; // obstacle resolution less than costmap resolution to prevent gaps
                }
                while (it_wy < entry_wy + WORKCELL_RADIUS)
                {

                    obstacle_points.push_back({entry_wx, it_wy});
                    obstacle_points.push_back({exit_wx, it_wy});
                    it_wy += 0.02; // obstacle resolution less than costmap resolution to prevent gaps
                }
            }
            else if (shape_ == 2)
            { // circle
                double c_wx = (exit_wx + entry_wx) / 2;
                double c_wy = (exit_wy + entry_wy) / 2;
                double r_w = c_wx-entry_wx;

                for (double it = 0; it < 2 * M_PI; it += M_PI / 100)
                {
                    obstacle_points.push_back({c_wx + r_w * cos(it), c_wy + r_w * sin(it)});
                }
            }
            else if (shape_ == 3)
            { // filled square
                double it_wx = entry_wx;
                double it_wy = entry_wy - WORKCELL_RADIUS;
            
                while (it_wy < entry_wy + WORKCELL_RADIUS)
                {
                    while (it_wx < exit_wx)
                    {
                        obstacle_points.push_back({it_wx, it_wy});
                        it_wx += 0.02; // obstacle resolution less than costmap resolution to prevent gaps
                    }
                    it_wy += 0.02; // obstacle resolution less than costmap resolution to prevent gaps
                    it_wx = entry_wx;
                }
            }
            else
            {
                RCLCPP_WARN(
                    nh_->get_logger(), "[WorkcellCostLayer]Current shape is not viable");
                return;
            }

            for (auto point : obstacle_points) // set cost for calculated obstacle points
            {
                tf2::Vector3 tfPoint(point[0], point[1], 0.0);
                tfPoint = rot_m * tfPoint;

                if (worldToMap(tfPoint.getX(), tfPoint.getY(), mx, my))
                {
                    setCost(mx, my, cost_);

                    *min_x = std::min(tfPoint.getX(), *min_x);
                    *min_y = std::min(tfPoint.getY(), *min_y);
                    *max_x = std::max(tfPoint.getX(), *max_x);
                    *max_y = std::max(tfPoint.getY(), *max_y);
                }
                else{
                    RCLCPP_WARN(
                    nh_->get_logger(), "[WorkcellCostLayer]could not transform tunnel point to costmap domain");
                }
            }
            
            // Add cost to exit area
            double work_cell_length = zero_rot_exit.getX() - zero_rot_entry.getX();
            
            std::vector<std::vector<double>> inflate_exit_w_points;
			for (double y = -WORKCELL_RADIUS; y < WORKCELL_RADIUS*2; y+=getResolution()) // width of a work cell
			{
				for (double x = -(work_cell_length+ROBOT_RADIUS); x < (ROBOT_RADIUS*2); x+=getResolution()) // goes from 1.5 robot radius before entry to 1.5 robot radius after exit
				{
                    inflate_exit_w_points.push_back({zero_rot_exit.getX() + x, zero_rot_exit.getY() + y});
				}
			}

            unsigned char exit_cost = 250;
            unsigned char current_cost;
            unsigned int exit_mx, exit_my;
            for(auto point : inflate_exit_w_points)
            {
                tf2::Vector3 un_rotated_point(point[0], point[1], 0.0); // is actually still rotated here
                un_rotated_point = rot_m * un_rotated_point;
                if(worldToMap(un_rotated_point.getX(), un_rotated_point.getY(), exit_mx, exit_my))
                {    
                    current_cost = getCost(exit_mx, exit_my);
                    if( current_cost < exit_cost || current_cost == nav2_costmap_2d::NO_INFORMATION)
                    {
                        setCost(exit_mx, exit_my, exit_cost);

                        *min_x = std::min(un_rotated_point.getX(), *min_x);
                        *min_y = std::min(un_rotated_point.getY(), *min_y);
                        *max_x = std::max(un_rotated_point.getX(), *max_x);
                        *max_y = std::max(un_rotated_point.getY(), *max_y);
                    }
                }
            }
        }
    }

    void WorkcellCostLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            return;
        }

        for (int j = min_j; j < max_j; j++)
        {
            for (int i = min_i; i < max_i; i++)
            {
                int index = getIndex(i, j);
                if (costmap_[index] == getDefaultValue())
                {
                    continue;
                }
                master_grid.setCost(i, j, costmap_[index]);
            }
        }
    }

    void WorkcellCostLayer::onFootprintChanged()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }


} // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::WorkcellCostLayer, nav2_costmap_2d::Layer)