#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "spice_msgs/srv/get_robots_by_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotQueuer : public rclcpp::Node
{
public:
    RobotQueuer() : Node("RobotQueuer")
    {
        m_costmap_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap/costmap", 10, std::bind(&global_costmap_cb, this, std::placeholders::_1));

        m_global_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>();

        get_workcell_cli = create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");

        get_robots_cli = create_client<spice_msgs::srv::GetRobotsByState>("get_robots_by_state");

        get_robots_timer = create_wall_timer(5s, std::bind(&get_robots_on_timer_cb, this));

        get_workcells_timer = create_wall_timer(5s, std::bind(&get_workcells_on_timer_cb, this));

        tf2_ros::DynamicListenerQoS QoS;
        QoS.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true, QoS);
    }

    void global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        m_global_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*msg);
        auto x_cells = m_global_costmap->getSizeInCellsX();
        auto y_cells = m_global_costmap->getSizeInCellsY();
        RCLCPP_INFO_ONCE(get_logger(), "got a costmap with size in cells x: %d, y: %d", x_cells, y_cells);
        PreProcessing();
    }

    std::vector<std::vector<int>> BresenhamCircle(unsigned int r, int cx, int cy)
    {
        int x = 0;
        int y = r;
        int decisionParameter = 3 - 2 * r;
        std::vector<std::vector<int>> circlePoints;

        while (y >= x)
        {
            x++;
            if (decisionParameter > 0)
            {
                y--;
                decisionParameter = decisionParameter + 4 * (x - y) + 10;
            }
            else
            {
                decisionParameter = decisionParameter + 4 * x + 6;
            }

            circlePoints.push_back({cx + x, cy + y});
            circlePoints.push_back({cx + y, cy + x});

            if (cx - x > 0)
            {
                circlePoints.push_back({cx - x, cy + y});
                if (cy - y > 0)
                {
                    circlePoints.push_back({cx - x, cy - y});
                }
            }

            if (cy - x > 0)
            {
                circlePoints.push_back({cx + y, cy - x});
                if (cx - y > 0)
                {
                    circlePoints.push_back({cx - y, cy - x});
                }
            }

            if (cy - y > 0)
            {
                circlePoints.push_back({cx + x, cy - y});
            }

            if (cx - y > 0)
            {
                circlePoints.push_back({cx - y, cy + x});
            }
        }
    }

    void PreProcessing(){
        std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*m_global_costmap);
        std::vector<std::vector<int>> costPositions;
        for(int i = 0; i < costmap->getSizeInCellsX(); i++){
            for(int j = 0; j < costmap->getSizeInCellsY(); j++){
                if(costmap->getCost(i,j)==nav2_costmap_2d::LETHAL_OBSTACLE){
                    costPositions.push_back({i,j});
                }
            }
        }
        inflateCostMap(0,*costmap, costPositions);
    }

    void inflateCostMap(int loopnr, nav2_costmap_2d::Costmap2D &costmap, std::vector<std::vector<int>> costpositions)
    {
        std::vector<std::vector<int>> nextcosts;
        int mx, my;
        unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE / loopnr;

        if (costpositions.size() > 0)
        {
            for (auto it : costpositions)
            {
                for (int i = -1; i <= 1; i += 2)
                {
                    for (int j = -1; j <= 1; j += 2)
                    {
                        mx = it[0] + i;
                        my = it[1] + j;

                        //check mx and my is in boundaries of map and new cost is higher 
                        if (mx <= costmap.getSizeInCellsX() && mx >=0 && my <= costmap.getSizeInCellsY() && my >= 0 && costmap.getCost(mx, my) < cost) 
                        {
                            costmap.setCost(mx, my, cost);
                            nextcosts.push_back({mx, my});
                        }
                    }
                }
            }

            loopnr++;
            inflateCostMap(loopnr, costmap, nextcosts);
        }
        return;
    }

    std::vector<int> CalcQueuePositionCosts(int queueIndex, spice_msgs::msg::Id workcell)
    {
        // get workcell
        geometry_msgs::msg::TransformStamped workcellPose;
        // viable queue position and associated costs
        std::map<std::vector<int>, int> queueCircleCost;
        // obstacle to take into account when queueing, position, distance
        std::map<std::vector<int>, int> queueObstDists;
        std::vector<int> queuePoint;

        try
        {
            workcellPose = tf_buffer_->lookupTransform("map", workcell.id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Failed to calculate quqing pose, could not transform %s to %s: %s",
                        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
        }

        unsigned int mx, my;

        if(!costmap->worldToMap(workcellPose.transform.translation.x, workcellPose.transform.translation.y, mx, my)){
            std::vector<int> empty;
            return empty; //desired workcell is outside of map
        }

        // calc cirlce of possible positions
        auto circlePoints = BresenhamCircle(queueIndex, mx ,my);
        for (auto point : circlePoints)
        {
            queueCircleCost[point] = INFINITY;
        }

        // calc costs of poses in circle
        for (auto queue_robot : queuedRobots)
        {
            // check if robot is in same queue
            if (std::find(robotsInQueue.begin(), robotsInQueue.end(), queue_robot.id.id) != robotsInQueue.end())
            {
                break;
            }

            try
            {
                // calc dist from other queuing obstacles to posiiton circle

                auto obstPose = tf_buffer_->lookupTransform(workcell.id, queue_robot.id.id + "_base_link", tf2::TimePointZero);
                int circleDistx, circleDisty;
                if (obstPose.transform.translation.x > 0)
                    circleDistx = obstPose.transform.translation.x - queueIndex;
                else
                    circleDistx = obstPose.transform.translation.x + queueIndex;

                if (obstPose.transform.translation.y > 0)
                    circleDisty = obstPose.transform.translation.y - queueIndex;
                else
                    circleDisty = obstPose.transform.translation.y + queueIndex;

                int distToCircle = floor(sqrt(pow(circleDistx, 2) + pow(circleDisty, 2)));

                // map of robots poses and dists to circle
                queueObstDists[{circleDistx, circleDisty}] = distToCircle;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(),
                            ex.what());
                break;
            }
        }
        std::vector<std::pair<std::vector<int>, int>> vecQueueObstDists;
        std::copy(queueObstDists.begin(), queueObstDists.end(), std::back_inserter<std::vector<std::pair<std::vector<int>, int>>>(vecQueueObstDists));
        std::sort(vecQueueObstDists.begin(), vecQueueObstDists.end(),
                  [](const std::pair<std::vector<int>, int> &a, const std::pair<std::vector<int>, int> &b)
                  {
                if (a.second != b.second) {
                    return a.second < b.second;
                }
                return a.first < b.first; });

        while (circlePoints.size() > 0)
        {
            int dist = floor(vecQueueObstDists[0].second);
            auto costCircle = BresenhamCircle(vecQueueObstDists[0].second, vecQueueObstDists[0].first[0], vecQueueObstDists[0].first[1]);
            for (auto costPoint : costCircle)
            {
                if (std::find(circlePoints.begin(), circlePoints.end(), costPoint) != circlePoints.end())
                {
                    // fail safe
                    if (queueCircleCost[costPoint] > nav2_costmap_2d::LETHAL_OBSTACLE/dist)
                    {
                        // Oops oh no, someone (Oliver) made a whoopsy)
                        continue;
                    }
                    queueCircleCost[costPoint] = nav2_costmap_2d::LETHAL_OBSTACLE/dist;
                    circlePoints.erase(std::find(circlePoints.begin(), circlePoints.end(), costPoint));
                    
                }
            }

            vecQueueObstDists[0].second += 1;

            std::sort(vecQueueObstDists.begin(), vecQueueObstDists.end(),
                      [](const std::pair<std::vector<int>, int> &a, const std::pair<std::vector<int>, int> &b)
                      {
                if (a.second != b.second) {
                    return a.second < b.second;
                }
                return a.first < b.first; });
        }

        //best point to queue at
        return queuePoint;
    }

private:
    void get_workcells_on_timer_cb()
    {
        auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
        get_robots_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

        using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

        auto get_workcells_cb = [this](ServiceResponseFuture future)
        { this->workcells = future.get()->robots; };

        auto futureResult = get_workcell_cli->async_send_request(get_robots_request, get_workcells_cb);
    }

    void get_robots_on_timer_cb()
    {
        auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
        get_robots_request->type.type =
            spice_msgs::msg::RobotState::MR_PROCESSING_JOB; // Assumed this includes queueed robots

        using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

        auto get_workcells_cb = [this](ServiceResponseFuture future)
        { this->workcells = future.get()->robots; };

        auto futureResult = get_workcell_cli->async_send_request(get_robots_request, get_workcells_cb);
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr get_robots_timer{nullptr};
    rclcpp::TimerBase::SharedPtr get_workcells_timer{nullptr};
    std::vector<spice_msgs::msg::Robot> workcells;
    std::vector<spice_msgs::msg::Robot> queuedRobots;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_workcell_cli;
    rclcpp::Client<spice_msgs::srv::GetRobotsByState>::SharedPtr get_robots_cli;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> m_global_costmap;  
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_costmap_subscriber;
    std::vector<std::string> robotsInQueue;
    int QUEUE_DIST = 10; //* 0.05m dist between queue layers;
};