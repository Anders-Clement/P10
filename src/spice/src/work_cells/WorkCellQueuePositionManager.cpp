#include <chrono>
#include "tf2/LinearMath/Matrix3x3.h"
#include "spice/work_cells/work_cell_queue_position_manager.hpp"

using namespace std::chrono_literals;

WorkCellQueuePositionManager::WorkCellQueuePositionManager(WorkCellStateMachine& workCellStateMachine) : m_workCellStateMachine(workCellStateMachine){
    
    lastTime = std::chrono::system_clock::now();
  
    param_map[spice_msgs::msg::Param::WORK_CELL_REP_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("work_cell_rep_slope").get_parameter_value().get<float>();
    param_map[spice_msgs::msg::Param::CARRIER_BOT_REP_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("carrier_bot_rep_slope").get_parameter_value().get<float>();
    param_map[spice_msgs::msg::Param::WALL_REP_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("wall_rep_slope").get_parameter_value().get<float>();
    param_map[spice_msgs::msg::Param::PLAN_REP_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("plan_rep_slope").get_parameter_value().get<float>();
    param_map[spice_msgs::msg::Param::QUEUE_REP_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("queue_rep_slope").get_parameter_value().get<float>();
    param_map[spice_msgs::msg::Param::WORK_CELL_ATT_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("work_cell_att_slope").get_parameter_value().get<float>();
    param_map[spice_msgs::msg::Param::QUEUE_ATT_SLOPE] = m_workCellStateMachine.m_nodehandle.get_parameter("queue_att_slope").get_parameter_value().get<float>();
    MAP_NAME = m_workCellStateMachine.m_nodehandle.get_parameter("map").get_parameter_value().get<std::string>();

    param_map[spice_msgs::msg::Param::MIN_MOVE_DIST] = m_workCellStateMachine.m_nodehandle.get_parameter("min_move_dist").get_parameter_value().get<int>();
    param_map[spice_msgs::msg::Param::MAX_Q_VEL] = m_workCellStateMachine.m_nodehandle.get_parameter("q_max_vel").get_parameter_value().get<float>();

    m_timer_static_map =  m_workCellStateMachine.m_nodehandle.create_wall_timer(1s, std::bind(&WorkCellQueuePositionManager::update_static_map_cost, this));
    m_timer_q =  m_workCellStateMachine.m_nodehandle.create_wall_timer(0.3s, std::bind(&WorkCellQueuePositionManager::timer_update_q_locations, this));
    m_timer_robots_lists = m_workCellStateMachine.m_nodehandle.create_wall_timer(1s, std::bind(&WorkCellQueuePositionManager::timer_update_robots_lists, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(m_workCellStateMachine.m_nodehandle.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    get_workcells_cli = m_workCellStateMachine.m_nodehandle.create_client<spice_msgs::srv::GetRobotsByType>("/get_robots_by_type");
    get_carriers_cli = m_workCellStateMachine.m_nodehandle.create_client<spice_msgs::srv::GetRobotsByType>("/get_robots_by_type");


    auto qos_profile_TL = rmw_qos_profile_default;
    qos_profile_TL.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    m_costmap_subscriber = m_workCellStateMachine.m_nodehandle.create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_TL.history, 10), qos_profile_TL),
        std::bind(&WorkCellQueuePositionManager::global_costmap_cb, this, std::placeholders::_1)
    );

    m_plans_subscriber = m_workCellStateMachine.m_nodehandle.create_subscription<spice_msgs::msg::RobotPlan>(
        "/robot_plans",
        10,
        std::bind(&WorkCellQueuePositionManager::plans_cb, this, std::placeholders::_1)
    );

    m_param_subcriber = m_workCellStateMachine.m_nodehandle.create_subscription<spice_msgs::msg::Param>("/queue_params",10,std::bind(&WorkCellQueuePositionManager::param_cb, this, std::placeholders::_1));
    m_costmapPub =  m_workCellStateMachine.m_nodehandle.create_publisher<nav_msgs::msg::OccupancyGrid>("/queue_costmap/"+ m_workCellStateMachine.m_work_cell_name, 10);

    // polygon corners in world coordinates:
        
    if(MAP_NAME == "A4.yaml"){ // for A4:
        world_corners.push_back({2.8104, -1.0494}); // mid mid
        world_corners.push_back({2.6282, -3.9158}); // top mid
        world_corners.push_back({-1.9368, -3.9276}); // top right
        world_corners.push_back({-1.7121, 2.1869}); // bottom right
        world_corners.push_back({7.2171, 1.9909}); // bottom left
        world_corners.push_back({7.1994, -0.8825}); // mid left
    }
    else if(MAP_NAME == "C4.yaml"){ // for C4:
        world_corners.push_back({21.0, 9.2}); //gr window left
        world_corners.push_back({20.9, 6.08}); //gr window right
        world_corners.push_back({17.1, 6.07}); //gr door tv-side
        world_corners.push_back({16.95, 3.12}); //hall door gr-side
        world_corners.push_back({14.8, 3.45}); //hall door south
        world_corners.push_back({15.2, 17.8}); // hall tri-way south wall
        world_corners.push_back({17.37, 18.0}); // hall tri-way north wall
        world_corners.push_back({17.3, 9.4}); // gr door west
    }
    else{
        RCLCPP_ERROR(get_logger(), "[WorkCellQueuePositionManager] %s did not get a map", m_workCellStateMachine.m_work_cell_name.c_str());
    }
    //std::vector<float> world_corners_x({2.4185,-1.7283,-1.5848,6.9987,7.0078,2.6586});
    //std::vector<float> world_corners_y({-3.6402,-3.5899,-2.0436,1.7771,-0.6922,-0.8671});
    for(auto pair: world_corners){
        world_corners_x.push_back(pair.first);
        world_corners_y.push_back(pair.second);
    }
}

void WorkCellQueuePositionManager::global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    m_global_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*msg);
    
    if(!m_global_costmap){
        return;
    }

    double wx, wy;
    static_map_cost_points.clear();

    for (unsigned int x = 0; x < m_global_costmap->getSizeInCellsX(); x++)
    {
        for (unsigned int y = 0; y < m_global_costmap->getSizeInCellsY(); y++)
        {
            if(m_global_costmap->getCost(x,y) == nav2_costmap_2d::LETHAL_OBSTACLE){
                static_map_cost_points.push_back({x,y});
            }
            m_global_costmap->mapToWorld(x,y, wx, wy);

            if(pnpoly(world_corners.size(),world_corners_x, world_corners_y, wx, wy))
            {
                viable_points.push_back({x,y});
            }
        }
    }
    update_static_map_cost();
    for (auto queue_point = m_workCellStateMachine.m_queue_manager->m_queue_points.begin(); queue_point != m_workCellStateMachine.m_queue_manager->m_queue_points.end(); queue_point++)
    {
        queue_point->lastTime = m_workCellStateMachine.m_nodehandle.get_clock()->now().seconds();
    }
}

 void WorkCellQueuePositionManager::plans_cb(spice_msgs::msg::RobotPlan::SharedPtr msg){
    m_all_robot_plans[msg->id.id] = msg->plan;
 }

void WorkCellQueuePositionManager::param_cb(spice_msgs::msg::Param::SharedPtr msg){
    param_map[msg->param] = msg->value;
}

void WorkCellQueuePositionManager::update_static_map_cost(){
    if(!m_global_costmap){
        return;
    }
    static_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*m_global_costmap);
    inflateCostMap(1, static_costmap, param_map[spice_msgs::msg::Param::WALL_REP_SLOPE], static_map_cost_points); //inflate cost of static map obstacles
    update_workcell_costmap();
}

void WorkCellQueuePositionManager::timer_update_q_locations()
{
    // RCLCPP_INFO(get_logger, "[debug] timer for updating q frames for %s",m_work_cell_name.c_str());
    if (!workcell_costmap || workcell_list.size() == 0 || !m_global_costmap || !static_costmap)
    {
        // RCLCPP_WARN(get_logger, "did not get costmap or workcells for queue");
        return;
    }
    
    //setup transforms between world and workcell
    tf2::Quaternion wc_q;
        
    wc_q.setW(m_workCellStateMachine.m_transform.rotation.w);
    wc_q.setX(m_workCellStateMachine.m_transform.rotation.x);
    wc_q.setY(m_workCellStateMachine.m_transform.rotation.y);
    wc_q.setZ(m_workCellStateMachine.m_transform.rotation.z);
    tf2::Matrix3x3 wc_rot(wc_q);

    tf2::Vector3 wc_t(m_workCellStateMachine.m_transform.translation.x, m_workCellStateMachine.m_transform.translation.y, m_workCellStateMachine.m_transform.translation.z);
    
    tf2::Transform wc_tf_world(wc_rot, wc_t);
    tf2::Transform world_tf_wc = wc_tf_world.inverse();
    std::shared_ptr<nav2_costmap_2d::Costmap2D> queue_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*workcell_costmap);
    std::shared_ptr<nav2_costmap_2d::Costmap2D> carrier_costmap;
    for (auto it = m_workCellStateMachine.m_queue_manager->m_queue_points.begin(); it != m_workCellStateMachine.m_queue_manager->m_queue_points.end(); it++)
    {
        // find coordinates of all carrier bots
        std::vector<std::pair<unsigned int, unsigned int>> carriers_map_coords;
        carrier_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*workcell_costmap);
        for (auto const &carrier : carrier_list)
        {
            if (it->occupied)
            {
                if (carrier.id.id == it->queued_robot.id)
                {
                    continue;
                }
            }
            try
            {
                unsigned int map_x_coord, map_y_coord;
                auto carrier_tf = tf_buffer_->lookupTransform("map", carrier.id.id+"_base_link", tf2::TimePointZero);
                m_global_costmap->worldToMap(carrier_tf.transform.translation.x, carrier_tf.transform.translation.y, map_x_coord, map_y_coord);
                carriers_map_coords.push_back({map_x_coord, map_y_coord});
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "[update_q_location] failed transform");
                continue;
            }
        }
        
        inflateCostMap(1,carrier_costmap, param_map[spice_msgs::msg::Param::CARRIER_BOT_REP_SLOPE], carriers_map_coords); // infalte Carrier_bot cost

        std::vector<std::pair<unsigned int, unsigned int>> carriers_plan_coords;
        for (auto robot_plan = m_all_robot_plans.begin(); robot_plan != m_all_robot_plans.end(); robot_plan++)
        {
            if (it->occupied)
            {
                if (robot_plan->first == it->queued_robot.id)
                {
                    continue;
                }
            }
            unsigned int mx, my;
            for (auto pose : m_all_robot_plans[robot_plan->first].poses)
            {
                if (carrier_costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
                {
                    carriers_plan_coords.push_back({mx, my});
                }
            }
        }
        inflateCostMap(1,carrier_costmap, param_map[spice_msgs::msg::Param::PLAN_REP_SLOPE], carriers_plan_coords);

        for(unsigned int x = 0; x < queue_costmap->getSizeInCellsX(); x++){
            for(unsigned int y = 0; y < queue_costmap->getSizeInCellsY(); y++){
                if(queue_costmap->getCost(x,y) > carrier_costmap->getCost(x,y) && queue_costmap->getCost(x,y) != nav2_costmap_2d::NO_INFORMATION){
                    carrier_costmap->setCost(x,y,queue_costmap->getCost(x,y));
                }
            }
        }

        unsigned int cheapest_cost = nav2_costmap_2d::NO_INFORMATION;
        unsigned int current_cost;
        
        std::pair<unsigned int, unsigned int> cheapest_point;
        double dt = m_workCellStateMachine.m_nodehandle.get_clock()->now().seconds() - it->lastTime; // delta time since last pos update
        int moveRange = round((param_map[spice_msgs::msg::Param::MAX_Q_VEL]*dt)/carrier_costmap->getResolution());
        //RCLCPP_WARN(get_logger(), "move range: %d",moveRange);
        unsigned int mx, my;
        double wx, wy;
        std::pair<unsigned int, unsigned int> queueMapPoint;
        
        if(moveRange > param_map[spice_msgs::msg::Param::MIN_MOVE_DIST]){

            tf2::Quaternion queue_q;

            queue_q.setW(it->transform.rotation.w);
            queue_q.setX(it->transform.rotation.x);
            queue_q.setY(it->transform.rotation.y);
            queue_q.setZ(it->transform.rotation.z);

            tf2::Matrix3x3 queue_rot(queue_q);
            tf2::Vector3 queue_t(it->transform.translation.x, it->transform.translation.y, it->transform.translation.z);
            tf2::Vector3 queueToMap = wc_tf_world * queue_t;

            for (int x = -moveRange; x < moveRange; x++)
            {
                for (int y = -moveRange; y < moveRange; y++)
                {
                    if (carrier_costmap->worldToMap(queueToMap.getX(), queueToMap.getY(), mx, my))
                    {
                        signed int checkx = mx + x;
                        signed int checky = my + y;

                        if (checkx < carrier_costmap->getSizeInCellsX() && checkx > 0 && checky > 0 && checky < carrier_costmap->getSizeInCellsY())
                        {
                            mx = checkx;
                            my = checky;
                            current_cost = carrier_costmap->getCost(mx, my);
                            if (cheapest_cost >= current_cost)
                            {
                                carrier_costmap->mapToWorld(mx, my, wx, wy);
                                if (pnpoly(world_corners.size(), world_corners_x, world_corners_y, wx, wy))
                                {
                                    cheapest_cost = current_cost;
                                    cheapest_point = {mx, my};
                                }
                            }
                        }
                    }
                }
            }

            carrier_costmap->mapToWorld(cheapest_point.first, cheapest_point.second, wx, wy);

            // calc queue pose in work_cell space
            tf2::Vector3 queue_translation(wx, wy, 0.0);
            tf2::Vector3 queue_to_wc = world_tf_wc * queue_translation;
            tf2::Vector3 norm_vec_to_entry = (wc_tf_world*tf2::Vector3(m_workCellStateMachine.m_entry_transform.translation.x, m_workCellStateMachine.m_entry_transform.translation.y, 0.0));
            norm_vec_to_entry = (norm_vec_to_entry-queue_translation).normalize();

            double angle =atan2(norm_vec_to_entry.getY(),norm_vec_to_entry.getX());
        
            tf2::Quaternion q_to_entry_world;
            q_to_entry_world.setRPY(0.0,0.0,angle);
            
            tf2::Quaternion q_to_entry_wc = q_to_entry_world * world_tf_wc.getRotation();
            
            if(queue_to_wc.getX() == 0.0 || queue_to_wc.getY() == 0.0)
            {
                continue;
            }
            
            //RCLCPP_WARN(get_logger(), "queue point world space x: %f ,y: %f",it->transform.translation.x, it->transform.translation.y);
            queueMapPoint = cheapest_point;
            it->lastTime = m_workCellStateMachine.m_nodehandle.get_clock()->now().seconds();

            it->transform.translation.x = queue_to_wc.getX();
            it->transform.translation.y = queue_to_wc.getY();
            it->transform.rotation.x = q_to_entry_wc.getX();
            it->transform.rotation.y = q_to_entry_wc.getY();
            it->transform.rotation.z = q_to_entry_wc.getZ();
            it->transform.rotation.w = q_to_entry_wc.getW();
            
            //RCLCPP_WARN(get_logger(), "queue point workcell space x: %f ,y: %f",it->transform.translation.x, it->transform.translation.y);
           
        }
        else
        {   
            tf2::Vector3 q_w_position =  wc_tf_world * tf2::Vector3(it->transform.translation.x,it->transform.translation.y, 0.0 );
            if (carrier_costmap->worldToMap(q_w_position.getX(), q_w_position.getY(), mx, my))
            {
                queueMapPoint = {mx, my};
            }
        }
        inflateCostMap(1, queue_costmap, param_map[spice_msgs::msg::Param::QUEUE_REP_SLOPE],{queueMapPoint}); // Inflate queueu in costmap
        attraction(queue_costmap, param_map[spice_msgs::msg::Param::QUEUE_ATT_SLOPE], queueMapPoint); //add attraction to local queue points
    }
    m_workCellStateMachine.publish_transform();
    m_workCellStateMachine.m_queue_manager->publish_queue_points();

    publish_costmap(carrier_costmap);
    return;
}

void WorkCellQueuePositionManager::timer_update_robots_lists(){
    // get all workcells
    if (!get_workcells_cli->wait_for_service(1s))
    {
        RCLCPP_WARN(get_logger(), "Timeout on Swarm manager get_robots_by_type: workcells");
        return;
    }

    auto get_workcells_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
    get_workcells_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;
    using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;
    auto get_workcells_cb = [this](ServiceResponseFuture future)
    {
        workcell_list = future.get()->robots;
    };
    auto futureResult_ws = get_workcells_cli->async_send_request(get_workcells_request, get_workcells_cb);


    //get carrier robots
    if (!get_carriers_cli->wait_for_service(1s))
    {
        RCLCPP_WARN(get_logger(), "Timeout on Swarm manager get_robots_by_type: carrier");
        return;
    }
    auto get_carriers_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
    get_carriers_request->type.type = spice_msgs::msg::RobotType::CARRIER_ROBOT;
    using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;
    auto get_carriers_cb = [this](ServiceResponseFuture future)
    { 
        carrier_list = future.get()->robots;
        std::vector<std::string> mapKeys;

        for(auto keys : m_all_robot_plans){
            mapKeys.push_back(keys.first);
        }

        for (auto robot_plan = mapKeys.begin(); robot_plan != mapKeys.end(); robot_plan++)
        {
            for(auto carrier: carrier_list){
                if(carrier.id.id == *robot_plan){
                    break;
                }
                m_all_robot_plans.erase(*robot_plan);
            }
        }
    };

    auto futureResult_carrier = get_carriers_cli->async_send_request(get_carriers_request, get_carriers_cb);

    return;
}


void WorkCellQueuePositionManager::update_workcell_costmap()
{   
    if(!m_global_costmap || workcell_list.size()== 0 || !static_costmap)
    {
        //RCLCPP_WARN(get_logger(), "did not get costmap or workcells for queue");
        return;
    }

    workcell_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*static_costmap);
    std::vector<std::pair<unsigned int, unsigned int>> workcells_map_coords;
    
    // find coordinates of all workcell bots
    for (auto const &workcell : workcell_list)
    {
        try
        {
            unsigned int map_x_coord, map_y_coord;
            auto workcell_tf = tf_buffer_->lookupTransform("map", workcell.id.id, tf2::TimePointZero);
            m_global_costmap->worldToMap(workcell_tf.transform.translation.x, workcell_tf.transform.translation.y, map_x_coord, map_y_coord);
            workcells_map_coords.push_back({map_x_coord, map_y_coord});

            unsigned int map_x_coord_entry, map_y_coord_entry;
            unsigned int map_x_coord_exit, map_y_coord_exit;
            auto workcell_tf_entry = tf_buffer_->lookupTransform("map", workcell.id.id+ "_entry", tf2::TimePointZero);
            auto workcell_tf_exit = tf_buffer_->lookupTransform("map", workcell.id.id+ "_exit", tf2::TimePointZero);
            m_global_costmap->worldToMap(workcell_tf_entry.transform.translation.x, workcell_tf_entry.transform.translation.y, map_x_coord_entry, map_y_coord_entry);
            m_global_costmap->worldToMap(workcell_tf_exit.transform.translation.x, workcell_tf_exit.transform.translation.y, map_x_coord_exit, map_y_coord_exit);
            workcells_map_coords.push_back({map_x_coord_entry, map_y_coord_entry});
            workcells_map_coords.push_back({map_x_coord_exit, map_y_coord_exit});
            if (workcell.id.id == m_workCellStateMachine.m_work_cell_name)
            {
                map_coord_entry = {map_x_coord_entry, map_y_coord_entry};
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "[update_q_location] failed transform");
            continue;
        }

    }

    inflateCostMap(1,workcell_costmap, param_map[spice_msgs::msg::Param::WORK_CELL_REP_SLOPE], workcells_map_coords); //Infalte workcell in costmap
    attraction(workcell_costmap, param_map[spice_msgs::msg::Param::WORK_CELL_ATT_SLOPE], map_coord_entry); // set workcell att0raction
    return;
}

void WorkCellQueuePositionManager::attraction(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope, std::pair<unsigned int, unsigned int> attraction_center)
{   
    unsigned char current_cost;
    float sqr_distance_to_center;
    signed int new_cost;
    float dist_goal;
    float dist_threshold = 5.0; // meters until switch from quadratic attraction to conic

    for(auto point : viable_points)
    {
        current_cost = costmap->getCost(point.first, point.second);
        dist_goal = sqrt(pow(std::max(point.first, attraction_center.first) - std::min(point.first, attraction_center.first), 2) + pow(std::max(point.second, attraction_center.second) - std::min(point.second, attraction_center.second), 2));
        if(dist_goal > dist_threshold){
            new_cost = std::floor(0.5*slope*pow(dist_goal,2));
        }
        else{
            new_cost = std::floor((dist_threshold * slope * dist_goal - 0.5*slope*pow(dist_threshold,2)));
        }
        new_cost = new_cost+current_cost;
                    
        if(new_cost > nav2_costmap_2d::LETHAL_OBSTACLE || new_cost < 0){
        
            new_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }   
            costmap->setCost(point.first, point.second, new_cost);
    }
  return;
}   


void WorkCellQueuePositionManager::inflateCostMap(int current_loop,  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope, std::vector<std::pair<unsigned int, unsigned int>> cost_points)
{
    unsigned int mx, my;
    double wx, wy;
    unsigned int cost = std::floor(0.5*slope*pow(1.0/(1.0+(current_loop)*0.05),2));
    std::vector<std::pair<unsigned int, unsigned int>> nextcosts;


    if (cost > 255)
    {
        cost = 255;
    }
                
    if(!costmap){
        RCLCPP_WARN(get_logger(), "got no costmap");
        return;
    }

    if (cost < 5 || cost_points.size() == 0 || cost > 255 || current_loop > 250)
    {
        // RCLCPP_WARN(get_logger(), "Exiting at loop: %d",current_loop);
        return;
    }
    for (auto it = cost_points.begin(); it < cost_points.end(); it++)
    {
        costmap->mapToWorld(it->first, it->second, wx, wy);
        if (!pnpoly(world_corners.size(), world_corners_x, world_corners_y, wx, wy))
        {
            continue;
        }
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int signed checkx = it->first + i;
                int signed checky = it->second + j;

                if (checkx > costmap->getSizeInCellsX() || checky > costmap->getSizeInCellsY() || checkx < 0 || checky < 0)
                {
                    continue;
                }
                mx = checkx;
                my = checky;
                unsigned int current_cost = costmap->getCost(mx, my);
                if(cost > current_cost){
                    costmap->setCost(mx, my, cost);
                nextcosts.push_back({mx, my});
                }
            }
        }
    }

    cost_points = nextcosts;
    nextcosts.clear();
	current_loop ++;
	inflateCostMap(current_loop, costmap, slope, cost_points);
    //RCLCPP_WARN(get_logger(), "Returning: Loop: %d ,inflation cost: %d, costpositions.size(): %ld", current_loop, cost, costpositions.size());

  return;
}


int WorkCellQueuePositionManager::pnpoly(int nvert, std::vector<float> vertx, std::vector<float> verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
	 (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

void WorkCellQueuePositionManager::publish_costmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap){
    nav_msgs::msg::OccupancyGrid occGrid;
	  occGrid.header.frame_id = "map";
	  occGrid.header.stamp = m_workCellStateMachine.m_nodehandle.now();
	  occGrid.info.width = costmap->getSizeInCellsX();
	  occGrid.info.height = costmap->getSizeInCellsY();
	  occGrid.info.origin.position.x = costmap->getOriginX();
	  occGrid.info.origin.position.y = costmap->getOriginY();
	  occGrid.info.resolution = costmap->getResolution();
	  occGrid.info.map_load_time = m_workCellStateMachine.m_nodehandle.now();
	  occGrid.data.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY());
	  unsigned char* grid = costmap->getCharMap();
	  for (unsigned int i = 0; i < costmap->getSizeInCellsX() * costmap->getSizeInCellsY(); i++)
	  {
		occGrid.data[i] = *grid++;
	  }
	  m_costmapPub->publish(occGrid);
      return;
}

rclcpp::Logger WorkCellQueuePositionManager::get_logger(){
    return m_workCellStateMachine.m_nodehandle.get_logger();
} 
