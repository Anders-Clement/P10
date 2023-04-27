#include <chrono>
#include "tf2/LinearMath/Matrix3x3.h"
#include "spice/work_cells/work_cell_queue_position_manager.hpp"

using namespace std::chrono_literals;

WorkCellQueuePositionManager::WorkCellQueuePositionManager(WorkCellStateMachine& workCellStateMachine) : m_workCellStateMachine(workCellStateMachine){
    
    lastTime = std::chrono::system_clock::now();

    try
    {
        //if(!m_workCellStateMachine.m_nodehandle.get_parameter_or("work_cell_rep_slope", 0.0)) // this if should make the try redundant
        //{
            m_workCellStateMachine.m_nodehandle.declare_parameter("work_cell_rep_slope", 0.05);
            m_workCellStateMachine.m_nodehandle.declare_parameter("carrier_bot_rep_slope", 0.1);
            m_workCellStateMachine.m_nodehandle.declare_parameter("wall_rep_slope", 0.1);
            m_workCellStateMachine.m_nodehandle.declare_parameter("queue_rep_slope", 0.1);
            m_workCellStateMachine.m_nodehandle.declare_parameter("work_cell_att_slope", 0.05);
            m_workCellStateMachine.m_nodehandle.declare_parameter("queue_att_slope", 0.0);
            m_workCellStateMachine.m_nodehandle.declare_parameter("map", "");
        //}
    }
    catch(rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
    {
        //RCLCPP_WARN(get_logger(), "[debug] params already declared?");
    }

    WORK_CELL_REP_SLOPE = m_workCellStateMachine.m_nodehandle.get_parameter("work_cell_rep_slope").get_parameter_value().get<float>();
    CARRIER_BOT_REP_SLOPE = m_workCellStateMachine.m_nodehandle.get_parameter("carrier_bot_rep_slope").get_parameter_value().get<float>();
    WALL_REP_SLOPE = m_workCellStateMachine.m_nodehandle.get_parameter("wall_rep_slope").get_parameter_value().get<float>();
    QUEUE_REP_SLOPE = m_workCellStateMachine.m_nodehandle.get_parameter("queue_rep_slope").get_parameter_value().get<float>();
    WORK_CELL_ATT_SLOPE = m_workCellStateMachine.m_nodehandle.get_parameter("work_cell_att_slope").get_parameter_value().get<float>();
    QUEUE_ATT_SLOPE = m_workCellStateMachine.m_nodehandle.get_parameter("queue_att_slope").get_parameter_value().get<float>();
    MAP_NAME = m_workCellStateMachine.m_nodehandle.get_parameter("map").get_parameter_value().get<std::string>();

    m_timer_q =  m_workCellStateMachine.m_nodehandle.create_wall_timer(0.1s, std::bind(&WorkCellQueuePositionManager::timer_update_q_locations, this));
    m_timer_robots_lists = m_workCellStateMachine.m_nodehandle.create_wall_timer(2s, std::bind(&WorkCellQueuePositionManager::timer_update_robots_lists, this));
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

    m_costmapPub =  m_workCellStateMachine.m_nodehandle.create_publisher<nav_msgs::msg::OccupancyGrid>("/queue_costmap/"+ m_workCellStateMachine.m_work_cell_name, 10);

    // polygon corners in world coordinates:
        
    if(MAP_NAME == "A4.yaml"){ // for A4:
        world_corners.push_back({2.85, -1.07}); // mid mid
        world_corners.push_back({2.63, -3.73}); // top mid
        world_corners.push_back({-1.81, -3.68}); // top right
        world_corners.push_back({-1.63, 2.12}); // bottom right
        world_corners.push_back({7.13, 1.92}); // bottom left
        world_corners.push_back({7.08, -0.85}); // mid left
    }
    else if(MAP_NAME == "C4.yaml"){ // for C4:
        world_corners.push_back({20.78, 8.91}); //gr window left
        world_corners.push_back({20.72, 6.08}); //gr window right
        world_corners.push_back({16.91, 6.18}); //gr door tv-side
        world_corners.push_back({16.82, 3.29}); //hall door gr-side
        world_corners.push_back({14.92, 3.36}); //hall door south
        world_corners.push_back({15.20, 18.33}); // hall tri-way south wall
        world_corners.push_back({17.25, 18.30}); // hall tri-way north wall
        world_corners.push_back({16.98, 9.07}); // gr door west
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

    double wx, wy;
    costpoints.clear();

    for (unsigned int x = 0; x < m_global_costmap->getSizeInCellsX(); x++)
    {
        for (unsigned int y = 0; y < m_global_costmap->getSizeInCellsY(); y++)
        {
            if(m_global_costmap->getCost(x,y) == nav2_costmap_2d::LETHAL_OBSTACLE){
                costpoints.push_back({x,y});
            }
            m_global_costmap->mapToWorld(x,y, wx, wy);

            if(pnpoly(world_corners.size(),world_corners_x, world_corners_y, wx, wy))
            {
                viable_points.push_back({x,y});
            }
        }
    }
    inflateCostMap(1, m_global_costmap, WALL_REP_SLOPE); //inflate cost of static map obstacles
}

void WorkCellQueuePositionManager::timer_update_q_locations(){
     //RCLCPP_INFO(get_logger, "[debug] timer for updating q frames for %s",m_work_cell_name.c_str());
    if(!workcell_costmap || workcell_list.size()== 0)
    {
        //RCLCPP_WARN(get_logger, "did not get costmap or workcells for queue");
        return;
    }
    m_mutex.lock();
    std::shared_ptr<nav2_costmap_2d::Costmap2D> carrier_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*workcell_costmap);

    // find coordinates of all carrier bots
    std::vector<std::pair<unsigned int, unsigned int>> carriers_map_coords;
    for (auto const &carrier : carrier_list)
        {
            // if(carrier.id.id == queueing_robot){
            //     continue;
            // }
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
    
    costpoints = carriers_map_coords;
    inflateCostMap(1,carrier_costmap, CARRIER_BOT_REP_SLOPE); // infalte Carrier_bot cost

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
    

    for (auto it = m_workCellStateMachine.m_queue_manager->m_queue_points.begin(); it != m_workCellStateMachine.m_queue_manager->m_queue_points.end(); it++)
    {
        unsigned int cheapest_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        unsigned int current_cost;
        std::pair<unsigned int, unsigned int> cheapest_point;
        double dt = m_workCellStateMachine.m_nodehandle.get_clock()->now().seconds() - it->lastTime; // delta time since last pos update
        int moveRange = round((MAX_Q_VEL*dt)/carrier_costmap->getResolution());
        //RCLCPP_WARN(get_logger(), "move range: %d",moveRange);
        unsigned int mx, my;
        double wx, wy;
        std::pair<unsigned int, unsigned int> queueMapPoint;

        if(!it->occupied && moveRange > 0){
        
        // for(auto point : viable_points){
        //     unsigned char current_cost = carrier_costmap->getCost(point.first,point.second);
        //     if(cheapest_cost > current_cost){
        //         cheapest_point = point;
        //         cheapest_cost = current_cost;
        //     }
        // }

        //transform queue point to world space
        
        tf2::Quaternion queue_q;
        
        queue_q.setW(it->transform.rotation.w);
        queue_q.setX(it->transform.rotation.x);
        queue_q.setY(it->transform.rotation.y);
        queue_q.setZ(it->transform.rotation.z);

        tf2::Matrix3x3 queue_rot(queue_q);
        tf2::Vector3 queue_t(it->transform.translation.x, it->transform.translation.y, it->transform.translation.z);
        tf2::Transform queue_tf_wc(queue_rot, queue_t);
        tf2::Vector3 queueToMap = wc_tf_world * queue_t;


        for(int x = -moveRange; x < moveRange; x++){
            for(int y = -moveRange; y < moveRange; y++){
                if(carrier_costmap->worldToMap(queueToMap.getX(), queueToMap.getY(), mx,my)){
                    signed int checkx = mx + x;
                    signed int checky = my + y;

                    if(checkx < carrier_costmap->getSizeInCellsX() && checkx > 0 && checky > 0 && checky < carrier_costmap->getSizeInCellsY()){
                        mx = checkx;
                        my = checky;
                        current_cost = carrier_costmap->getCost(mx,my);
                        if(cheapest_cost > current_cost){
                            carrier_costmap->mapToWorld(mx,my,wx,wy);
                            if(pnpoly(world_corners.size(), world_corners_x, world_corners_y, wx, wy)){
                                cheapest_cost = current_cost;
                                cheapest_point = {mx,my};
                            }

                        }
                    }
                }
            }
        }
        
        carrier_costmap->mapToWorld(cheapest_point.first, cheapest_point.second, wx, wy);

        //calc queue pose in work_cell space
        tf2::Vector3 queue_translation(wx, wy, 0.0);
        tf2::Vector3 queue_to_wc = world_tf_wc * queue_translation;

        it->transform.translation.x = queue_to_wc.getX();
        it->transform.translation.y = queue_to_wc.getY();
        queueMapPoint = cheapest_point;
        it->lastTime = m_workCellStateMachine.m_nodehandle.get_clock()->now().seconds();
        }
        else{
            if(carrier_costmap->worldToMap(it->transform.translation.x + m_workCellStateMachine.m_transform.translation.x, it->transform.translation.y + m_workCellStateMachine.m_transform.translation.y,mx,my));
            queueMapPoint = {mx,my};
        }
        costpoints = {queueMapPoint};
        inflateCostMap(1, carrier_costmap, QUEUE_REP_SLOPE); // Inflate queueu in costmap
        attraction(carrier_costmap, QUEUE_ATT_SLOPE, queueMapPoint); //add attraction to local queue points
        m_workCellStateMachine.publish_transform();
        
    }
    publish_costmap(carrier_costmap);

    m_mutex.unlock();
    
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
        update_workcell_costmap();
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
    };
    auto futureResult_carrier = get_carriers_cli->async_send_request(get_carriers_request, get_carriers_cb);

    return;
}


void WorkCellQueuePositionManager::update_workcell_costmap()
{   
    if(!m_global_costmap || workcell_list.size()== 0)
    {
        //RCLCPP_WARN(get_logger(), "did not get costmap or workcells for queue");
        return;
    }

    m_mutex.lock();

    workcell_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*m_global_costmap);
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

    costpoints = workcells_map_coords;
    inflateCostMap(1,workcell_costmap, WORK_CELL_REP_SLOPE); //Infalte workcell in costmap
    attraction(workcell_costmap, WORK_CELL_ATT_SLOPE, map_coord_entry); // set workcell att0raction
    m_mutex.unlock();

    return;
}

void WorkCellQueuePositionManager::attraction(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope, std::pair<unsigned int, unsigned int> attraction_center)
{
    slope = 1;
    unsigned char current_cost;
    float distance_to_center;
    unsigned char new_cost;

    for(auto point : viable_points)
    {
        current_cost = costmap->getCost(point.first, point.second);
        distance_to_center = sqrt(pow(std::max(point.first, attraction_center.first) - std::min(point.first, attraction_center.first), 2) + pow(std::max(point.second, attraction_center.second) - std::min(point.second, attraction_center.second), 2));
        new_cost = std::floor((nav2_costmap_2d::LETHAL_OBSTACLE/200)*distance_to_center*slope);
        if(new_cost > nav2_costmap_2d::LETHAL_OBSTACLE) new_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        if(new_cost > current_cost)
        {
            costmap->setCost(point.first, point.second, new_cost);
        }
    }

  return;
}


void WorkCellQueuePositionManager::inflateCostMap(int current_loop,  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, float slope)
{
    unsigned int mx, my;
    double wx, wy;
    unsigned int cost = std::floor(nav2_costmap_2d::LETHAL_OBSTACLE/pow(1+(current_loop*slope),2));
    std::vector<std::pair<unsigned int, unsigned int>> nextcosts;
    if(!costmap){
        RCLCPP_WARN(get_logger(), "costmap is fucked");
    }

    if (cost < 5 || costpoints.size() == 0 || cost > 255 || current_loop > 50)
    {
        //RCLCPP_WARN(get_logger(), "Exiting at loop: %d",current_loop);
        return;
    }
    for (auto it = costpoints.begin(); it < costpoints.end(); it++)
    {
        costmap->mapToWorld(it->first, it->second, wx,wy);
        if(!pnpoly(world_corners.size(), world_corners_x,world_corners_y,wx,wy)){
            continue;
        }
            for (int i = -1; i <= 1; i ++)
            {
                for (int j = -1; j <= 1; j ++)
                {
                    int signed checkx = it->first + i;
                    int signed checky = it->second + j;

                    if (checkx > costmap->getSizeInCellsX() || checky > costmap->getSizeInCellsY() || checkx < 0 || checky < 0)
                    {
                        continue;
                    }
                    mx = checkx;
                    my = checky;
                    if (costmap->getCost(mx, my) < cost)
                    {
                        costmap->setCost(mx, my, cost);
                        nextcosts.push_back({mx,my});
                    }
                }
            }  
        }

    costpoints = nextcosts;
    nextcosts.clear();
	current_loop ++;
	inflateCostMap(current_loop, costmap, slope);
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
