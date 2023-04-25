#include <string>
#include <rclcpp/node.hpp>
#include <optional>
#include "tf2/LinearMath/Matrix3x3.h"
#include "spice_msgs/msg/robot_type.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"

using namespace std::chrono_literals;

WorkCellStateMachine::WorkCellStateMachine(std::string work_cell_name, spice_msgs::msg::RobotType::_type_type robot_type, 
    rclcpp::Node& node_handle, geometry_msgs::msg::Transform transform)
:  m_nodehandle(node_handle), m_work_cell_name(work_cell_name), m_robot_type(robot_type), m_transform(transform)
{
    // m_nodehandle.declare_parameter("robot_radius",rclcpp::ParameterValue(0.25));
    // m_nodehandle.declare_parameter("workcell_radius",rclcpp::ParameterValue(0.25));
    // ROBOT_RADIUS = m_nodehandle.get_parameter("robot_radius").as_double();
    // WORKCELL_RADIUS = m_nodehandle.get_parameter("workcell_radius").as_double();
    m_register_work_service = m_nodehandle.create_service<spice_msgs::srv::RegisterWork>(
        m_work_cell_name + "/register_work", 
        std::bind(&WorkCellStateMachine::on_register_work, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_ready_for_processing_service = m_nodehandle.create_service<std_srvs::srv::Trigger>(
        m_work_cell_name + "/robot_ready_for_processing", 
        std::bind(&WorkCellStateMachine::on_robot_ready_for_processing, this, std::placeholders::_1, std::placeholders::_2));
    m_robot_ready_in_queue_service = m_nodehandle.create_service<spice_msgs::srv::RobotReady>(
        m_work_cell_name + "/robot_ready_in_queue", 
        std::bind(&WorkCellStateMachine::on_robot_ready_in_queue, this, std::placeholders::_1, std::placeholders::_2));

    // m_timer = m_nodehandle.create_wall_timer(2s, std::bind(&WorkCellStateMachine::update_q_location, this));

    // tf_buffer_ =
    //     std::make_unique<tf2_ros::Buffer>(m_nodehandle.get_clock());

    // tf_listener_ =
    //     std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // get_workcells_cli = m_nodehandle.create_client<spice_msgs::srv::GetRobotsByType>("/get_robots_by_type");

    // m_costmap_subscriber = m_nodehandle.create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/costmap/costmap",
    //     10,
    //     std::bind(&WorkCellStateMachine::global_costmap_cb, this, std::placeholders::_1));

    

    // m_timer = m_nodehandle.create_wall_timer(2s, std::bind(&WorkCellStateMachine::update_q_location, this));

    // tf_buffer_ =
    //     std::make_unique<tf2_ros::Buffer>(m_nodehandle.get_clock());

    // tf_listener_ =
    //     std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // get_workcells_cli = m_nodehandle.create_client<spice_msgs::srv::GetRobotsByType>("/get_robots_by_type");

    // m_costmap_subscriber = m_nodehandle.create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/costmap/costmap",
    //     10,
    //     std::bind(&WorkCellStateMachine::global_costmap_cb, this, std::placeholders::_1));

    

    auto qos_profile_TL = rmw_qos_profile_default;
    qos_profile_TL.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    m_state_transition_event_pub = m_nodehandle.create_publisher<spice_msgs::msg::RobotStateTransition>(
        m_work_cell_name + "/robot_state_transition_event",
        rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_TL.history, 10), qos_profile_TL)
    );
    m_tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_nodehandle);
    
    m_entry_transform.translation.x = -STEP_DISTANCE;
    m_exit_transform.translation.x = STEP_DISTANCE;
    
    m_queue_manager.initialize_points(3, m_transform);

    m_current_state = WORK_CELL_STATE::STARTUP;
    m_states = {
        std::make_shared<StartupState>(*this),
        std::make_shared<ReadyForRobotState>(*this),
        std::make_shared<RobotEnteringState>(*this),
        std::make_shared<ProcessingState>(*this),
        std::make_shared<RobotExitingState>(*this)
    };
    m_states[static_cast<int>(m_current_state)]->init();

    // polygon corners in world coordinates:
        // for A4:
    world_corners.push_back({2.4185, -3.6402}); // top mid
    world_corners.push_back({-1.7283, -3.5899}); // top right
    world_corners.push_back({-1.5848, 2.0436}); // bottom right
    world_corners.push_back({6.9987, 1.7771}); // bottom left
    world_corners.push_back({7.0078, -0.6922}); // mid left
    world_corners.push_back({2.6586, -0.8671}); // mid mid
    world_corners.push_back({2.4185, -3.6402}); // top mid again. For the sake of the check poly function

    //std::vector<float> world_corners_x({2.4185,-1.7283,-1.5848,6.9987,7.0078,2.6586});
    //std::vector<float> world_corners_y({-3.6402,-3.5899,-2.0436,1.7771,-0.6922,-0.8671});
    for(auto pair: world_corners){
        world_corners_x.push_back(pair.first);
        world_corners_y.push_back(pair.second);
    }
}

void WorkCellStateMachine::change_state(WORK_CELL_STATE new_state)
{
    if(new_state == m_current_state)
    {
        RCLCPP_WARN(m_nodehandle.get_logger(), "TRYING TO GO TO SAME STATE AS CURRENT STATE");
        return;
    }
    RCLCPP_INFO(m_nodehandle.get_logger(), "%s state transition to %d from %d", m_work_cell_name.c_str(),
            static_cast<int>(new_state), static_cast<int>(m_current_state));

    auto robot_state_transition_msg = std::make_unique<spice_msgs::msg::RobotStateTransition>();
    robot_state_transition_msg->old_state = internal_state_to_robot_state(m_current_state);
    robot_state_transition_msg->new_state = internal_state_to_robot_state(new_state);
    robot_state_transition_msg->id = get_work_cell_id();
    m_state_transition_event_pub->publish(std::move(robot_state_transition_msg));

    m_states[static_cast<int>(m_current_state)]->deinit();
    m_current_state = new_state;
    m_states[static_cast<int>(m_current_state)]->init();
}

tf2::Matrix3x3 q_to_mat(geometry_msgs::msg::Quaternion q)
{
    tf2::Quaternion q_tf2;
    q_tf2.setX(q.x); 
    q_tf2.setY(q.y); 
    q_tf2.setZ(q.z); 
    q_tf2.setW(q.w); 
    return tf2::Matrix3x3(q_tf2);
}

geometry_msgs::msg::Pose transform_to_map(geometry_msgs::msg::Transform& target_transform, geometry_msgs::msg::Transform& cell_transform)
{
    auto from_rot = q_to_mat(target_transform.rotation);
    auto to_rot = q_to_mat(cell_transform.rotation);

    tf2::Matrix3x3 from_to_rot_mat = from_rot * to_rot;
    tf2::Quaternion from_to_rot;
    from_to_rot_mat.getRotation(from_to_rot);

    tf2::Vector3 from_translation;
    from_translation.setX(target_transform.translation.x);
    from_translation.setY(target_transform.translation.y);
    from_translation.setZ(target_transform.translation.z);
    tf2::Vector3 to_translation;
    to_translation.setX(cell_transform.translation.x);
    to_translation.setY(cell_transform.translation.y);
    to_translation.setZ(cell_transform.translation.z);

    tf2::Vector3 from_to_trans = to_rot*from_translation + to_translation;

    geometry_msgs::msg::Pose map_pose;
    map_pose.position.x = from_to_trans.getX();
    map_pose.position.y = from_to_trans.getY();
    map_pose.position.z = from_to_trans.getZ();
    map_pose.orientation.x = from_to_rot.getX();
    map_pose.orientation.y = from_to_rot.getY();
    map_pose.orientation.z = from_to_rot.getZ();
    map_pose.orientation.w = from_to_rot.getW();
    return map_pose;
}

// bool WorkCellStateMachine::enqueue_robot(spice_msgs::srv::RegisterWork::Request::SharedPtr request)
// {
//     // TODO: add check that there is space in queue, if not, return false
//     // TODO: get queue pose here
//     carrier_robot robot(geometry_msgs::msg::Pose(), request->work, request->robot_id);
//     m_enqueued_robots.push_back(robot);
//     return true;
// }

void WorkCellStateMachine::on_register_work(
    const std::shared_ptr<spice_msgs::srv::RegisterWork::Request> request, 
    std::shared_ptr<spice_msgs::srv::RegisterWork::Response> response)
{
    RCLCPP_INFO(m_nodehandle.get_logger(), "On register work from %s", request->robot_id.id.c_str());
    // TODO: do we want to implement simulated checks for work compatibility, queue length etc?

    auto queue_point_opt = m_queue_manager.get_queue_point();
    if(!queue_point_opt)
    {
        RCLCPP_INFO(m_nodehandle.get_logger(), "Failed to register work due to no space in queue");
        response->work_is_enqueued = false;
        return;
    }

    QueuePoint* queue_point = queue_point_opt.value();
    carrier_robot robot(queue_point, request->work, request->robot_id);
    m_enqueued_robots.push_back(robot);
    response->work_is_enqueued = true;
    
    response->queue_pose.pose = transform_to_map(queue_point->transform, m_transform);
    response->queue_pose.header.frame_id = "map";
    response->queue_pose.header.stamp = m_nodehandle.get_clock()->now();

    response->entry_pose.pose = transform_to_map(m_entry_transform, m_transform);
    response->entry_pose.header.frame_id = "map";
    response->entry_pose.header.stamp = m_nodehandle.get_clock()->now();
    
    response->processing_pose.header.frame_id = "map";
    response->processing_pose.header.stamp = m_nodehandle.get_clock()->now();
    response->processing_pose.pose.position.x = m_transform.translation.x;
    response->processing_pose.pose.position.y = m_transform.translation.y;
    response->processing_pose.pose.position.z = m_transform.translation.z;
    response->processing_pose.pose.orientation.x = m_transform.rotation.x;
    response->processing_pose.pose.orientation.y = m_transform.rotation.y;
    response->processing_pose.pose.orientation.z = m_transform.rotation.z;
    response->processing_pose.pose.orientation.w = m_transform.rotation.w;
    
    response->exit_pose.pose = transform_to_map(m_exit_transform, m_transform);
    response->exit_pose.header.frame_id = "map";
    response->exit_pose.header.stamp = m_nodehandle.get_clock()->now();
    
    RCLCPP_INFO(m_nodehandle.get_logger(), "Enqueued robot: %s", request->robot_id.id.c_str());
}

void WorkCellStateMachine::on_robot_ready_in_queue(
    const std::shared_ptr<spice_msgs::srv::RobotReady::Request> request,
    std::shared_ptr<spice_msgs::srv::RobotReady::Response> response)
{
    for(auto& robot : m_enqueued_robots)
    {
        if(robot.robot_id.id == request->robot_id.id)
        {
            robot.ready_in_queue = true;
            response->success = true;
            return;
        }
    }
    RCLCPP_WARN(get_logger(), "On_robot_ready_in_queue got request for unknown robot");
    response->success = false; // return false if we do now know the robot
}

void WorkCellStateMachine::on_robot_ready_for_processing(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_states[static_cast<int>(m_current_state)]->on_robot_ready_for_processing(request, response);
}

std::optional<carrier_robot> WorkCellStateMachine::get_enqueued_robot()
{
    for(auto enqueued_robot = m_enqueued_robots.begin(); enqueued_robot != m_enqueued_robots.end(); enqueued_robot++)
    {
        if(enqueued_robot->ready_in_queue)
        {
            std::optional<carrier_robot> next_robot(*enqueued_robot);
            m_enqueued_robots.erase(enqueued_robot);
            return next_robot;
        }
    }
    return {};
}

void WorkCellStateMachine::activate_heartbeat()
{
    //RCLCPP_INFO(this->m_nodehandle.get_logger(), "%s activating heartbeat", m_work_cell_name.c_str());
    if (!m_heartbeat_client)
    {
        m_heartbeat_client = m_nodehandle.create_client<spice_msgs::srv::Heartbeat>("heartbeat");
    }
    if (!m_heartbeat_timer)
    {
        m_heartbeat_timer = rclcpp::create_timer(
            &m_nodehandle,
            m_nodehandle.get_clock(),
            rclcpp::Duration::from_seconds(5),
            [this]() -> void {
                if (!this->m_heartbeat_client->wait_for_service(1s))
                {
                    RCLCPP_WARN(this->m_nodehandle.get_logger(), "Timeout on heartbeat service");
                }
                auto request = std::make_shared<spice_msgs::srv::Heartbeat::Request>();
                request->id = get_work_cell_id();
                this->m_heartbeat_client->async_send_request(request,
                [this](rclcpp::Client<spice_msgs::srv::Heartbeat>::SharedFuture future) -> void{
                    auto response = future.get();
                    if(response->restart_robot)
                    {
                        RCLCPP_WARN(this->m_nodehandle.get_logger(), 
                            "%s was asked to restart, but is is not implemented", 
                            this->m_work_cell_name.c_str());
                    }
                });
            }
        );
    }
    m_heartbeat_timer->reset();
}

void WorkCellStateMachine::deactivate_heartbeat()
{
    if(m_heartbeat_timer)
    {
        m_heartbeat_timer->cancel();
    }
}

void WorkCellStateMachine::publish_transform()
{
    geometry_msgs::msg::TransformStamped t;
    // publish center of cell
    t.header.stamp = m_nodehandle.get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = get_work_cell_id().id;
    t.transform = m_transform;
    m_tf_static_broadcaster->sendTransform(t);

    // publish transform for entry to cell
    t.transform = m_entry_transform;
    t.header.frame_id = get_work_cell_id().id;
    t.child_frame_id = get_work_cell_id().id + "_entry";
    m_tf_static_broadcaster->sendTransform(t);

    // publish transform for exit of cell
    t.transform = m_exit_transform;
    t.header.frame_id = get_work_cell_id().id;
    t.child_frame_id = get_work_cell_id().id + "_exit";
    m_tf_static_broadcaster->sendTransform(t);

    // publish queue positions
    auto queue_transforms = m_queue_manager.get_queue_point_transforms();
    for (size_t i = 0; i < queue_transforms.size(); i++)
    {   
        t.transform = queue_transforms[i];
        t.header.frame_id = get_work_cell_id().id;
        t.child_frame_id = get_work_cell_id().id + "_q" + std::to_string(i);
        m_tf_static_broadcaster->sendTransform(t);
    }
}

spice_msgs::msg::RobotState WorkCellStateMachine::internal_state_to_robot_state(WORK_CELL_STATE state)
{
    spice_msgs::msg::RobotState robot_state;
    if(state == WORK_CELL_STATE::STARTUP)
    {
        robot_state.state = spice_msgs::msg::RobotState::STARTUP;
    }
    else if (state == WORK_CELL_STATE::READY_FOR_ROBOT ||
        state == WORK_CELL_STATE::ROBOT_ENTERING ||
        state == WORK_CELL_STATE::PROCESSING ||
        state == WORK_CELL_STATE::ROBOT_EXITING)
    {
        robot_state.state = spice_msgs::msg::RobotState::WC_READY_FOR_ROBOTS;
    }
    int num_queue_points = m_queue_manager.m_queue_points.size();
    int used_points = 0;
    for(auto it = m_queue_manager.m_queue_points.begin(); it != m_queue_manager.m_queue_points.end(); it++)
    {
        if(it->occupied)
            used_points++;
    }
    std::string info = ", " + std::to_string(used_points) + "/" + std::to_string(num_queue_points);
    robot_state.internal_state = WORK_CELL_STATE_NAMES[static_cast<uint8_t>(state)] + info;
    return robot_state;
}

spice_msgs::msg::Id WorkCellStateMachine::get_work_cell_id() 
{ 
    spice_msgs::msg::Id id;
    id.id = m_work_cell_name;
    id.robot_type.type = m_robot_type;
    return id; 
}

void WorkCellStateMachine::release_robot()
{
    if(m_current_robot_work)
    {
        m_queue_manager.free_queue_point(m_current_robot_work->queue_point);
        m_current_robot_work.release();
    }
    else
    {
        RCLCPP_WARN(m_nodehandle.get_logger(), "Tried to release robot, but did not have one");
    }
}

// void WorkCellStateMachine::global_costmap_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//     m_global_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*msg);
// }

// void WorkCellStateMachine::update_q_location(){
//     RCLCPP_INFO(m_nodehandle.get_logger(), "[debug] timer for updating q frames for %s",m_work_cell_name.c_str());

//     if (!get_workcells_cli->wait_for_service(1s))
//     {
//         RCLCPP_WARN(m_nodehandle.get_logger(), "Timeout on Swarm manager get_robots_by_type");
//         return;
//     }
//     auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
//     get_robots_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

//     using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

//     auto get_robots_cb = [this](ServiceResponseFuture future)
//     { workcell_list = future.get()->robots; };

//     auto futureResult = get_workcells_cli->async_send_request(get_robots_request, get_robots_cb);

//     if (!get_carriers_cli->wait_for_service(1s))
//     {
//         RCLCPP_WARN(m_nodehandle.get_logger(), "Timeout on Swarm manager get_robots_by_type");
//         return;
//     }
//     auto get_robots_request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
//     get_robots_request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

//     using ServiceResponseFuture = rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

//     auto get_robots_cb = [this](ServiceResponseFuture future)
//     { carrier_list = future.get()->robots; };

//     auto futureResult = get_workcells_cli->async_send_request(get_robots_request, get_robots_cb);
    
//     /*
//     float test_x = 0.4771;
//     float test_y = -0.1844;
//     auto temp = pnpoly(world_corners_x.size(), world_corners_x, world_corners_y, test_x, test_y);

//     for(auto x: world_corners_x){
//         RCLCPP_WARN(m_nodehandle.get_logger(), "debug: x: [%f]",x);
//     }
//     for(auto y: world_corners_y){
//         RCLCPP_ERROR(m_nodehandle.get_logger(), "debug: y: [%f]",y);
//     }

     
//     RCLCPP_ERROR(m_nodehandle.get_logger(), "test world x: [x:%f, y:%f]",world_corners_x[1],world_corners_x[2]);

//     for (auto i: world_corners)
//     {
//         for (size_t i = 0; i < world_corners.size(); i++)
//         {
            
//         }
//         for (auto i: world_corners)

        
        
//         pnpoly(world_corners[][2]);


//         unsigned int corner_x, corner_y;
//         if (!m_global_costmap->worldToMap(world_corners[i][1], world_corners[i][2], corner_x, corner_y))
//         {
//             RCLCPP_ERROR(m_nodehandle.get_logger(), "Could not convert world coordinates to map coordinates: [x:%f, y:%f]",world_corners[i][1],world_corners[i][2]);
//         }
//     }
//     */
    

// }

int WorkCellStateMachine::pnpoly(int nvert, std::vector<float> vertx, std::vector<float> verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}


// enum class WORK_CELL_STATE : uint8_t{
    // STARTUP = 0,
    // READY_FOR_ROBOT,
    // ROBOT_ENTERING,
    // PROCESSING,
    // ROBOT_EXITING,
    // NUM_STATES
// };