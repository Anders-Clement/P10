#include "spice/work_cells/queue_manager.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"
#include "tf2/LinearMath/Matrix3x3.h"


QueueManager::QueueManager(rclcpp::Node& nodehandle, std::string work_cell_name, WorkCellStateMachine* work_cell_state_machine) : m_nodehandle(nodehandle)
{
    m_queue_points_publisher = m_nodehandle.create_publisher<spice_msgs::msg::QueuePoints>(
        work_cell_name + "/queue_points", 10);
    m_work_cell_state_machine = work_cell_state_machine;
    m_work_cell_name = work_cell_name;

    auto qos_profile_TL = rmw_qos_profile_default;
    qos_profile_TL.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    m_enqueued_pub = m_nodehandle.create_publisher<spice_msgs::msg::QueueOccupancy>(
        "/workstations_occupancy",
        rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_TL.history, 10), qos_profile_TL)
    );
    
}

void QueueManager::initialize_points(int num_points, double time)
{
    m_queue_points.clear();

    geometry_msgs::msg::Transform q_transform = m_work_cell_state_machine->m_entry_transform;
    geometry_msgs::msg::Transform last_q_transform = q_transform;

    m_queue_points.emplace_back(q_transform, m_queue_id_counter++, time);
    // static queue positions, can be replaced with dynamic positions

    for (int i = 0; i < num_points/*-1to make num queue points fit */; i++)
    {
        q_transform.translation.x = -STEP_DISTANCE + (STEP_DISTANCE*i);
        q_transform.translation.y =  WORKCELL_RADIUS + ROBOT_RADIUS;
        float angle = atan2(last_q_transform.translation.y - q_transform.translation.y, last_q_transform.translation.x - q_transform.translation.x);
        tf2::Quaternion rotation;
        rotation.setRPY(0.0,0.0,angle);
        q_transform.rotation.x = rotation.getX(); 
        q_transform.rotation.y = rotation.getY(); 
        q_transform.rotation.z = rotation.getZ(); // rotate 180 deg cc
        q_transform.rotation.w = rotation.getW();
        m_queue_points.emplace_back(q_transform, m_queue_id_counter++, time);
        last_q_transform = q_transform;
    }
    publish_queue_points();



    spice_msgs::msg::QueueOccupancy msg;
    msg.id.id = m_work_cell_name;
    msg.enqueued = 0;
    m_enqueued_pub->publish(msg);

    
}

std::optional<QueuePoint*> QueueManager::get_queue_point()
{
    for(auto it = m_queue_points.begin(); it != m_queue_points.end(); it++)
    {
        if(!it->occupied)
        {
            it->occupied = true;

            if (m_occupied_queue_counter < m_queue_id_counter)
            {
                m_occupied_queue_counter++;
                spice_msgs::msg::QueueOccupancy msg;
                msg.id.id = m_work_cell_name;
                msg.enqueued = m_occupied_queue_counter;
                m_enqueued_pub->publish(msg);
            }
            else{
                RCLCPP_ERROR(m_nodehandle.get_logger(), "Error in keeping track of occupied queues");
            }

            return {&*it};
        }
    }
    return {};
}

void QueueManager::free_queue_point(QueuePoint* queuepoint)
{
    for(auto it = m_queue_points.begin(); it != m_queue_points.end(); it++)
    {
        if(it->id == queuepoint->id)
        {   
            it->occupied = false;

            if (m_occupied_queue_counter != 0 && m_occupied_queue_counter <= m_queue_id_counter)
            {
                m_occupied_queue_counter--;
                spice_msgs::msg::QueueOccupancy msg;
                msg.id.id = m_work_cell_name;
                msg.enqueued = m_occupied_queue_counter;
                m_enqueued_pub->publish(msg);
            }
            else{
                RCLCPP_ERROR(m_nodehandle.get_logger(), "Error in keeping track of occupied queues");
            }
            
            RCLCPP_WARN(m_nodehandle.get_logger(), "freeing queue point");
            it->queued_robot = spice_msgs::msg::Id{};
            break;
        }
    }
    fill_queue_points();
}

std::vector<geometry_msgs::msg::Transform> QueueManager::get_queue_point_transforms()
{
    std::vector<geometry_msgs::msg::Transform> transforms;

    for(auto& queue_point : m_queue_points)
        transforms.push_back(queue_point.transform);
    return transforms;
}

void QueueManager::publish_queue_points()
{
    spice_msgs::msg::QueuePoints msg;
    for(auto& queue_point : m_queue_points)
    {
        spice_msgs::msg::QueuePoint queue_point_msg;

        geometry_msgs::msg::Pose queue_pose_stamped = m_work_cell_state_machine->transform_to_map(queue_point.transform, m_work_cell_state_machine->m_transform);
        queue_point_msg.queue_transform.translation.x = queue_pose_stamped.position.x;
        queue_point_msg.queue_transform.translation.y = queue_pose_stamped.position.y; 
        queue_point_msg.queue_transform.translation.z = queue_pose_stamped.position.z;
        queue_point_msg.queue_transform.rotation.x =  queue_pose_stamped.orientation.x;
        queue_point_msg.queue_transform.rotation.y =  queue_pose_stamped.orientation.y;
        queue_point_msg.queue_transform.rotation.z =  queue_pose_stamped.orientation.z;
        queue_point_msg.queue_transform.rotation.w =  queue_pose_stamped.orientation.w;

        queue_point_msg.queue_id = queue_point.id;
        queue_point_msg.queue_robot_id = queue_point.queued_robot;
        msg.queue_points.push_back(queue_point_msg);
    }
    m_queue_points_publisher->publish(msg);
}


void QueueManager::fill_queue_points(){
    for(auto queue_point_empty = m_queue_points.begin(); queue_point_empty != m_queue_points.end(); queue_point_empty++){
        if(!queue_point_empty->occupied){
            RCLCPP_WARN(m_nodehandle.get_logger(), "free queue point is %d", queue_point_empty->id);
            auto queue_point_occ = queue_point_empty;
            for(queue_point_occ; queue_point_occ != m_queue_points.end(); queue_point_occ++){
                if(queue_point_occ->occupied){
                    RCLCPP_WARN(m_nodehandle.get_logger(), "occ queue point is %d", queue_point_occ->id);
                    queue_point_empty->queued_robot = queue_point_occ->queued_robot;
                    queue_point_empty->occupied = true;
                    queue_point_occ->occupied = false;
                    queue_point_occ->queued_robot = spice_msgs::msg::Id{};
                    RCLCPP_WARN(m_nodehandle.get_logger(), "occ queue point has robot: %s and empty quueue point has robot %s", queue_point_occ->queued_robot.id.c_str(), queue_point_empty->queued_robot.id.c_str());

                    break;
                }
            }
        }
    }
    
    //update enqueued robot list
    for(auto enqueued_robot = m_work_cell_state_machine->m_enqueued_robots.begin(); enqueued_robot != m_work_cell_state_machine->m_enqueued_robots.end(); enqueued_robot++){
        for(auto queue_point = m_queue_points.begin(); queue_point != m_queue_points.end(); queue_point++){
            if(enqueued_robot->robot_id.id == queue_point->queued_robot.id){
                enqueued_robot->queue_point = &*queue_point;
                break;
            }
        }
    }
    publish_queue_points();
}