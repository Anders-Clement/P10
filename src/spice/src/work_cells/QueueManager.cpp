#include "spice/work_cells/queue_manager.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"
#include "tf2/LinearMath/Matrix3x3.h"


QueueManager::QueueManager(rclcpp::Node& nodehandle, std::string work_cell_name, WorkCellStateMachine* work_cell_state_machine) : m_nodehandle(nodehandle)
{
    m_queue_points_publisher = m_nodehandle.create_publisher<spice_msgs::msg::QueuePoints>(
        work_cell_name + "/queue_points", 10);
    m_work_cell_state_machine = work_cell_state_machine;
    
}

void QueueManager::initialize_points(int num_points, double time)
{
    m_queue_points.clear();
    geometry_msgs::msg::Transform q_transform = m_work_cell_state_machine->m_entry_transform;
    //m_queue_points.emplace_back(q_transform, m_queue_id_counter++, time);
    // static queue positions, can be replaced with dynamic positions
    for (int i = 0; i < num_points/*-1to make num queue points fit */; i++)
    {
        q_transform.translation.x = -STEP_DISTANCE + (STEP_DISTANCE*i);
        q_transform.translation.y =  WORKCELL_RADIUS + ROBOT_RADIUS;
        q_transform.rotation.z = 1; // rotate 180 deg cc
        q_transform.rotation.w = 0;
        m_queue_points.emplace_back(q_transform, m_queue_id_counter++, time);
    }
    publish_queue_points();
}

std::optional<QueuePoint*> QueueManager::get_queue_point()
{
    for(auto it = m_queue_points.begin(); it != m_queue_points.end(); it++)
    {
        if(!it->occupied)
        {
            it->occupied = true;
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
            it->queued_robot = spice_msgs::msg::Id{};
            return;
        }
    }
    fill_queue_points();
    publish_queue_points();
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
        if(queue_point_empty->occupied == false){
            auto queue_point_occ = queue_point_empty++;
            for(queue_point_occ; queue_point_occ != m_queue_points.end(); queue_point_occ++){
                if(queue_point_occ->occupied = true){
                    queue_point_empty->queued_robot = queue_point_occ->queued_robot;
                    queue_point_empty->occupied = true;
                    queue_point_occ->occupied=false;
                    spice_msgs::msg::Id reset;
                    queue_point_occ->queued_robot = reset;
                    break;
                }
            }
            break;
        }
    }
    //update enqueued robot list
    for(auto enqueued_robot = m_work_cell_state_machine->m_enqueued_robots.begin(); enqueued_robot != m_work_cell_state_machine->m_enqueued_robots.end(); enqueued_robot++){
        for(auto queue_point = m_queue_points.begin(); queue_point != m_queue_points.end(); queue_point++){
            if(enqueued_robot->robot_id.id == queue_point->queued_robot.id){
                enqueued_robot->queue_point = &*queue_point;
            }
        }
    }
    publish_queue_points();
}