#include "spice/work_cells/queue_manager.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"

QueueManager::QueueManager(rclcpp::Node& nodehandle, std::string work_cell_name) : m_nodehandle(nodehandle)
{
    m_queue_points_publisher = m_nodehandle.create_publisher<spice_msgs::msg::QueuePoints>(
        work_cell_name + "/queue_points", 10);
}

void QueueManager::initialize_points(int num_points, double time)
{
    m_queue_points.clear();
    // static queue positions, can be replaced with dynamic positions
    for (int i = 0; i < num_points; i++)
    {
        geometry_msgs::msg::Transform q_transform;
        q_transform.translation.x = -STEP_DISTANCE + (STEP_DISTANCE*i);
        q_transform.translation.y =  WORKCELL_RADIUS + ROBOT_RADIUS;
        q_transform.rotation.z = 0.7071; // rotate 90 deg cc
        q_transform.rotation.w = 0.7071;
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
            return;
        }
    }   
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
        queue_point_msg.queue_transform = queue_point.transform; 
        queue_point_msg.queue_id = queue_point.id;
        msg.queue_points.push_back(queue_point_msg);
    }
    m_queue_points_publisher->publish(msg);
}