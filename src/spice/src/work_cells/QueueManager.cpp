#include "spice/work_cells/queue_manager.hpp"

QueueManager::QueueManager() : m_queue_size(3)
{
    initialize_points(m_queue_size)
}

QueueManager::QueueManager(int num_points) : m_queue_size(num_points)
{
    initialize_points(m_queue_size);
}

void QueueManager::initialize_points(int num_points=3)
{
    
}

std::optional<QueuePoint> QueueManager::get_queue_point()
{

}