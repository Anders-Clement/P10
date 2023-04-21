#ifndef QUEUE_MANAGER_HPP
#define QUEUE_MANAGER_HPP

#include "spice/work_cells/work_cell_state_machine.hpp"


struct QueuePoint
{
    bool occupied = false;
    geometry_msgs::msg::Transform transform;

    QueuePoint(geometry_msgs::msg::Transform _transform) : transform(_transform) {};
};


class QueueManager
{
public:
    QueueManager();
    QueueManager(int num_points);
    std::optional<QueuePoint> get_queue_point();

private:
    void initialize_points(int num_points = 3);
    int m_queue_size;
    std::vector<QueuePoint> m_queue_points;
};


#endif //QUEUE_MANAGER_HPP