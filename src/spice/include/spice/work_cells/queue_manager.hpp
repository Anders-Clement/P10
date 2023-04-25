#ifndef QUEUE_MANAGER_HPP
#define QUEUE_MANAGER_HPP

#include <list>
#include <vector>
#include <optional>
#include "geometry_msgs/msg/transform.hpp"

struct QueuePoint
{
    geometry_msgs::msg::Transform transform;
    int id;
    bool occupied = false;

    QueuePoint(geometry_msgs::msg::Transform _transform, int _id) : transform(_transform), id(_id) {};
};

class QueueManager
{
public:
    void initialize_points(int num_points, geometry_msgs::msg::Transform work_cell_transform);
    std::optional<QueuePoint*> get_queue_point();
    void free_queue_point(QueuePoint* queuepoint);
    std::vector<geometry_msgs::msg::Transform> get_queue_point_transforms();

    unsigned int m_queue_id_counter = 0;
    std::list<QueuePoint> m_queue_points;
};

#endif //QUEUE_MANAGER_HPP