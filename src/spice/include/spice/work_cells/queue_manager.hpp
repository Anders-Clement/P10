#ifndef QUEUE_MANAGER_HPP
#define QUEUE_MANAGER_HPP

#include <list>
#include <vector>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "spice_msgs/msg/id.hpp"
#include "spice_msgs/msg/queue_points.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"


struct QueuePoint
{
    geometry_msgs::msg::Transform transform;
    int id;
    spice_msgs::msg::Id queued_robot;
    bool occupied = false;
    double lastTime;

    QueuePoint(geometry_msgs::msg::Transform _transform, int _id, double _time) : transform(_transform), id(_id), lastTime(_time) {};
};
class WorkCellStateMachine;
class QueueManager
{
public:
    QueueManager() = delete;
    QueueManager(rclcpp::Node& nodehandle, std::string work_cell_name, WorkCellStateMachine* work_cell_state_machine);
    void initialize_points(int num_points, double time);
    std::optional<QueuePoint*> get_queue_point();
    void free_queue_point(QueuePoint* queuepoint);
    std::vector<geometry_msgs::msg::Transform> get_queue_point_transforms();
    void publish_queue_points();
    void fill_queue_points();

    std::list<QueuePoint> m_queue_points;
    WorkCellStateMachine* m_work_cell_state_machine;
private:
    rclcpp::Node& m_nodehandle;
    rclcpp::Publisher<spice_msgs::msg::QueuePoints>::SharedPtr m_queue_points_publisher;
    unsigned int m_queue_id_counter = 0;
};

#endif //QUEUE_MANAGER_HPP