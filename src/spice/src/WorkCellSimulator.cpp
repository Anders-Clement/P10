#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "spice/work_cell_state_machine.hpp"

class PositionGenerator
{
public:
    PositionGenerator(uint32_t height, uint32_t width, float scale) :
    m_num_positions_generated(0), m_height(height), m_width(width), m_scale(scale) {}
    geometry_msgs::msg::Transform generate_position()
    {
        geometry_msgs::msg::Transform transform;
        uint32_t x_pos = m_num_positions_generated % m_width;
        transform.translation.x = m_scale * x_pos;
        transform.translation.y = m_scale * ((m_num_positions_generated / m_width) % m_height);
        transform.translation.z = 0; 
        m_num_positions_generated++;
        return transform;        
    }

private:
    uint32_t m_num_positions_generated;
    uint32_t m_height;
    uint32_t m_width;
    float m_scale;
};

class WorkCellSimulator : public rclcpp::Node
{
public:
    WorkCellSimulator() : Node("work_cell_simulator_node")
    {
        PositionGenerator generator(3, 3, 2);
        work_cell_state_machines = {
            std::make_shared<WorkCellStateMachine>(
                "back_cover_cell", 
                spice_msgs::msg::RobotType::WORK_CELL_BACK_COVER, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "drill_cell", spice_msgs::msg::RobotType::WORK_CELL_DRILL, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "fuses_cell", 
                spice_msgs::msg::RobotType::WORK_CELL_FUSES, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "lid_cell", 
                spice_msgs::msg::RobotType::WORK_CELL_TOP, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "back_cover_cell_2", 
                spice_msgs::msg::RobotType::WORK_CELL_BACK_COVER, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "drill_cell_2", spice_msgs::msg::RobotType::WORK_CELL_DRILL, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "fuses_cell_2", 
                spice_msgs::msg::RobotType::WORK_CELL_FUSES, 
                *this,
                generator.generate_position()),
            std::make_shared<WorkCellStateMachine>(
                "lid_cell_2", 
                spice_msgs::msg::RobotType::WORK_CELL_TOP, 
                *this,
                generator.generate_position())
        };
    }

private:
    std::vector<std::shared_ptr<WorkCellStateMachine>> work_cell_state_machines;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorkCellSimulator>());
    rclcpp::shutdown();
    return 0;
}