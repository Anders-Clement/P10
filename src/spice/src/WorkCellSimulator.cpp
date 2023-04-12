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

    geometry_msgs::msg::Transform C4Locations(){

        geometry_msgs::msg::Transform poses[6];
        // C4 POSES
        poses[0].rotation.w = 0.999997;
        poses[0].rotation.x = 0.0 ;
        poses[0].rotation.y = 0.0;
        poses[0].rotation.z = 0.00250493;
        poses[0].translation.x = 18.7613;
        poses[0].translation.y = 7.78589;
        poses[0].translation.z = 0.0;
        
        poses[1].rotation.w = 0.999997;
        poses[1].rotation.x = 0;
        poses[1].rotation.y = 0;
        poses[1].rotation.z = 0.00250493;
        poses[1].translation.x = 15.9208;
        poses[1].translation.y = 5.6733;
        poses[1].translation.z = 0;

        poses[2].rotation.w = 0.999997;
        poses[2].rotation.x = 0;
        poses[2].rotation.y = 0;
        poses[2].rotation.z = 0.00250493;
        poses[2].translation.x = 15.9608;
        poses[2].translation.y = 9.71498;
        poses[2].translation.z = 0;

        poses[3].rotation.w = 0.999997;
        poses[3].rotation.x = 0;
        poses[3].rotation.y = 0;
        poses[3].rotation.z = 0.00250493;
        poses[3].translation.x = 16.0792;
        poses[3].translation.y = 12.8173;
        poses[3].translation.z = 0;

        poses[4].rotation.w = 0.999997;
        poses[4].rotation.x = 0;
        poses[4].rotation.y = 0;
        poses[4].rotation.z = 0.00250493;
        poses[4].translation.x = 16.229;
        poses[4].translation.y = 17.366;
        poses[4].translation.z = 0;

        poses[5].rotation.w = 1;
        poses[5].rotation.x = 0;
        poses[5].rotation.y = 0;
        poses[5].rotation.z = 0;
        poses[5].translation.x = 16.6054;
        poses[5].translation.y = 19.8698;
        poses[5].translation.z = 0;
        // CANTEEN POSES
        // poses[0].rotation.w = 1.0;
        // poses[0].rotation.x =0.0 ;
        // poses[0].rotation.y = 0.0;
        // poses[0].rotation.z = 0.0;
        // poses[0].translation.x = 4.5;
        // poses[0].translation.y = -8.0;
        // poses[0].translation.z = 0.0;
        
        // poses[1].rotation.w = 1.0;
        // poses[1].rotation.x =0.0 ;
        // poses[1].rotation.y = 0.0;
        // poses[1].rotation.z = 0.0;
        // poses[1].translation.x = 7.0;
        // poses[1].translation.y = -9.0;
        // poses[1].translation.z = 0.0;

        // poses[2].rotation.w = 1.0;
        // poses[2].rotation.x =0.0 ;
        // poses[2].rotation.y = 0.0;
        // poses[2].rotation.z = 0.0;
        // poses[2].translation.x = 5.0;
        // poses[2].translation.y = -5.0;
        // poses[2].translation.z = 0.0;

        // poses[3].rotation.w = 1.0;
        // poses[3].rotation.x =0.0 ;
        // poses[3].rotation.y = 0.0;
        // poses[3].rotation.z = 0.0;
        // poses[3].translation.x = 7.5;
        // poses[3].translation.y = -5.5;
        // poses[3].translation.z = 0.0;

        // poses[4].rotation.w = 1.0;
        // poses[4].rotation.x =0.0 ;
        // poses[4].rotation.y = 0.0;
        // poses[4].rotation.z = 0.0;
        // poses[4].translation.x = 5.5;
        // poses[4].translation.y = -2.0;
        // poses[4].translation.z = 0.0;

        // poses[5].rotation.w = 1.0;
        // poses[5].rotation.x =0.0 ;
        // poses[5].rotation.y = 0.0;
        // poses[5].rotation.z = 0.0;
        // poses[5].translation.x = 8;
        // poses[5].translation.y = -2;
        // poses[5].translation.z = 0.0;

        m_num_c4_positions++;


        return poses[m_num_c4_positions];      
  }

private:
    uint32_t m_num_positions_generated;
    uint8_t m_num_c4_positions = 0;
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
                generator.C4Locations()),
            std::make_shared<WorkCellStateMachine>(
                "drill_cell", spice_msgs::msg::RobotType::WORK_CELL_DRILL, 
                *this,
                generator.C4Locations()),
            std::make_shared<WorkCellStateMachine>(
                "lid_cell", 
                spice_msgs::msg::RobotType::WORK_CELL_TOP, 
                *this,
                generator.C4Locations()),
            std::make_shared<WorkCellStateMachine>(
                "fuses_cell", 
                spice_msgs::msg::RobotType::WORK_CELL_FUSES, 
                *this,
                generator.C4Locations()),
            std::make_shared<WorkCellStateMachine>(
                "back_cover_cell_2", 
                spice_msgs::msg::RobotType::WORK_CELL_BACK_COVER, 
                *this,
                generator.C4Locations()),
            
            /*std::make_shared<WorkCellStateMachine>(
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
                generator.generate_position())*/
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