#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"

class PositionGenerator
{
public:
    PositionGenerator(uint32_t height, uint32_t width, float scale, std::string map_name) :
    m_num_positions_generated(0), m_height(height), m_width(width), m_scale(scale), m_map_name(map_name) {}
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

    geometry_msgs::msg::Transform work_cell_locations(){

        geometry_msgs::msg::Transform poses[4];
        if(m_map_name == "A4.yaml" || m_map_name == "A4_legless.yaml") //A4
        {  
            poses[0].rotation.w = 0.709132;
            poses[0].rotation.x = 0.0;
            poses[0].rotation.y = 0.0;
            poses[0].rotation.z = 0.7050776;
            poses[0].translation.x = 6.27535;
            poses[0].translation.y = 0.518605;
            poses[0].translation.z = 0.0;
            
            poses[1].rotation.w = 0.723702;
            poses[1].rotation.x = 0;
            poses[1].rotation.y = 0;
            poses[1].rotation.z = 0.690113;
            poses[1].translation.x = 4.10358;
            poses[1].translation.y = 0.446661;
            poses[1].translation.z = 0;

            poses[2].rotation.w = 1;
            poses[2].rotation.x = 0;
            poses[2].rotation.y = 0;
            poses[2].rotation.z = 0.000737653;
            poses[2].translation.x = 0.358586;
            poses[2].translation.y = -1.50395;
            poses[2].translation.z = 0;

            poses[3].rotation.w = 0.999716;
            poses[3].rotation.x = 0;
            poses[3].rotation.y = 0;
            poses[3].rotation.z = 0.0238133;
            poses[3].translation.x = 0.383676;
            poses[3].translation.y = 0.486885;
            poses[3].translation.z = 0;
        }
        else if(m_map_name == "low_res/A4.yaml")
        {
            poses[0].rotation.w = 1.0; //0.709132;
            poses[0].rotation.x = 0.0;
            poses[0].rotation.y = 0.0;
            poses[0].rotation.z = 0.0; //0.7050776;
            poses[0].translation.x = 5.0 +2.0;
            poses[0].translation.y = 0.0 +4.0;
            poses[0].translation.z = 0.0;
            
            poses[1].rotation.w = 1.0; //0.723702;
            poses[1].rotation.x = 0;
            poses[1].rotation.y = 0;
            poses[1].rotation.z = 0.0; //0.690113;
            poses[1].translation.x = 3.0 +2.0;
            poses[1].translation.y = 0.0 +4.0;
            poses[1].translation.z = 0;

            poses[2].rotation.w = 1;
            poses[2].rotation.x = 0;
            poses[2].rotation.y = 0;
            poses[2].rotation.z = 0.000737653;
            poses[2].translation.x = 0.5 +2.0;
            poses[2].translation.y = -1.5 +4.0;
            poses[2].translation.z = 0;

            poses[3].rotation.w = 0.999716;
            poses[3].rotation.x = 0;
            poses[3].rotation.y = 0;
            poses[3].rotation.z = 0.0238133;
            poses[3].translation.x = 0.5 +2.0;
            poses[3].translation.y = 0.0 +4.0;
            poses[3].translation.z = 0;
        }
        // // C4 POSES
        else if(m_map_name == "C4.yaml")
        {
            poses[0].rotation.w = 0.999997;
            poses[0].rotation.x = 0.0 ;
            poses[0].rotation.y = 0.0;
            poses[0].rotation.z = 0.00250493;
            poses[0].translation.x = 15.9; //18.7613;
            poses[0].translation.y = 7.0; //7.78589;
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

            /*poses[4].rotation.w = 0.999997;
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
            poses[5].translation.z = 0;*/
        }
        else if(m_map_name == "low_res/C4.yaml")
        {
            poses[0].rotation.w = 1.0;
            poses[0].rotation.x =0.0 ;
            poses[0].rotation.y = 0.0;
            poses[0].rotation.z = 0.0;
            poses[0].translation.x = 2.0;
            poses[0].translation.y = 12.5;
            poses[0].translation.z = 0.0;
            
            poses[1].rotation.w = 1.0;
            poses[1].rotation.x =0.0 ;
            poses[1].rotation.y = 0.0;
            poses[1].rotation.z = 0.0;
            poses[1].translation.x = 2.0;
            poses[1].translation.y = 10.0;
            poses[1].translation.z = 0.0;

            poses[2].rotation.w = 1.0;
            poses[2].rotation.x =0.0 ;
            poses[2].rotation.y = 0.0;
            poses[2].rotation.z = 0.0;
            poses[2].translation.x = 2.0;
            poses[2].translation.y = 5.0;
            poses[2].translation.z = 0.0;

            poses[3].rotation.w = 1.0;
            poses[3].rotation.x =0.0 ;
            poses[3].rotation.y = 0.0;
            poses[3].rotation.z = 0.0;
            poses[3].translation.x = 2.0;
            poses[3].translation.y = 7.5;
            poses[3].translation.z = 0.0;
        }
        // CANTEEN POSES
        else if(m_map_name == "CANTEEN.yaml")
        {
            poses[0].rotation.w = 1.0;
            poses[0].rotation.x =0.0 ;
            poses[0].rotation.y = 0.0;
            poses[0].rotation.z = 0.0;
            poses[0].translation.x = 4.5;
            poses[0].translation.y = -8.0;
            poses[0].translation.z = 0.0;
            
            poses[1].rotation.w = 1.0;
            poses[1].rotation.x =0.0 ;
            poses[1].rotation.y = 0.0;
            poses[1].rotation.z = 0.0;
            poses[1].translation.x = 7.0;
            poses[1].translation.y = -9.0;
            poses[1].translation.z = 0.0;

            poses[2].rotation.w = 1.0;
            poses[2].rotation.x =0.0 ;
            poses[2].rotation.y = 0.0;
            poses[2].rotation.z = 0.0;
            poses[2].translation.x = 5.0;
            poses[2].translation.y = -5.0;
            poses[2].translation.z = 0.0;

            poses[3].rotation.w = 1.0;
            poses[3].rotation.x =0.0 ;
            poses[3].rotation.y = 0.0;
            poses[3].rotation.z = 0.0;
            poses[3].translation.x = 7.5;
            poses[3].translation.y = -5.5;
            poses[3].translation.z = 0.0;

            /*poses[4].rotation.w = 1.0;
            poses[4].rotation.x =0.0 ;
            poses[4].rotation.y = 0.0;
            poses[4].rotation.z = 0.0;
            poses[4].translation.x = 5.5;
            poses[4].translation.y = -2.0;
            poses[4].translation.z = 0.0;

            poses[5].rotation.w = 1.0;
            poses[5].rotation.x =0.0 ;
            poses[5].rotation.y = 0.0;
            poses[5].rotation.z = 0.0;
            poses[5].translation.x = 8;
            poses[5].translation.y = -2;
            poses[5].translation.z = 0.0; */
        }
        

        return poses[m_num_c4_positions++];      
  }

private:
    uint32_t m_num_positions_generated;
    uint8_t m_num_c4_positions = 0;
    uint32_t m_height;
    uint32_t m_width;
    float m_scale;
    std::string m_map_name;
};

class WorkCellSimulator : public rclcpp::Node
{
public:
    WorkCellSimulator() : Node("work_cell_simulator_node")
    {
        try
        {
            declare_parameter("work_cell_rep_slope", 0.05);
            declare_parameter("carrier_bot_rep_slope", 0.1);
            declare_parameter("wall_rep_slope", 0.1);
            declare_parameter("plan_rep_slope", 0.1);
            declare_parameter("queue_rep_slope", 0.1);
            declare_parameter("work_cell_att_slope", 0.05);
            declare_parameter("queue_att_slope", 0.0);
            declare_parameter("min_move_dist", 5);
            declare_parameter("q_max_vel", 0.33);
            declare_parameter("map", "");
        }
        catch(rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
        {
            //RCLCPP_WARN(get_logger(), "[debug] params already declared?");
        }
        MAP_NAME = get_parameter("map").get_parameter_value().get<std::string>();
        if(MAP_NAME == "")
        {
            RCLCPP_WARN(get_logger(), "[WorkCellSimulator] Did not get a map");
        }
        
        //std::string work_cell_name, rclcpp::Node& node_handle, 
        //geometry_msgs::msg::Transform transform, spice_msgs::msg::RobotType::_type_type robot_type
        PositionGenerator generator(3, 3, 2, MAP_NAME);
        work_cell_state_machines = {
            std::make_shared<WorkCellStateMachine>(
                "fuses_cell", 
                *this,
                generator.work_cell_locations(), 
                spice_msgs::msg::RobotType::WORK_CELL_FUSES),
            std::make_shared<WorkCellStateMachine>(
                "drill_cell", 
                *this,
                generator.work_cell_locations(),
                spice_msgs::msg::RobotType::WORK_CELL_DRILL),
            std::make_shared<WorkCellStateMachine>(
                "lid_cell", 
                *this,
                generator.work_cell_locations(), 
                spice_msgs::msg::RobotType::WORK_CELL_TOP),
            std::make_shared<WorkCellStateMachine>(
                "back_cover_cell", 
                *this,
                generator.work_cell_locations(),
                spice_msgs::msg::RobotType::WORK_CELL_BACK_COVER)
        };
    }
    std::string MAP_NAME;
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