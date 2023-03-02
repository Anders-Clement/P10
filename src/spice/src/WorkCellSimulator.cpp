#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "spice/work_cell.hpp"

class WorkCellSimulator : public rclcpp::Node
{
public:
    WorkCellSimulator() : Node("work_cell_simulator_node")
    {
        work_cell_state_machines = {
            std::make_shared<WorkCellStateMachine>("back_cover_cell", WORK_CELL_TYPE::BACK, *this),
            std::make_shared<WorkCellStateMachine>("drill_cell", WORK_CELL_TYPE::DRILL, *this),
            std::make_shared<WorkCellStateMachine>("fuses_cell", WORK_CELL_TYPE::FUSES, *this),
            std::make_shared<WorkCellStateMachine>("lid_cell", WORK_CELL_TYPE::LID, *this)
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