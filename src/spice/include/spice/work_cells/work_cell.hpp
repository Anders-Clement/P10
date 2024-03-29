#ifndef WORK_CELL_HPP
#define WORK_CELL_HPP

#include <string>
#include <rclcpp/node.hpp>
#include <array>
#include "spice_msgs/msg/task.hpp"
#include "spice_msgs/srv/register_robot.hpp"
#include "spice/work_cells/work_cell_state_machine.hpp"

class WorkCellStateMachine;

class WorkCellState
{
public:
    WorkCellState(){};
    virtual void init() {};
    virtual void deinit() {};
    virtual void on_robot_ready_for_processing(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        response->success = false;
    }
    virtual void on_robot_exited(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = false;
    }
};

class StartupState : public WorkCellState
{
public:
    StartupState(WorkCellStateMachine& sm);
    void init() override;
    void deinit() override;
private:
    void try_register_robot();
    rclcpp::Client<spice_msgs::srv::RegisterRobot>::SharedPtr m_register_work_cell_client;
    WorkCellStateMachine& m_sm;
    rclcpp::TimerBase::SharedPtr m_timer;
};

class ReadyForRobotState : public WorkCellState
{
public:
    ReadyForRobotState(WorkCellStateMachine& sm);
    void init() override;
    void deinit() override;
    WorkCellStateMachine& m_sm;
private:
    void try_call_robot();
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<std_srvs::srv::Trigger::Request> call_robot_request;
};

class RobotEnteringState : public WorkCellState
{
public:
    RobotEnteringState(WorkCellStateMachine& sm);
    void init() override;
    void deinit() override;
    void on_robot_ready_for_processing(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
private:
    WorkCellStateMachine& m_sm;
};

class ProcessingState : public WorkCellState
{
public:
    ProcessingState(WorkCellStateMachine& sm);
    void init() override;
    void deinit() override;
    WorkCellStateMachine& m_sm;
private:
    rclcpp::TimerBase::SharedPtr m_processing_timer;
};

class RobotExitingState : public WorkCellState
{
public:
    RobotExitingState(WorkCellStateMachine& sm);
    void init() override;
    void deinit() override;
    void on_robot_exited(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
private:
    WorkCellStateMachine& m_sm;
};


#endif