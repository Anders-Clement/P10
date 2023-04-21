#include "spice/work_cells/work_cell.hpp"
#include "rclcpp/create_timer.hpp"

using namespace std::chrono_literals;

StartupState::StartupState(WorkCellStateMachine& sm) : m_sm(sm)
{

}
void StartupState::init()
{
    m_register_work_cell_client = m_sm.m_nodehandle.create_client<spice_msgs::srv::RegisterRobot>("register_robot");
    m_timer = rclcpp::create_timer(
        &m_sm.m_nodehandle,
        m_sm.m_nodehandle.get_clock(),
        rclcpp::Duration::from_seconds(1),
        std::bind(&StartupState::try_register_robot, this)
    );
    m_sm.deactivate_heartbeat();
}
void StartupState::deinit()
{
    m_register_work_cell_client.reset();
    m_timer.reset();
}
void StartupState::try_register_robot()
{
    auto request = std::make_shared<spice_msgs::srv::RegisterRobot::Request>();
    request->id = m_sm.get_work_cell_id();
    m_register_work_cell_client->async_send_request(
        request,
        [this](rclcpp::Client<spice_msgs::srv::RegisterRobot>::SharedFuture future) -> void {
            rclcpp::Client<spice_msgs::srv::RegisterRobot>::SharedResponse response = future.get();
            if ( response->success)
            {
                // TODO: activate heartbeat timer here
                this->m_sm.activate_heartbeat();
                this->m_sm.publish_transform();
                this->m_sm.change_state(WORK_CELL_STATE::READY_FOR_ROBOT);
            }
        }
    );
}


ReadyForRobotState::ReadyForRobotState(WorkCellStateMachine& sm) : m_sm(sm)
{

}
void ReadyForRobotState::init() 
{
    m_timer = rclcpp::create_timer(
        &m_sm.m_nodehandle, 
        m_sm.m_nodehandle.get_clock(),
        rclcpp::Duration::from_seconds(.1),
        std::bind(&ReadyForRobotState::try_call_robot, this));
}
void ReadyForRobotState::deinit() 
{
    m_sm.m_call_robot_client.reset();
}
void ReadyForRobotState::try_call_robot()
{
    auto robot = m_sm.get_enqueued_robot();
    if (robot)
    {
        m_timer->cancel();
        m_sm.m_current_robot_work = std::make_unique<carrier_robot>(robot.value());
        std::string call_robot_client_topic_name = m_sm.m_current_robot_work->robot_id.id + "/call_robot";
        m_sm.m_call_robot_client = m_sm.m_nodehandle.create_client<std_srvs::srv::Trigger>(call_robot_client_topic_name);
        
        RCLCPP_INFO(m_sm.get_logger(), "Calling robot into cell: %s", robot.value().robot_id.id.c_str());
        call_robot_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        if (!m_sm.m_call_robot_client->wait_for_service(1s))
        {
            RCLCPP_WARN(m_sm.get_logger(), "Timeout waiting for service: %s, ignoring the robot", call_robot_client_topic_name.c_str());
            m_timer->reset();
            return;
        }
        m_sm.m_call_robot_client->async_send_request(
            call_robot_request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
                auto response = future.get();
                if (response->success)
                {
                    this->m_sm.change_state(WORK_CELL_STATE::ROBOT_ENTERING);
                }
                else
                {
                    this->m_sm.m_current_robot_work.reset();
                    m_timer->reset();
                    RCLCPP_WARN(m_sm.get_logger(), "Called robot, but got response false");
                }
            }
        ); 
    }
}


RobotEnteringState::RobotEnteringState(WorkCellStateMachine& sm) : m_sm(sm)
{

}
void RobotEnteringState::init()
{

}
void RobotEnteringState::deinit()
{

}

void RobotEnteringState::on_robot_ready_for_processing(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
    RCLCPP_INFO(m_sm.m_nodehandle.get_logger(), "robot is ready for processing");
    response->success = true;
    m_sm.change_state(WORK_CELL_STATE::PROCESSING);
}



ProcessingState::ProcessingState(WorkCellStateMachine& sm) : m_sm(sm)
{

}
void ProcessingState::init()
{
    std::string robot_done_processing_topic = m_sm.m_current_robot_work->robot_id.id + "/robot_done_processing";
    m_sm.m_done_processing_client = m_sm.m_nodehandle.create_client
        <std_srvs::srv::Trigger>(robot_done_processing_topic);

    m_processing_timer = rclcpp::create_timer(
        &m_sm.m_nodehandle, 
        m_sm.m_nodehandle.get_clock(), 
        rclcpp::Duration::from_seconds(m_sm.get_processing_time()),
        [this]() -> void {
                this->m_processing_timer->cancel();
                if (!this->m_sm.m_done_processing_client->wait_for_service(1s))
                {
                    RCLCPP_WARN(this->m_sm.get_logger(), "Timeout waiting for service %s, ignoring the robot",
                        (m_sm.m_current_robot_work->robot_id.id + "/robot_done_processing").c_str());
                    this->m_sm.change_state(WORK_CELL_STATE::READY_FOR_ROBOT);
                    return;
                }                
                this->m_sm.m_done_processing_client->async_send_request(
                    std::make_shared<std_srvs::srv::Trigger::Request>(),
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
                        this->m_sm.change_state(WORK_CELL_STATE::ROBOT_EXITING);
                    }
                );
            }
        );
}
void ProcessingState::deinit()
{   
    m_processing_timer.reset();
}


RobotExitingState::RobotExitingState(WorkCellStateMachine& sm) : m_sm(sm)
{

}
void RobotExitingState::init()
{
    // TODO: do we want to add check and ensure robot is out of the cell
    // or just call the next one immediately?
    m_timer = rclcpp::create_timer(
        &m_sm.m_nodehandle, 
        m_sm.m_nodehandle.get_clock(),
        rclcpp::Duration::from_seconds(.1),
        [this]() -> void {
            this->m_sm.change_state(WORK_CELL_STATE::READY_FOR_ROBOT);
        }
    );
}
void RobotExitingState::deinit()
{
    m_timer.reset();
}