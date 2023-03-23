#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"


class WaitCondition : public BT::ConditionNode
{
public:
    WaitCondition() = delete;
    WaitCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf);



    BT::NodeStatus tick() override;



    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("wait_topic", std::string("nav_wait_condition"), "The topic that will be subscribed to")
        };
    }

private:

    void cb_getIfWait(std_msgs::msg::Bool msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_getIfWait;
    std::string wait_topic_;
    bool do_wait = false;
};