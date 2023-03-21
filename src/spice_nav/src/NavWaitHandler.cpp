#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"


using std::placeholders::_1;


class WaitCondition : public BT::ConditionNode
{
public:
    WaitCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          wait_topic_(prefix + "/Wait")
    {
        getInput("wait_topic", wait_topic_);

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        prefix = node_->declare_parameter<std::string>("prefix", getenv("ROBOT_NAMESPACE"));

        sub_getIfWait = node_->create_subscription<std_msgs::msg::Bool>(
            wait_topic_, 100, std::bind(&WaitCondition::cb_getIfWait, this, _1));
    }

    BT::NodeStatus tick()
    {
        if (do_wait)
        {
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
    }

    void cb_getIfWait(std_msgs::msg::Bool msg)
    {
        do_wait = msg.data;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("wait_topic", "The topic that will be subscribed to")};
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string prefix;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_getIfWait;
    std::string wait_topic_;
    bool do_wait = false;

};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<WaitCondition>("WaitCondition");
}