#include "spice_nav/NavWaitConditionNode.hpp"

using std::placeholders::_1;

    WaitCondition::WaitCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        wait_topic_("nav_wait_condition")
    {
        getInput("wait_topic", wait_topic_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        // sub_getIfWait = node_->create_subscription<std_msgs::msg::Bool>(wait_topic_, 100, std::bind(&WaitCondition::cb_getIfWait, this, _1));

        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        sub_getIfWait = node_->create_subscription<std_msgs::msg::Bool>(
            wait_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&WaitCondition::cb_getIfWait, this, std::placeholders::_1),
            sub_option
        );
    }


    BT::NodeStatus WaitCondition::tick()
    {
        callback_group_executor_.spin_some();
        RCLCPP_WARN(node_->get_logger(), "WaitCondition: do_wait is (%d)", do_wait);
        if (do_wait)
        {
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
    }


    void WaitCondition::cb_getIfWait(std_msgs::msg::Bool msg)
    {
        RCLCPP_WARN(node_->get_logger(), "WaitCondition: cb_getIfWait: msg.data is (%d)", msg.data);
        do_wait = msg.data;
    }


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<WaitCondition>("WaitCondition");
}