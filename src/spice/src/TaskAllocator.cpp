#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <time.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spice_msgs/srv/get_ready_robots.hpp"
#include "spice_msgs/msg/robot.hpp"
#include "spice_msgs/srv/robot_task.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spice_msgs/msg/task.hpp"
#include "spice_msgs/msg/layer.hpp"
#include "spice_msgs/msg/node.hpp"
#include "spice_msgs/msg/work.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Node
{
public:
    Node(uint8_t id_, std::string work_type_, std::vector<Node *> children_)
    {
        this->node_msg.id = id_;
        this->node_msg.work.type = work_type_;
        work_type = work_type_;
        children = children_;
    }

    bool is_leaf() { return this->node_msg.children_id.size() == 0; }
    int8_t get_id() { return this->node_msg.id; }

    spice_msgs::msg::Node node_msg;
    int8_t id;
    std::vector<Node *> children;
    std::string work_type;

private:
};

class handmadeTrees
{
public:
    handmadeTrees(int8_t type = 0)
    {
        this->typeOfTree = type;
        spice_msgs::msg::Task task;

        if (type == 1) // type h & s
        {
            Node node0(0, "l", {});
            Node node1(1, "l", {});
            Node node2(2, "s", {&node0});
            Node node3(3, "h", {&node1});
            Node node4(4, "h", {&node2});
            Node node5(5, "s", {&node3});
            Node node6(6, "b", {&node4, &node5});
            this->root_node = &node6;
            this->nodes.push_back(node6);
            this->nodes.push_back(node5);
            this->nodes.push_back(node4);
            this->nodes.push_back(node3);
            this->nodes.push_back(node2);
            this->nodes.push_back(node1);
            this->nodes.push_back(node0);
        }
        else if (type == 2) // type h
        {
            Node node0(0, "l", {});
            Node node1(1, "h", {&node0});
            Node node2(2, "b", {&node1});
            this->root_node = &node2;
            this->nodes.push_back(node2);
            this->nodes.push_back(node1);
            this->nodes.push_back(node0);
        }
        else if (type == 3) // type s
        {
            Node node0(0, "l", {});
            Node node1(1, "s", {&node0});
            Node node2(2, "b", {&node1});
            this->root_node = &node2;
            this->nodes.push_back(node2);
            this->nodes.push_back(node1);
            this->nodes.push_back(node0);
        }
        else if (type == 4) // type l
        {
            Node node0(0, "l", {});
            Node node1(1, "b", {&node0});
            this->root_node = &node1;
            this->nodes.push_back(node1);
            this->nodes.push_back(node0);
        }
    }

    spice_msgs::msg::Task to_task_msg()
    {
        spice_msgs::msg::Task task_msg;

        spice_msgs::msg::Layer layer_msg;
        for (size_t j = 0; j < this->nodes.size(); j++)
        {

            spice_msgs::msg::Node node_msg;
            node_msg.id = this->nodes[j].id;
            node_msg.work.type = this->nodes[j].work_type;

            for (size_t k = 0; k < this->nodes[j].children.size(); k++)
            {
                node_msg.children_id.push_back(this->nodes[j].children[k]->id);
            }
            layer_msg.nodes.push_back(node_msg);

            if (this->typeOfTree == 0)
            {
                if (j == 0 || j == 3 || j == 7 || j == 9)
                {
                    task_msg.layers.push_back(layer_msg);
                    layer_msg.nodes = {};
                }
            }
            else if (this->typeOfTree == 1)
            {
                if (j == 0 || j == 2 || j == 4 || j == 6)
                {
                    task_msg.layers.push_back(layer_msg);
                    layer_msg.nodes = {};
                }
            }
            else if (this->typeOfTree == 2 || this->typeOfTree == 3)
            {
                if (j == 0 || j == 1 || j == 2)
                {
                    task_msg.layers.push_back(layer_msg);
                    layer_msg.nodes = {};
                }
            }
            else if (this->typeOfTree == 4)
            {
                if (j == 0 || j == 1)
                {
                    task_msg.layers.push_back(layer_msg);
                    layer_msg.nodes = {};
                }
            }
        }

        return task_msg;
    }

private:
    std::vector<Node> nodes;
    Node *root_node;
    int8_t typeOfTree;
};

class Tree
{
public:
    Tree()
    {
        Node node0(0, "l", {});
        Node node1(1, "l", {});
        Node node2(2, "s", {&node0});
        Node node3(3, "l", {});
        Node node4(4, "h", {&node1});
        Node node5(5, "l", {});
        Node node6(6, "h", {&node2, &node3});
        Node node7(7, "s", {&node4, &node5});
        Node node8(8, "l", {});
        Node node9(9, "b", {&node6, &node7, &node8});
        this->root_node = &node9;
        this->nodes.push_back(node9);
        this->nodes.push_back(node8);
        this->nodes.push_back(node7);
        this->nodes.push_back(node6);
        this->nodes.push_back(node5);
        this->nodes.push_back(node4);
        this->nodes.push_back(node3);
        this->nodes.push_back(node2);
        this->nodes.push_back(node1);
        this->nodes.push_back(node0);
    }

    spice_msgs::msg::Task to_task_msg(std::vector<Node> product_variation){
        for (auto i : product_variation){
            
        }
    }
    void search_tree(std::vector<std::string> Works)
    {
        std::vector<std::vector<Node>> all_path_variations;
        while (true)
        {
            std::vector<Node> visited;
            Node *last_parent;
            std::vector<Node> stack;
            stack.push_back(*root_node);
            for (size_t i = 0; i < stack.size(); i++)
            {
                visited.push_back(stack[i]);

                if (stack[i].children.size() == 0)
                {
                    all_path_variations.push_back(visited);
                    break;
                }

                for (size_t j = 0; j < stack[i].children.size(); j++)
                {
                    stack.insert(stack.begin() + i + 1, *stack[i].children[j]);
                }

                if (stack[i].children.size() >= 2)
                {
                    last_parent = &stack[i];
                }
            }

            if (last_parent == NULL)
            {
                // save 'paths' to a file or something
                break;
            }
            //last_parent->children.erase(last_parent->children.end());
            last_parent->children.pop_back();
         
        }

        
        to_task_msg(all_path_variations[0]);
    }

private:
    std::vector<Node> nodes;
    Node *root_node;
    std::vector<std::vector<Node>> all_path_variations;
};

class TaskAllocator : public rclcpp::Node
{

public:
    TaskAllocator() : Node("task_allocator")
    {

        readyRobotsCli_ = this->create_client<spice_msgs::srv::GetReadyRobots>("get_ready_robots");

        GetReadyRobot();

        timerReadyBots_ = this->create_wall_timer(
            10s, std::bind(&TaskAllocator::GetReadyRobot, this));
    }

    void GetReadyRobot()
    {
        robotsRecieved = false;

        if (!readyRobotsCli_->wait_for_service(1s))
        {

            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for Swarm Manager service. Exiting.");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Swarm Manager service not available");
        }

        auto request = std::make_shared<spice_msgs::srv::GetReadyRobots::Request>();

        // We give the async_send_request() method a callback that will get executed once the response
        // is received.
        // This way we can return immediately from this method and allow other work to be done by the
        // executor in `spin` while waiting for the response.

        using ServiceResponseFuture =
            rclcpp::Client<spice_msgs::srv::GetReadyRobots>::SharedFuture;

        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            auto result = future.get();
            robots_ = result->robots;
            robotsRecieved = true;
            AllocateTasks();
        };

        auto futureResult = readyRobotsCli_->async_send_request(request, response_received_callback);
    }

    void AllocateTasks()
    {
        if (!robotsRecieved)
        {
            return;
        }

        for (const auto &robot : robots_)
        {
            allocTaskCli_ = this->create_client<spice_msgs::srv::RobotTask>(robot.id + "/allocate_task");

            if (!allocTaskCli_->wait_for_service(1s))
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "could not find %s task service ", robot.id.c_str());
                continue;
            }
            auto randnr = rand() % 4 + 1;
            tree = handmadeTrees(randnr);
            auto jobRequest = std::make_shared<spice_msgs::srv::RobotTask::Request>();
            jobRequest->task = this->tree.to_task_msg();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Allocating task to %s", robot.id.c_str());

            using ServiceResponseFuture =
                rclcpp::Client<spice_msgs::srv::RobotTask>::SharedFuture;

            auto response_received_callback = [this](ServiceResponseFuture future)
            {
                auto result = future.get();

                std::string tookJob;
                if (result->job_accepted)
                {
                    tookJob = "took the job";
                }
                else
                {
                    tookJob = "did not take the job";
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot %s ", tookJob.c_str());
            };

            auto futureResult = allocTaskCli_->async_send_request(jobRequest, response_received_callback);
        }
    }

private:
    rclcpp::Client<spice_msgs::srv::GetReadyRobots>::SharedPtr readyRobotsCli_;
    rclcpp::Client<spice_msgs::srv::RobotTask>::SharedPtr allocTaskCli_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr jobSubscription_;
    bool robotsRecieved = false;
    std::shared_ptr<spice_msgs::srv::GetReadyRobots_Request> request;
    std::vector<spice_msgs::msg::Id, std::allocator<spice_msgs::msg::Id>> robots_;
    rclcpp::TimerBase::SharedPtr timerReadyBots_{nullptr};

    handmadeTrees tree;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskAllocator>());
    rclcpp::shutdown();
    return 0;
}