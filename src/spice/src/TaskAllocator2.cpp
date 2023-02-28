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
    Node(int id_, int8_t work_type_, std::vector<Node *> children_)
    {
        this->id = id_;
        this->work_type = work_type_;
        this->children = children_;
    }

    bool is_leaf() { return children.size() == 0; }
    // int8_t depth() { return children.size(); }

    int id;
    int8_t work_type;
    std::vector<Node *> children;
};

class Tree
{
public:
    Tree()
    {
        spice_msgs::msg::Task task;
        Node node1(1, 1, {});
        Node node2(2, 1, {});
        Node node3(3, 2, {&node1});
        Node node4(4, 1, {});
        Node node5(5, 3, {&node2});
        Node node6(6, 1, {});
        Node node7(7, 3, {&node3, &node4});
        Node node8(8, 2, {&node5, &node6});
        Node node9(9, 1, {});
        Node node10(10, 4, {&node7, &node8, &node9});
        this->root_node = &node10;
        this->nodes.push_back(node1);
        this->nodes.push_back(node2);
        this->nodes.push_back(node3);
        this->nodes.push_back(node4);
        this->nodes.push_back(node5);
        this->nodes.push_back(node6);
        this->nodes.push_back(node7);
        this->nodes.push_back(node8);
        this->nodes.push_back(node9);
        this->nodes.push_back(node10);
    }

    spice_msgs::msg::Task to_task_msg()
    {
        spice_msgs::msg::Task task_msg;

        for (size_t j = 0; j < this->nodes.size(); j++)
        {
            int8_t i = 0;
            if (j == 1)
            {
                i = 1;
            }
            else if (j <= 2 && j >= 4)
            {
                i = 2;
            }
            else if (j <= 5 && j >= 8)
            {
                i = 3;
            }
            else if (j <= 9)
            {
                i = 4;
            }
            else
            {
                continue;
            }

            task_msg.layers[i].nodes[j].id = this->nodes[j].id;
            task_msg.layers[i].nodes[j].work.type = (int8_t)this->nodes[j].work_type;
            for (size_t k = 0; k < this->nodes[j].children.size(); k++)
            {
                task_msg.layers[i].nodes[j].children_id[k] = this->nodes[j].children[k]->id;
            }
        }
        return task_msg;
    }

private:
    std::vector<Node> nodes;
    Node *root_node;
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
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "in grr_cb; now calling allocate Task " << robots_.size());
            AllocateTasks();
        };

        auto futureResult = readyRobotsCli_->async_send_request(request, response_received_callback);
    }

    void AllocateTasks()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "in AT func");
        if (!robotsRecieved)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "!robotsRecieved");
            return;
        }

        for (const auto &robot : robots_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "creating client");
            allocTaskCli_ = this->create_client<spice_msgs::srv::RobotTask>(robot.id + "/allocate_task");

            if (!allocTaskCli_->wait_for_service(1s))
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "could not find %s task service ", robot.id.c_str());
                continue;
            }

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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "end of AT func");
    }

    // Initialize the matrix to zero
    void graph(int numVertices)
    {
        this->numVertices = numVertices;
        adjMatrix = new float *[numVertices];
        for (int i = 0; i < numVertices; i++)
        {
            adjMatrix[i] = new float[numVertices];
            for (int j = 0; j < numVertices; j++)
                adjMatrix[i][j] = false;
        }
    }

    // Add edges
    void addEdge(int i, int j, float cost)
    {
        adjMatrix[i][j] = cost;
    }

    // Remove edges
    void removeEdge(int i, int j)
    {
        adjMatrix[i][j] = 0;
    }

    // Print the martix
    void printGraph()
    {
        for (int i = 0; i < numVertices; i++)
        {
            std::cout << i << " : ";
            for (int j = 0; j < numVertices; j++)
                std::cout << adjMatrix[i][j] << " ";
            std::cout << "\n";
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

    float **adjMatrix;
    int numVertices;
    Tree tree;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // TaskAllocator tm;
    // tm.graph(6);
    // tm.addEdge(0, 1, 1);
    // tm.addEdge(0, 2, 2);
    // tm.addEdge(1, 3, 1);
    // tm.addEdge(2, 4, 1);
    // tm.addEdge(3, 5, 1);
    // tm.addEdge(4, 5, 2);
    // tm.printGraph();

    rclcpp::spin(std::make_shared<TaskAllocator>());
    rclcpp::shutdown();
    return 0;
}