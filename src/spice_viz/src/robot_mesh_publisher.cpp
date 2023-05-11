#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spice_msgs/srv/get_robots_by_type.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class RobotMeshPublisher : public rclcpp::Node
{

public:
    RobotMeshPublisher() : Node("robot_mesh_publisher")
    {
        get_robots_cli = create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");

        get_work_cells_cli = create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");

        m_get_carrier_robots_timer = this->create_wall_timer(
            3s, std::bind(&RobotMeshPublisher::GetRobots, this));

        m_get_work_cells_timer = this->create_wall_timer(
            3s, std::bind(&RobotMeshPublisher::GetWorkcells, this));

        m_publish_mesh_timer = this->create_wall_timer(
            100ms, std::bind(&RobotMeshPublisher::PublishCarrierRobotMesh, this));

        m_mesh_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("/robot_meshes", 20);

        std::string package_share = ament_index_cpp::get_package_share_directory("spice_viz");
        work_cell_mesh = "file://" + package_share + "/meshes/polybot.obj";
        carrier_robot_mesh = "file://" + package_share + "/meshes/polybot.obj";

        tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void GetRobots()
    {
        if (!get_robots_cli->wait_for_service(1s))
        {

            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for Swarm Manager service. Exiting.");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Swarm Manager service not available");
        }

        auto request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
        request->type.type = spice_msgs::msg::RobotType::CARRIER_ROBOT;

        using ServiceResponseFuture =
            rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            auto result = future.get();
            m_carrier_robots = result->robots;
        };

        auto futureResult = get_robots_cli->async_send_request(request, response_received_callback);
    }

    void GetWorkcells()
    {
        if (!get_work_cells_cli->wait_for_service(1s))
        {

            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for Swarm Manager service. Exiting.");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Swarm Manager service not available");
        }

        auto request = std::make_shared<spice_msgs::srv::GetRobotsByType::Request>();
        request->type.type = spice_msgs::msg::RobotType::WORK_CELL_ANY;

        using ServiceResponseFuture =
            rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedFuture;

        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            visualization_msgs::msg::MarkerArray msg;
            auto result = future.get();
            m_work_cells = result->robots;
            for (auto work_cell : m_work_cells)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = -0.15;
                pose.position.y = 0.1;
                pose.position.z = 0;
                pose.orientation.x = 0;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 1;

                std_msgs::msg::ColorRGBA colour;
                colour.a = 1;
                colour.r = 1;
                colour.g = 0;
                colour.b = 0;

                visualization_msgs::msg::Marker marker;
                marker.ns = work_cell.id.id;
                marker.header.frame_id = work_cell.id.id;
                marker.header.stamp = this->get_clock()->now();
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                marker.pose = pose;
                marker.id = 0;
                marker.mesh_resource = work_cell_mesh;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color = colour;
                msg.markers.push_back(marker);
                // m_mesh_publisher->publish(marker);
                // RCLCPP_WARN(this->get_logger(), "publishing");

                marker.id = 1;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.text = work_cell.id.id;
                marker.pose.position.z += 0.2;
                msg.markers.push_back(marker);
            }
            //m_mesh_publisher->publish(msg);
        };

        auto futureResult = get_work_cells_cli->async_send_request(request, response_received_callback);
    }

    void PublishCarrierRobotMesh()
    {
        visualization_msgs::msg::MarkerArray msg;
        geometry_msgs::msg::TransformStamped t;

        for (auto robot : m_carrier_robots)
        {

        try {
            t = tf_buffer_->lookupTransform(
            "map", robot.id.id + "_base_link",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
         
          return;
        }
            //mesh transform;
            tf2::Quaternion q(0,0,1,0);
            tf2::Vector3 translation(-0.225, 0.15, 0.0);
            tf2::Transform mesh_trasform(q,translation);

            //bot transform;

            tf2::Quaternion mr_q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Vector3 mr_translation(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            tf2::Transform mr_transform(mr_q, mr_translation);

            auto mesh_wc = mr_transform * mesh_trasform;
            auto mesh_wc_t = mr_transform * translation;
            
        
            geometry_msgs::msg::Pose pose;
            pose.position.x = mesh_wc_t.getX();//t.transform.translation.x + 0.15;
            pose.position.y = mesh_wc_t.getY();//t.transform.translation.y - 0.1;
            pose.position.z = 0;
            pose.orientation.x = mesh_wc.getRotation().getX();
            pose.orientation.y = mesh_wc.getRotation().getY();
            pose.orientation.z = mesh_wc.getRotation().getZ(); 
            pose.orientation.w = mesh_wc.getRotation().getW();

            std_msgs::msg::ColorRGBA colour;
            colour.a = 1;
            colour.r = 0;
            colour.g = 1;
            colour.b = 1;

            visualization_msgs::msg::Marker marker;
            marker.ns = robot.id.id;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.pose = pose;
            marker.id = 0;
            marker.mesh_resource = work_cell_mesh;
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color = colour;
            msg.markers.push_back(marker);
            // m_mesh_publisher->publish(marker);

            marker.id = 1;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            
            std::string s = "polybot";
            std::string::size_type i = robot.id.id.find(s);

            if (i != std::string::npos)
            robot.id.id.erase(i, s.length());

            marker.text = robot.id.id;
            marker.pose.position.x = t.transform.translation.x;
            marker.pose.position.y = t.transform.translation.y;
            marker.pose.position.z += 0.2;
            marker.scale.z = 0.2;

            colour.a = 1;
            colour.r = 0;
            colour.g = 0;
            colour.b = 0.5;
            marker.color = colour;
            pose.orientation.x = t.transform.rotation.x;
            pose.orientation.y = t.transform.rotation.y;
            pose.orientation.z = t.transform.rotation.z; 
            pose.orientation.w = t.transform.rotation.w;
            
            msg.markers.push_back(marker);

            // m_mesh_publisher->publish(marker);
        }
        m_mesh_publisher->publish(msg);
    }

    std::vector<spice_msgs::msg::Robot> m_carrier_robots;
    std::vector<spice_msgs::msg::Robot> m_work_cells;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> m_mesh_publisher;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_robots_cli;
    rclcpp::Client<spice_msgs::srv::GetRobotsByType>::SharedPtr get_work_cells_cli;

    rclcpp::TimerBase::SharedPtr m_get_carrier_robots_timer{nullptr};
    rclcpp::TimerBase::SharedPtr m_get_work_cells_timer{nullptr};
    rclcpp::TimerBase::SharedPtr m_publish_mesh_timer{nullptr};

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string carrier_robot_mesh;
    std::string work_cell_mesh;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robot_mesh_publisher = std::make_shared<RobotMeshPublisher>();
    rclcpp::spin(robot_mesh_publisher);
    rclcpp::shutdown();
    return 0;
}
