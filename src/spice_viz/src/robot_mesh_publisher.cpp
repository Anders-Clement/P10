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
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::map<uint8_t, std::string> state_to_string{{spice_msgs::msg::RobotState::ERROR, "ERROR"}, {spice_msgs::msg::RobotState::MR_PROCESSING_JOB, "PROCESSING"}, {spice_msgs::msg::RobotState::MR_READY_FOR_JOB, "READY FOR JOB"}, {spice_msgs::msg::RobotState::STARTUP, "STARTUP"}, {spice_msgs::msg::RobotState::WC_READY_FOR_ROBOTS, "READY FOR ROBOT"}};

std::map<uint8_t, std::vector<float>> state_to_colour{{spice_msgs::msg::RobotState::ERROR, {0.75, 0.0, 0.0, 1.0}}, {spice_msgs::msg::RobotState::MR_PROCESSING_JOB, {0.0, 0.0, 0.75, 1.0}}, {spice_msgs::msg::RobotState::MR_READY_FOR_JOB, {0.0, 0.75, 0.0, 1.0}}, {spice_msgs::msg::RobotState::STARTUP, {0.75, 0.75, 0.0, 1.0}}, {spice_msgs::msg::RobotState::WC_READY_FOR_ROBOTS, {0.3, 0.99, 0.3, 1.0}}};//0.1, 0.67, 0.23 grøn; 0.1, 0.5, 0.9 blå

class RobotMeshPublisher : public rclcpp::Node
{

public:
    RobotMeshPublisher() : Node("robot_mesh_publisher")
    {
        get_robots_cli = create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");

        get_work_cells_cli = create_client<spice_msgs::srv::GetRobotsByType>("get_robots_by_type");

        m_get_carrier_robots_timer = this->create_wall_timer(
            1s, std::bind(&RobotMeshPublisher::GetRobots, this));

        m_get_work_cells_timer = this->create_wall_timer(
            1s, std::bind(&RobotMeshPublisher::GetWorkcells, this));

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

        // sub_remove_object = this->create_subscription<std_msgs::msg::String>("/carrier_timeout", 10, std::bind(&RobotMeshPublisher::remove_marker_cb, this, _1));
    }

private:

    std::string removeSpaces(std::string str)
    {
        str.erase(remove(str.begin(), str.end(), ' '), str.end());
        return str;
    }

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
                geometry_msgs::msg::TransformStamped t;
                try
                {
                    t = tf_buffer_->lookupTransform(
                        "map", work_cell.id.id,
                        tf2::TimePointZero);
                }
                catch (const tf2::TransformException &ex)
                {

                    continue;
                }
                std::vector<float> state_colour = state_to_colour[work_cell.robot_state.state];
                geometry_msgs::msg::Pose pose;
                pose.position.x = t.transform.translation.x;
                pose.position.y = t.transform.translation.y;
                pose.position.z = t.transform.translation.z;
                pose.orientation.x = t.transform.rotation.x;
                pose.orientation.y = t.transform.rotation.y;
                pose.orientation.z = t.transform.rotation.z;
                pose.orientation.w = t.transform.rotation.w;

                std_msgs::msg::ColorRGBA colour;
                colour.a = state_colour[3];
                colour.r = state_colour[0];
                colour.g = state_colour[1];
                colour.b = state_colour[2];

                visualization_msgs::msg::Marker marker;
                marker.ns = work_cell.id.id;
                marker.header.frame_id = "map";
                marker.header.stamp = this->get_clock()->now();
                marker.pose = pose;
                marker.color = colour;
                // m_mesh_publisher->publish(marker);
                // RCLCPP_WARN(this->get_logger(), "publishing");

                marker.id = 1;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.text = work_cell.id.id;
                marker.scale.z = 0.2;
                marker.pose.position.z += 0.2;
                msg.markers.push_back(marker);

                std::string internal_state_condensed = removeSpaces(work_cell.robot_state.internal_state);
                marker.id = 2;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.text = internal_state_condensed;
                marker.scale.z = 0.1;
                marker.pose.position.y += 0.2;
                msg.markers.push_back(marker);


                marker.id = 4;
                float cubeScale = 0.5;
                marker.pose.position.y = t.transform.translation.y;
                marker.pose.position.z = t.transform.translation.z+cubeScale/2;    
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = cubeScale;
                marker.scale.y = cubeScale;
                marker.scale.z = cubeScale;

                marker.color.a = 0.03;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                msg.markers.push_back(marker);


                marker.id = 3;  
                float lines_offset = 0.24;
                marker.header.frame_id = work_cell.id.id;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                marker.scale.x = 0.05;
                marker.pose.position.x = 0.0;
                marker.pose.position.y = 0.0;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                geometry_msgs::msg::Point point = marker.pose.position;
                point.x = lines_offset;
                point.y = -lines_offset;
                marker.points.push_back(point);
                point.x = -lines_offset;
                point.y = -lines_offset;
                marker.points.push_back(point);
                point.x = lines_offset;
                point.y = lines_offset;
                marker.points.push_back(point);
                point.x = -lines_offset;
                point.y = lines_offset;
                marker.points.push_back(point);
                marker.color.a = 1.0;
                marker.color.r = 0.35;
                marker.color.g = 0.35;
                marker.color.b = 0.35;
                msg.markers.push_back(marker);




                std::string frame_id_exit = work_cell.id.id+"_exit";
                // std::string frame_id_entry = work_cell.id.id+"_entry";

                colour.a = 1.0;
                colour.r = 1.0;
                colour.g = 0.0;
                colour.b = 0.0;

                msg.markers.push_back(makeArrow(frame_id_exit,colour,0));
                // msg.markers.push_back(makeArrow(frame_id_entry,colour,0));

                for (size_t i = 0; i < 99; i++)
                {
                    std::string frame_id =work_cell.id.id+"_q"+std::to_string(i);
                    try
                    {
                        t = tf_buffer_->lookupTransform(
                            "map", frame_id,
                            tf2::TimePointZero);
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        
                        break;
                    }

                    colour.a = 1.0;
                    colour.r = 1.0;
                    colour.g = 1.0;
                    colour.b = 1.0;
                    int id = i;

                    msg.markers.push_back(makeArrow(frame_id,colour,id));
                }
                
            }
            m_mesh_publisher->publish(msg);
        };

        auto futureResult = get_work_cells_cli->async_send_request(request, response_received_callback);
    }

    visualization_msgs::msg::Marker makeArrow(std::string frame_id, std_msgs::msg::ColorRGBA colour, int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = frame_id;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->get_clock()->now();
        float xScale = 0.2;
        marker.pose.position.x = -xScale/2;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.color = colour;

        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.scale.x = xScale;
        marker.scale.y = 0.1;
        marker.scale.z = 0.000001;

        return marker;
    }

    void PublishCarrierRobotMesh()
    {
        visualization_msgs::msg::MarkerArray msg;
        geometry_msgs::msg::TransformStamped t;

        for (auto robot : m_carrier_robots){
            try{
                t = tf_buffer_->lookupTransform("map", robot.id.id + "_base_link", tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex){
                return;
            }
            // mesh transform;
            tf2::Quaternion q(0, 0, 1, 0);
            tf2::Vector3 translation(-0.225, 0.15, 0.0);
            tf2::Transform mesh_trasform(q, translation);

            // bot transform;

            tf2::Quaternion mr_q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Vector3 mr_translation(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            tf2::Transform mr_transform(mr_q, mr_translation);

            auto mesh_wc = mr_transform * mesh_trasform;
            auto mesh_wc_t = mr_transform * translation;

            std::vector<float> state_colour = state_to_colour[robot.robot_state.state];
            geometry_msgs::msg::Pose pose;
            pose.position.x = mesh_wc_t.getX(); // t.transform.translation.x + 0.15;
            pose.position.y = mesh_wc_t.getY(); // t.transform.translation.y - 0.1;
            pose.position.z = 0;
            pose.orientation.x = mesh_wc.getRotation().getX();
            pose.orientation.y = mesh_wc.getRotation().getY();
            pose.orientation.z = mesh_wc.getRotation().getZ();
            pose.orientation.w = mesh_wc.getRotation().getW();

            std_msgs::msg::ColorRGBA colour;
            // robot model colour
            colour.a = state_colour[3];
            colour.r = state_colour[0];
            colour.g = state_colour[1];
            colour.b = state_colour[2];

            visualization_msgs::msg::Marker marker;
            marker.ns = robot.id.id;
            marker.lifetime.sec = 15;
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
            marker.pose.position.z += 0.3;
            marker.scale.z = 0.2;

            colour.a = 1;
            colour.r = 1;
            colour.g = 1;
            colour.b = 1;
            marker.color = colour;
            pose.orientation.x = t.transform.rotation.x;
            pose.orientation.y = t.transform.rotation.y;
            pose.orientation.z = t.transform.rotation.z;
            pose.orientation.w = t.transform.rotation.w;

            msg.markers.push_back(marker);

            marker.id = 2;
            //marker.scale.z = 0.2;
            colour.a = 1;
            colour.r = 1;
            colour.g = 1;
            colour.b = 1;
            marker.color = colour;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.text = robot.robot_state.internal_state;
            marker.scale.z = 0.1;


            marker.pose.position.y += 0.1;
            msg.markers.push_back(marker);

            marker.id = 3;
            //marker.scale.z = 0.2;
            colour.a = 1;
            colour.r = 0;
            colour.g = 0;
            colour.b = 0;
            marker.pose.position.y += -0.01;
            marker.color = colour;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.text = robot.robot_state.internal_state;
            marker.scale.z = 0.105;
            msg.markers.push_back(marker);

            // m_mesh_publisher->publish(marker);
        }
        m_mesh_publisher->publish(msg);
    }

    void remove_marker_cb(std_msgs::msg::String to_delete_id){
        std::string robot_to_delete = to_delete_id.data;
        RCLCPP_INFO(this->get_logger(), "remove mesh with ID: [%s]", robot_to_delete.c_str());

        visualization_msgs::msg::Marker marker;
        visualization_msgs::msg::MarkerArray msg;
        marker.ns = robot_to_delete;
        marker.id = 0;
        // marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.action = visualization_msgs::msg::Marker::DELETE;
        // marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        // marker.mesh_resource = work_cell_mesh;
        // marker.scale.x = 0.0;
        // marker.scale.y = 0.0;
        // marker.scale.z = 0.0;
        // marker.pose.position.x = 0.0;
        // marker.pose.position.y = 0.0;
        // marker.pose.position.z = 0.0;
        marker.color.a = 0.0; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.0;
        msg.markers.push_back(marker);
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

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_remove_object;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robot_mesh_publisher = std::make_shared<RobotMeshPublisher>();
    rclcpp::spin(robot_mesh_publisher);
    rclcpp::shutdown();
    return 0;
}
