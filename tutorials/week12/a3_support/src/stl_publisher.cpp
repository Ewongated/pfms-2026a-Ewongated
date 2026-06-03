#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tinyxml2.h>
#include <fstream>
#include <sstream>
#include "ament_index_cpp/get_package_share_directory.hpp"


class StlPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;//!< Visualisation Marker publsiher
  int project_; //!< Project number

public:
  StlPublisher() : 
    Node("stl_publisher")
  {
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",10); 


    // Declare and get the projecv parameter
    project_ = this->declare_parameter<int>("project", 1);

    // If project_ is not 1 or 3 report error
    if (project_ != 1 && project_ != 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid project number: %d. Only 1 and 3 are supported.", project_);
    }
  }

public:
    void publish_marker()
    {

        // Get the absolute path to the SDF file in the pfms package
        std::string pfms_share = ament_index_cpp::get_package_share_directory("pfms");
        
        // Set the paths based on the project number
        std::string sdf_path, stl_path;
        switch (project_) {
            case 1:
                sdf_path = pfms_share + "/models/terrain_1/model.sdf";
                stl_path = "package://pfms/models/terrain_1/media/terrain_1.dae";
                break;
        
            case 3:
                sdf_path = pfms_share + "/models/maze1/model.sdf";
                stl_path = "package://pfms/models/maze1/map.stl";
                break;
        
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid project number: %d. Only 1 and 3 are supported.", project_);
                return;
        }
        RCLCPP_INFO(this->get_logger(), "SDF path: %s", sdf_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Model path: %s", stl_path.c_str());

        // We are completing this as the existing rviz subscribes to the visualization marker as marker_array
        // So we create a MarkerArray message
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
    
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "stl_publisher";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        auto pose = read_pose_from_sdf(sdf_path);
        marker.pose.position.x = pose[0];
        marker.pose.position.y = pose[1];
        marker.pose.position.z = pose[2];

        RCLCPP_INFO(this->get_logger(), "Pose from SDF: %f %f %f", pose[0], pose[1], pose[2]);

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.mesh_resource =  stl_path;
        marker.mesh_use_embedded_materials = true;

        marker.lifetime = rclcpp::Duration(0, 0);
        marker_array.markers.push_back(marker);

        // Publish the marker array
        pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Publishing STL marker");

    }  

    std::array<double, 6> read_pose_from_sdf(const std::string& sdf_path) {
        std::array<double, 6> pose = {0, 0, 0, 0, 0, 0};
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger("stl_publisher"), "Failed to load SDF file: %s", sdf_path.c_str());
            return pose;
        }
        auto sdfElem = doc.FirstChildElement("sdf");
        if (!sdfElem) {
            RCLCPP_WARN(rclcpp::get_logger("stl_publisher"), "No <sdf> element in file: %s", sdf_path.c_str());
            return pose;
        }
        auto modelElem = sdfElem->FirstChildElement("model");
        if (!modelElem) {
            RCLCPP_WARN(rclcpp::get_logger("stl_publisher"), "No <model> element in file: %s", sdf_path.c_str());
            return pose;
        }
        auto linkElem = modelElem->FirstChildElement("link");
        if (!linkElem) {
            RCLCPP_WARN(rclcpp::get_logger("stl_publisher"), "No <link> element in file: %s", sdf_path.c_str());
            return pose;
        }
        auto visualElem = linkElem->FirstChildElement("visual");
        if (!visualElem) {
            RCLCPP_WARN(rclcpp::get_logger("stl_publisher"), "No <visual> element in file: %s", sdf_path.c_str());
            return pose;
        }
        auto poseElem = visualElem->FirstChildElement("pose");
        if (!poseElem || !poseElem->GetText()) {
            RCLCPP_WARN(rclcpp::get_logger("stl_publisher"), "No <pose> element or text in file: %s", sdf_path.c_str());
            return pose;
        }
        std::istringstream iss(poseElem->GetText());
        for (int i = 0; i < 6; ++i) iss >> pose[i];
        return pose;
    }

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StlPublisher>();
    node->publish_marker(); // Publish once
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
