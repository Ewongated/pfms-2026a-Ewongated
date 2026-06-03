#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "logger.h"
#include <thread>
#include <fstream>
#include <vector>
#include <random>

/*! @brief Obtain MarkerArray of CUBES from geometry_msgs::msg::Point
 * The markers are reported in world coordinate frames, namespace goals, type CUBE, colour green
 *
 *  @param goals - vector of geometry_msgs::msg::Point
 *  @return
 */
visualization_msgs::msg::MarkerArray produceMarkerList(std::vector<geometry_msgs::msg::Point> goals, rclcpp::Time stamp){

    visualization_msgs::msg::MarkerArray markerArray;
    unsigned int ct=0;

    for (auto pt:goals){

        {
            visualization_msgs::msg::Marker marker;

            //We need to set the frame
            // Set the frame ID and time stamp.
            marker.header.frame_id = "world";
            marker.header.stamp = stamp;

            //We set lifetime (it will dissapear in this many seconds)
            marker.lifetime = rclcpp::Duration(1000,0); //zero is forever

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
            marker.id = ct; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

            // The marker type
            marker.type = visualization_msgs::msg::Marker::CUBE;

            // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = pt.x;
            marker.pose.position.y = pt.y;
            marker.pose.position.z = pt.z;


            //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;


            // Set the scale of the marker -- 1m side
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            //Let's send a marker with color (green for reachable, red for now)
            std_msgs::msg::ColorRGBA color;
            color.a=0.5;//a is alpha - transparency 0.5 is 50%;
            color.r=0;
            color.g=1.0; // We picked green (1.0 is full color - sometimes colour is 0-255 here 0-1)
            color.b=0;

            marker.color = color;

            markerArray.markers.push_back(marker);
        }

        {

            visualization_msgs::msg::Marker marker;


            //We need to set the frame
            // Set the frame ID and time stamp.
            marker.header.frame_id = "world";
            //marker.header.seq = seq;
            marker.header.stamp = stamp;


            //We set lifetime (it will dissapear in this many seconds)
            marker.lifetime = rclcpp::Duration(1000,0);
            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "goals_text";
            marker.id = ct;

            // The marker type, we use a cylinder in this example
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
            marker.action = visualization_msgs::msg::Marker::ADD;

            //As an example, we are setting it
            marker.pose.position.x = pt.x;
            marker.pose.position.y = pt.y;
            marker.pose.position.z = pt.z + 0.5; // We are putting it 0.5m above the cube

            //Orientation, can we orientate it?
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 1.0;

            //Alpha is stransparency (50% transparent)
            marker.color.a = 0.5f;

            //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
            marker.color.r = 1.0;
            marker.color.g = 0.0;//static_cast<float>(177.0/255.0);
            marker.color.b = 0.0;

            std::stringstream ss;
            ss << marker.id;

            marker.text = ss.str();
            markerArray.markers.push_back(marker);
        }
        ct++;
    }    


    return markerArray;
}

bool loadPoints(std::string fileName, std::vector<geometry_msgs::msg::Point>& points){

    std::ifstream file(fileName , std::ios::in);

    points.clear();
    if (!file.is_open()){
        std::cerr << "Can not open " << fileName  << std::endl;
        return false;
    }
    std::string line;
    // Read one line at a time into the variable line:
    while (file.is_open()){
        if(std::getline(file, line))
        {
          std::stringstream  lineStream(line);
          geometry_msgs::msg::Point pt;
          lineStream >> pt.x;
          lineStream >> pt.y;
          lineStream >> pt.z;
          points.push_back(pt);
        }
        else {
            //std::cerr << "Closing " << __func__  << std::endl;
            file.close();
        }
    }
    return true;
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    //! By default this code will load goals from this package (data/goals.txt)
    //! rosrun project_setup goals_publisher
    //!
    //! You can also supply goals from a file, for example:
    //! ros2 run a3_support goals_publisher --ros-args -p filename:=/home/student/ros2_ws/install/a3_support/share/a3_support/data/A3_RACING_GOALS.TXT

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("goals_publisher");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_support");
    std::string default_filename=package_share_directory + "/data/GOALS.TXT"; 

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Filename to load goals";
    node->declare_parameter("filename", default_filename, param_desc);
    std::string filename = node->get_parameter("filename").as_string();    
    RCLCPP_INFO_STREAM(node->get_logger(),"file name with goals to be saved:" << filename);


    std::vector<geometry_msgs::msg::Point> goals;
    bool OK = loadPoints(filename, goals);

    if(!OK){
        RCLCPP_FATAL_STREAM(node->get_logger(), "Unable to read the goals from :" << filename);
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::Time stamp = node->get_clock()->now();

    geometry_msgs::msg::PoseArray msg; // This is our message type
    msg.header.frame_id = "world";
    msg.header.stamp = stamp;

    for(auto goal:goals){
        geometry_msgs::msg::Pose p;
        p.position = goal;
        p.orientation.w=1.0; // Just making the quaternion proper (sums to 1)
        msg.poses.push_back(p);
    }

    //Publish goals 
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_pub = 
        node->create_publisher<geometry_msgs::msg::PoseArray>("goals", 100);
    //Publishing markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub = 
        node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",1000);

    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //Wait to allow connection to be established


    visualization_msgs::msg::MarkerArray marker_msg = produceMarkerList(goals, stamp);
    RCLCPP_INFO_STREAM(node->get_logger(),"We are sending " <<  msg.poses.size() << " goals" );

    goals_pub->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    viz_pub->publish(marker_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;
}
