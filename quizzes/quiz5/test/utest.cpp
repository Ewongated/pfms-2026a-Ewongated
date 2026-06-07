#include <gtest/gtest.h>
#include <climits>
#include <vector>

// #include "rclcpp/rclcpp.hpp"

// #include <ros/package.h> //This tool allows to identify the path of the package on your system
// #include <rosbag/bag.h>
// #include <rosbag/view.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// #include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/msg/laser_scan.hpp"
// #include <nav_msgs/Odometry.h>
// #include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "../src/laserprocessing.h"

TEST(LaserProcessing,CountReturns){

    // Reading bag files based on
    // https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("quiz5");
    std::string bag_filename=package_share_directory + "/data/ros2";

    rosbag2_cpp::Reader reader;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    reader.open(bag_filename);
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name != "/orange/laser/scan") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        break;
    }

    ////////////////////////////////////////////
    // Our code is tested below
    LaserProcessing laserProcessing(*laser_msg);
    unsigned int readings = laserProcessing.countObjectReadings();
    EXPECT_EQ(readings,34);

}

TEST(LaserProcessing,CountSegments){

    // Reading bag files based on
    // https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("quiz5");
    std::string bag_filename=package_share_directory + "/data/ros2";

    rosbag2_cpp::Reader reader;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    reader.open(bag_filename);
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name != "/orange/laser/scan") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        break;
    }


    ////////////////////////////////////////////
    // Our code is tested below

    sensor_msgs::msg::LaserScan scan;
    LaserProcessing laserProcessing(*laser_msg);

    unsigned int segments = laserProcessing.countSegments();
    EXPECT_EQ(segments,7);

}


TEST(LaserProcessing,DetectClosestCone){

    // Reading bag files based on
    // https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("quiz5");
    std::string bag_filename=package_share_directory + "/data/ros2";

    rosbag2_cpp::Reader reader;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    reader.open(bag_filename);
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name != "/orange/laser/scan") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        break;
    }

    ////////////////////////////////////////////
    // Our code is tested below

    LaserProcessing laserProcessing(*laser_msg);

    {
        geometry_msgs::msg::Point pt = laserProcessing.detectClosestCone();

        EXPECT_NEAR(pt.x,2.04752,0.1);
        EXPECT_NEAR(pt.y,-3.67013,0.1);
    }

}