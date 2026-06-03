#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "racing_track_pkg/track_analyser.h"

/**
 * @brief Unit tests for TrackAnalyser::obstacleAhead().
 *
 * Uses rosbags recorded in the Gazebo simulation:
 *  - obstacle_ahead_positive.bag: laser scan with a box placed directly in
 *    front of the car in the forward cone.  obstacleAhead() must return true.
 *  - obstacle_ahead_negative.bag: laser scan with no obstacle; only wall
 *    returns are present.  obstacleAhead() must return false.
 *
 * How to record the bags:
 * @code
 * # Positive case — teleport a box 4 m in front of the car, then record:
 * ros2 bag record -o obstacle_ahead_positive /orange/laserscan /orange/odom
 * # Negative case — clear track, record:
 * ros2 bag record -o obstacle_ahead_negative /orange/laserscan /orange/odom
 * @endcode
 *
 * How to run:
 * @code
 * ros2 run racing_track_pkg test_obstacle_detection
 * @endcode
 */

// ── Helpers ───────────────────────────────────────────────────────────────────

/**
 * @brief Reads the first LaserScan and Odometry from a rosbag.
 *
 * Opens the bag at bagPath, iterates messages until both a LaserScan and an
 * Odometry have been read, then closes the bag.
 *
 * @param bagPath     Path to the rosbag directory.
 * @param[out] scan   First LaserScan found in the bag.
 * @param[out] odom   First Odometry found in the bag.
 * @return true if both messages were found.
 */
static bool readFirstScanAndOdom(const std::string& bagPath,
                                 sensor_msgs::msg::LaserScan& scan,
                                 nav_msgs::msg::Odometry& odom)
{
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = bagPath;
    storage_opts.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions conv_opts;
    conv_opts.input_serialization_format  = "cdr";
    conv_opts.output_serialization_format = "cdr";

    reader.open(storage_opts, conv_opts);

    rclcpp::Serialization<sensor_msgs::msg::LaserScan>  scanSer;
    rclcpp::Serialization<nav_msgs::msg::Odometry>      odomSer;
    bool gotScan = false, gotOdom = false;

    while (reader.has_next() && !(gotScan && gotOdom)) {
        auto bag_msg = reader.read_next();

        if (bag_msg->topic_name == "/orange/laserscan" && !gotScan) {
            rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
            scanSer.deserialize_message(&ser_msg, &scan);
            gotScan = true;
        }
        if (bag_msg->topic_name == "/orange/odom" && !gotOdom) {
            rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
            odomSer.deserialize_message(&ser_msg, &odom);
            gotOdom = true;
        }
    }
    return gotScan && gotOdom;
}

// ── Fixture ───────────────────────────────────────────────────────────────────

class ObstacleDetectionTest : public ::testing::Test
{
protected:
    TrackAnalyser analyser_{8.0, 0.2, 6.0};

    // These paths are resolved relative to the package install share directory
    // at runtime.  Update if your rosbag locations differ.
    const std::string positiveBag_ =
        std::string(std::getenv("TEST_BAG_PATH") ? std::getenv("TEST_BAG_PATH") : ".") +
        "/obstacle_ahead_positive";
    const std::string negativeBag_ =
        std::string(std::getenv("TEST_BAG_PATH") ? std::getenv("TEST_BAG_PATH") : ".") +
        "/obstacle_ahead_negative";
};

// ── Positive case — obstacle IS present ──────────────────────────────────────

TEST_F(ObstacleDetectionTest, DetectsObstacleAhead)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(readFirstScanAndOdom(positiveBag_, scan, odom))
        << "Could not read scan/odom from " << positiveBag_;

    // Convert ranges; mark invalid as 0.0.
    std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
    for (auto& r : ranges) {
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
    }

    const bool result = analyser_.obstacleAhead(ranges, scan.angle_min, scan.angle_increment);
    EXPECT_TRUE(result) << "Expected obstacle detected ahead (positive bag).";
}

// ── Negative case — no obstacle, only walls ───────────────────────────────────

TEST_F(ObstacleDetectionTest, NoObstacleOnClearTrack)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(readFirstScanAndOdom(negativeBag_, scan, odom))
        << "Could not read scan/odom from " << negativeBag_;

    std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
    for (auto& r : ranges) {
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
    }

    const bool result = analyser_.obstacleAhead(ranges, scan.angle_min, scan.angle_increment);
    EXPECT_FALSE(result) << "Expected no obstacle on clear track (negative bag).";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
