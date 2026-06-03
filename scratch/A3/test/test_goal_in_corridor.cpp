#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "racing_track_pkg/track_analyser.h"

/**
 * @brief Unit tests for TrackAnalyser::isGoalInCorridor().
 *
 * Uses rosbags recorded with the car on the racing track:
 *  - goal_in_corridor_positive.bag: car on track, active goal placed at the
 *    track centre.  isGoalInCorridor() must return true.
 *  - goal_in_corridor_negative.bag: car on track, active goal placed off-centre
 *    by more than 0.2 m (e.g. 1 m to the left of centre).
 *    isGoalInCorridor() must return false.
 *
 * How to record the bags:
 * @code
 * # Positive case — goal at track centre, car on straight:
 * ros2 bag record -o goal_in_corridor_positive /orange/laserscan /orange/odom
 * # Negative case — goal placed 1 m to the left of centre:
 * ros2 bag record -o goal_in_corridor_negative /orange/laserscan /orange/odom
 * @endcode
 *
 * How to run:
 * @code
 * ros2 run racing_track_pkg test_goal_in_corridor
 * @endcode
 */

// ── Helper (same approach as obstacle test) ───────────────────────────────────

static bool readFirstScanAndOdom(const std::string& bagPath,
                                 sensor_msgs::msg::LaserScan& scan,
                                 nav_msgs::msg::Odometry& odom)
{
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri        = bagPath;
    storage_opts.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions conv_opts;
    conv_opts.input_serialization_format  = "cdr";
    conv_opts.output_serialization_format = "cdr";
    reader.open(storage_opts, conv_opts);

    rclcpp::Serialization<sensor_msgs::msg::LaserScan> scanSer;
    rclcpp::Serialization<nav_msgs::msg::Odometry>     odomSer;
    bool gotScan = false, gotOdom = false;

    while (reader.has_next() && !(gotScan && gotOdom)) {
        auto msg = reader.read_next();
        if (msg->topic_name == "/orange/laserscan" && !gotScan) {
            rclcpp::SerializedMessage s(*msg->serialized_data);
            scanSer.deserialize_message(&s, &scan);
            gotScan = true;
        }
        if (msg->topic_name == "/orange/odom" && !gotOdom) {
            rclcpp::SerializedMessage s(*msg->serialized_data);
            odomSer.deserialize_message(&s, &odom);
            gotOdom = true;
        }
    }
    return gotScan && gotOdom;
}

// ── Helper: extract yaw from odometry quaternion ──────────────────────────────

static double yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    tf2::Quaternion q(odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

// ── Fixture ───────────────────────────────────────────────────────────────────

class GoalInCorridorTest : public ::testing::Test
{
protected:
    TrackAnalyser analyser_{8.0, 0.2, 6.0};

    const std::string positiveBag_ =
        std::string(std::getenv("TEST_BAG_PATH") ? std::getenv("TEST_BAG_PATH") : ".") +
        "/goal_in_corridor_positive";
    const std::string negativeBag_ =
        std::string(std::getenv("TEST_BAG_PATH") ? std::getenv("TEST_BAG_PATH") : ".") +
        "/goal_in_corridor_negative";
};

// ── Positive case — goal IS at corridor centre ────────────────────────────────

TEST_F(GoalInCorridorTest, GoalAtCentreIsValid)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(readFirstScanAndOdom(positiveBag_, scan, odom))
        << "Could not read from " << positiveBag_;

    const double carX = odom.pose.pose.position.x;
    const double carY = odom.pose.pose.position.y;
    const double yaw  = yawFromOdom(odom);

    std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
    for (auto& r : ranges) {
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
    }

    auto hits     = analyser_.toWorldPoints(ranges, carX, carY, yaw,
                                            scan.angle_min, scan.angle_increment);
    auto corridor = analyser_.analyseCorridor(hits, carX, carY, yaw);

    ASSERT_TRUE(corridor.valid) << "Corridor must be detected for this test to be meaningful.";

    // The goal at the corridor centre should be within tolerance.
    const bool result = analyser_.isGoalInCorridor(
        corridor.centreX, corridor.centreY, corridor, carX, carY, yaw);

    EXPECT_TRUE(result) << "Goal at computed corridor centre should be valid.";
}

// ── Negative case — goal is off-centre by > goalTolerance ────────────────────

TEST_F(GoalInCorridorTest, GoalOffCentreIsInvalid)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(readFirstScanAndOdom(negativeBag_, scan, odom))
        << "Could not read from " << negativeBag_;

    const double carX = odom.pose.pose.position.x;
    const double carY = odom.pose.pose.position.y;
    const double yaw  = yawFromOdom(odom);

    std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
    for (auto& r : ranges) {
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) r = 0.0;
    }

    auto hits     = analyser_.toWorldPoints(ranges, carX, carY, yaw,
                                            scan.angle_min, scan.angle_increment);
    auto corridor = analyser_.analyseCorridor(hits, carX, carY, yaw);

    ASSERT_TRUE(corridor.valid) << "Corridor must be detected for this test to be meaningful.";

    // Shift the goal 1.0 m laterally from the corridor centre — should be rejected.
    const double sinY = std::sin(yaw);
    const double cosY = std::cos(yaw);
    const double offCentreX = corridor.centreX - 1.0 * sinY;  // 1 m to the right
    const double offCentreY = corridor.centreY + 1.0 * cosY;

    const bool result = analyser_.isGoalInCorridor(
        offCentreX, offCentreY, corridor, carX, carY, yaw);

    EXPECT_FALSE(result) << "Goal 1 m off-centre should be rejected by corridor check.";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
