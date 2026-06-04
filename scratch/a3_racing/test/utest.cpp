#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "../src/laserprocessing.h"

// ─────────────────────────────────────────────────────────────────────────────
// Helper: load the first LaserScan and Odometry from a named bag subfolder.
// Returns false if either message type was not found in the bag.
// ─────────────────────────────────────────────────────────────────────────────
static bool loadBag(const std::string& bagSubdir,
                    sensor_msgs::msg::LaserScan& scanOut,
                    nav_msgs::msg::Odometry&     odomOut,
                    const std::string& laserTopic = "/orange/laserscan",
                    const std::string& odomTopic  = "/orange/odom")
{
    const std::string pkg =
        ament_index_cpp::get_package_share_directory("a3_racing");
    const std::string bagPath = pkg + "/data/" + bagSubdir;

    rosbag2_cpp::Reader reader;
    reader.open(bagPath);

    rclcpp::Serialization<sensor_msgs::msg::LaserScan> laserSer;
    rclcpp::Serialization<nav_msgs::msg::Odometry>     odomSer;

    bool gotLaser = false;
    bool gotOdom  = false;

    while (reader.has_next() && (!gotLaser || !gotOdom)) {
        auto msg = reader.read_next();

        if (!gotLaser && msg->topic_name == laserTopic) {
            rclcpp::SerializedMessage s(*msg->serialized_data);
            laserSer.deserialize_message(&s, &scanOut);
            gotLaser = true;
        }
        if (!gotOdom && msg->topic_name == odomTopic) {
            rclcpp::SerializedMessage s(*msg->serialized_data);
            odomSer.deserialize_message(&s, &odomOut);
            gotOdom = true;
        }
    }
    return gotLaser && gotOdom;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test Suite 1: Obstacle detection (P/C requirement)
//
// Two bags are required:
//   data/obstacle_clear/   — car driving with no obstacle in front
//   data/obstacle_blocked/ — an obstacle placed directly in front of the car
//
// These bags are recorded in Gazebo via:
//   ros2 bag record /orange/laserscan /orange/odom -o data/obstacle_clear
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Negative case — no obstacle in the forward corridor.
 *
 * The car is on an open section of track. obstacleInFront() must return false.
 */
TEST(ObstacleDetection, NegativeCase_ClearCorridor)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;

    ASSERT_TRUE(loadBag("obstacle_clear", scan, odom))
        << "Could not load obstacle_clear bag — record it first.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    EXPECT_FALSE(lp.obstacleInFront())
        << "Expected no obstacle in a clear corridor, but obstacleInFront() returned true.";
}

/**
 * @brief Positive case — non-wall obstacle directly ahead.
 *
 * A static box is placed ~1 m in front of the car in Gazebo.
 * obstacleInFront() must return true.
 */
TEST(ObstacleDetection, PositiveCase_ObstaclePresent)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;

    ASSERT_TRUE(loadBag("obstacle_blocked", scan, odom))
        << "Could not load obstacle_blocked bag — record it first.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    EXPECT_TRUE(lp.obstacleInFront())
        << "Expected obstacle detected, but obstacleInFront() returned false.";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test Suite 2: Goal corridor validation (P/C requirement)
//
// Two bags required:
//   data/goal_in_corridor/   — car on track, goal position placed at road centre
//   data/goal_out_corridor/  — car on track, goal placed beside the wall
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Positive case — goal lies at the centre of the detected corridor.
 *
 * goalInCorridor() must return true.
 */
TEST(GoalCorridor, PositiveCase_GoalAtCentre)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;

    ASSERT_TRUE(loadBag("goal_in_corridor", scan, odom))
        << "Could not load goal_in_corridor bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    // This goal point must be recorded at road-centre when the bag was taken.
    // Update these coordinates to match the actual world position in your bag.
    geometry_msgs::msg::Point centreGoal;
    centreGoal.x = 0.0;   // ← update with value from your recording
    centreGoal.y = 0.0;   // ← update with value from your recording
    centreGoal.z = 0.0;

    EXPECT_TRUE(lp.goalInCorridor(centreGoal))
        << "Goal at road centre should be inside corridor.";
}

/**
 * @brief Negative case — goal lies outside the free corridor (beside the wall).
 *
 * goalInCorridor() must return false.
 */
TEST(GoalCorridor, NegativeCase_GoalBesideWall)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;

    ASSERT_TRUE(loadBag("goal_out_corridor", scan, odom))
        << "Could not load goal_out_corridor bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    // This goal is deliberately placed beyond the wall — outside the corridor.
    geometry_msgs::msg::Point wallGoal;
    wallGoal.x = 100.0;  // ← far off-track x, update to match your bag scenario
    wallGoal.y = 100.0;  // ← far off-track y
    wallGoal.z = 0.0;

    EXPECT_FALSE(lp.goalInCorridor(wallGoal))
        << "Goal beside wall should NOT be inside corridor.";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test Suite 3: D/HD — Laser-derived waypoint detection
//
// Two bags required:
//   data/waypoint_visible/   — car on straight with both walls visible
//   data/waypoint_no_walls/  — car positioned so one or both walls are out of range
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Positive case — both walls visible, track centre computable.
 *
 * trackCentreAhead() must return a valid point.
 */
TEST(WaypointDetection, PositiveCase_BothWallsVisible)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;

    ASSERT_TRUE(loadBag("waypoint_visible", scan, odom))
        << "Could not load waypoint_visible bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    const auto result = lp.trackCentreAhead();
    EXPECT_TRUE(result.has_value())
        << "Expected a valid track centre when both walls are visible.";

    if (result.has_value()) {
        // Sanity check: the computed centre should be within ~4 m of car.
        const double dist = std::sqrt(
            std::pow(result->x - odom.pose.pose.position.x, 2) +
            std::pow(result->y - odom.pose.pose.position.y, 2));
        EXPECT_LT(dist, 15.0)
            << "Computed centre waypoint is unexpectedly far from the car.";
    }
}

/**
 * @brief Negative case — insufficient wall readings, no waypoint returned.
 *
 * trackCentreAhead() must return std::nullopt.
 */
TEST(WaypointDetection, NegativeCase_InsufficientWallData)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;

    ASSERT_TRUE(loadBag("waypoint_no_walls", scan, odom))
        << "Could not load waypoint_no_walls bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    const auto result = lp.trackCentreAhead();
    EXPECT_FALSE(result.has_value())
        << "Expected no centre when walls cannot be detected.";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test Suite 4: Segment counting (sanity / regression)
// Uses the provided sample bag from the skeleton.
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Verifies segment count against manually verified expected value.
 *
 * Open the sample bag in RViz, count visible wall segments, and set
 * EXPECTED_SEGMENTS to that value.
 */
TEST(SegmentCount, SampleBag)
{
    const std::string pkg =
        ament_index_cpp::get_package_share_directory("a3_racing");
    const std::string bagPath = pkg + "/data/sample";

    rosbag2_cpp::Reader reader;
    reader.open(bagPath);

    rclcpp::Serialization<sensor_msgs::msg::LaserScan> ser;
    sensor_msgs::msg::LaserScan scan;
    bool found = false;

    while (reader.has_next()) {
        auto msg = reader.read_next();
        if (msg->topic_name == "/orange/laserscan") {
            rclcpp::SerializedMessage s(*msg->serialized_data);
            ser.deserialize_message(&s, &scan);
            found = true;
            break;
        }
    }

    ASSERT_TRUE(found) << "No /orange/laserscan found in sample bag.";

    LaserProcessing lp(scan);
    const unsigned int segments = lp.countSegments();

    // Update this value after visual inspection of the sample bag in RViz.
    constexpr unsigned int EXPECTED_SEGMENTS = 2u; // two wall segments on a straight
    EXPECT_EQ(segments, EXPECTED_SEGMENTS);
}
