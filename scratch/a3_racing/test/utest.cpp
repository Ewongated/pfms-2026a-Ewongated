#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

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

    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri        = bagPath;
    storageOptions.storage_id = "sqlite3";

    rosbag2_cpp::Reader reader;
    reader.open(storageOptions);

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
// Test Suite 1: Obstacle detection (P/C)
//
// Bags required:
//   data/obstacle_clear/   — open track, nothing in front
//   data/obstacle_blocked/ — obstacle directly ahead
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Negative case — no obstacle in the forward corridor.
 * obstacleInFront() must return false.
 */
TEST(ObstacleDetection, NegativeCase_ClearCorridor)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("obstacle_clear", scan, odom))
        << "Could not load obstacle_clear bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);
    EXPECT_FALSE(lp.obstacleInFront())
        << "Expected no obstacle on clear track.";
}

/**
 * @brief Positive case — obstacle directly ahead.
 * obstacleInFront() must return true.
 */
TEST(ObstacleDetection, PositiveCase_ObstaclePresent)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("obstacle_blocked", scan, odom))
        << "Could not load obstacle_blocked bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);
    EXPECT_TRUE(lp.obstacleInFront())
        << "Expected obstacle detected ahead.";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test Suite 2: Goal corridor validation (P/C)
//
// Bags required:
//   data/goal_in_corridor/  — car on straight, goal at road centre
//   data/goal_out_corridor/ — car on straight, goal beside wall
//
// Car was at x=19.0, y=14.8 facing positive X when bags were recorded.
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Positive case — goal at road centre is inside corridor.
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

    // 5m ahead along positive X from car position — on track centre
    geometry_msgs::msg::Point centreGoal;
    centreGoal.x = 24.0;
    centreGoal.y = 14.8;
    centreGoal.z = 0.0;

    EXPECT_TRUE(lp.goalInCorridor(centreGoal))
        << "Goal at road centre should be inside corridor.";
}

/**
 * @brief Negative case — goal beside wall is outside corridor.
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

    // 6m to the side — well beyond the wall
    geometry_msgs::msg::Point wallGoal;
    wallGoal.x = 19.0;
    wallGoal.y = 20.8;
    wallGoal.z = 0.0;

    EXPECT_FALSE(lp.goalInCorridor(wallGoal))
        << "Goal beside wall should be outside corridor.";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test Suite 3: Laser-derived waypoint detection (D/HD)
//
// Bags required:
//   data/waypoint_visible/  — both walls visible to laser
//   data/waypoint_no_walls/ — one or both walls not visible
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Positive case — both walls visible, centre point computable.
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
        << "Expected valid track centre when both walls are visible.";

    if (result.has_value()) {
        const double dist = std::sqrt(
            std::pow(result->x - odom.pose.pose.position.x, 2) +
            std::pow(result->y - odom.pose.pose.position.y, 2));
        EXPECT_LT(dist, 15.0)
            << "Computed centre waypoint is unexpectedly far from the car.";
    }
}

/**
 * @brief Negative case — walls not visible, no waypoint returned.
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