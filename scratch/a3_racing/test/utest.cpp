/**
 * @file utest.cpp
 * @brief Unit tests for LaserProcessing — P/C and D/HD cases.
 *
 * ### Running the tests
 * @code
 * colcon build --packages-select a3_racing
 * colcon test  --packages-select a3_racing
 * colcon test-result --verbose
 * @endcode
 *
 * ### Bags required (relative to package share/data/)
 * | Folder              | Contents                                      |
 * |---------------------|-----------------------------------------------|
 * | obstacle_clear      | Open track, nothing ahead                     |
 * | obstacle_blocked    | Object directly in the forward corridor       |
 * | goal_in_corridor    | Car on straight, goal at road centre          |
 * | goal_out_corridor   | Car on straight, goal beside the wall         |
 * | waypoint_visible    | Both walls clearly visible to the laser       |
 * | waypoint_no_walls   | One or both walls absent / out of range       |
 *
 * Car position when corridor bags were recorded: x=19.0 m, y=14.8 m, facing +X.
 */
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

// ── Helper ────────────────────────────────────────────────────────────────────

/**
 * @brief Reads the first LaserScan and Odometry message from a named bag subfolder.
 * @param bagSubdir  Subdirectory name inside share/data/.
 * @param scanOut    Output laser scan.
 * @param odomOut    Output odometry.
 * @param laserTopic Topic name for the laser scan (default: /orange/laserscan).
 * @param odomTopic  Topic name for odometry (default: /orange/odom).
 * @return true if both message types were found.
 */
static bool loadBag(const std::string& bagSubdir,
                    sensor_msgs::msg::LaserScan& scanOut,
                    nav_msgs::msg::Odometry&     odomOut,
                    const std::string& laserTopic = "/orange/laserscan",
                    const std::string& odomTopic  = "/orange/odom")
{
    const std::string bagPath =
        ament_index_cpp::get_package_share_directory("a3_racing") + "/data/" + bagSubdir;

    rosbag2_storage::StorageOptions opts;
    opts.uri        = bagPath;
    opts.storage_id = "sqlite3";

    rosbag2_cpp::Reader reader;
    reader.open(opts);

    rclcpp::Serialization<sensor_msgs::msg::LaserScan> laserSer;
    rclcpp::Serialization<nav_msgs::msg::Odometry>     odomSer;
    bool gotLaser = false, gotOdom = false;

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

// ── Suite 1: Obstacle detection (P/C) ────────────────────────────────────────

/**
 * @brief Negative case — open track ahead, obstacleInFront() must return false.
 */
TEST(ObstacleDetection, NegativeCase_ClearCorridor)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("obstacle_clear", scan, odom)) << "Could not load obstacle_clear bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);
    EXPECT_FALSE(lp.obstacleInFront()) << "Expected no obstacle on clear track.";
}

/**
 * @brief Positive case — obstacle ahead, obstacleInFront() must return true.
 */
TEST(ObstacleDetection, PositiveCase_ObstaclePresent)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("obstacle_blocked", scan, odom)) << "Could not load obstacle_blocked bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);
    EXPECT_TRUE(lp.obstacleInFront()) << "Expected obstacle detected ahead.";
}

// ── Suite 2: Goal corridor validation (P/C) ───────────────────────────────────

/**
 * @brief Positive case — goal at road centre, goalInCorridor() must return true.
 */
TEST(GoalCorridor, PositiveCase_GoalAtCentre)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("goal_in_corridor", scan, odom)) << "Could not load goal_in_corridor bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    geometry_msgs::msg::Point centreGoal;
    centreGoal.x = 24.0; // 5 m ahead of recorded car position (x=19.0, y=14.8)
    centreGoal.y = 14.8;
    centreGoal.z = 0.0;

    EXPECT_TRUE(lp.goalInCorridor(centreGoal)) << "Goal at road centre should be inside corridor.";
}

/**
 * @brief Negative case — goal beside the wall, goalInCorridor() must return false.
 */
TEST(GoalCorridor, NegativeCase_GoalBesideWall)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("goal_out_corridor", scan, odom)) << "Could not load goal_out_corridor bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    geometry_msgs::msg::Point wallGoal;
    wallGoal.x = 24.0;
    wallGoal.y = 20.8; // 6 m lateral — beyond the wall
    wallGoal.z = 0.0;

    EXPECT_FALSE(lp.goalInCorridor(wallGoal)) << "Goal beside wall should be outside corridor.";
}

// ── Suite 3: Laser-derived waypoint detection (D/HD) ──────────────────────────

/**
 * @brief Positive case — both walls visible, trackCentreAhead() must return a valid point.
 */
TEST(WaypointDetection, PositiveCase_BothWallsVisible)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("waypoint_visible", scan, odom)) << "Could not load waypoint_visible bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    const auto result = lp.trackCentreAhead();
    EXPECT_TRUE(result.has_value()) << "Expected valid track centre when both walls are visible.";

    if (result.has_value()) {
        // The returned world-frame point should be within 12 m of the car
        // (laser offset 3.725 m + 5 m forward placement + rotation margin).
        const double dist = std::hypot(result->x - odom.pose.pose.position.x,
                                       result->y - odom.pose.pose.position.y);
        EXPECT_LT(dist, 12.0) << "Centre waypoint is unexpectedly far from the car.";
    }
}

/**
 * @brief Negative case — walls not visible, trackCentreAhead() must return std::nullopt.
 */
TEST(WaypointDetection, NegativeCase_InsufficientWallData)
{
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry     odom;
    ASSERT_TRUE(loadBag("waypoint_no_walls", scan, odom)) << "Could not load waypoint_no_walls bag.";

    LaserProcessing lp(scan);
    lp.newOdom(odom);

    const auto result = lp.trackCentreAhead();
    EXPECT_FALSE(result.has_value()) << "Expected nullopt when walls cannot be detected.";
}
