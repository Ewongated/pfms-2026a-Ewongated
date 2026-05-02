#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <thread>
#include <cmath>
#include "pfms_types.h"

// Student defined libraries
#include "ackerman.h"
#include "skidsteer.h"

// Helper
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;

///////////////////////////////////////////////////////////
// Ackerman Constructor Tests
///////////////////////////////////////////////////////////

/**
 * @brief Validates Ackerman initial state immediately after construction.
 *
 * No goals are supplied. All public methods must be safely callable.
 * Status must be IDLE, all counters zero, execute(true) must reject (no goals).
 */
TEST(AckermanConstructor, InitialState) {
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0, 2, 0);
        pfmsHogPtr->teleport(odo);
    }

    Ackerman ackerman;

    // Platform type correct
    EXPECT_EQ(ackerman.getPlatformType(), pfms::PlatformType::ACKERMAN);

    // Status is IDLE with goalIndex 0
    unsigned int goalIndex = 99;
    pfms::PlatformStatus status = ackerman.status(goalIndex);
    EXPECT_EQ(status, pfms::PlatformStatus::IDLE);
    EXPECT_EQ(goalIndex, 0u);

    // All counters initialised to zero
    EXPECT_DOUBLE_EQ(ackerman.distanceTravelled(), 0.0);
    EXPECT_DOUBLE_EQ(ackerman.timeTravelled(), 0.0);
    EXPECT_DOUBLE_EQ(ackerman.distanceToGoal(), 0.0);
    EXPECT_DOUBLE_EQ(ackerman.timeToGoal(), 0.0);

    // execute(true) should fail — no goals set
    EXPECT_FALSE(ackerman.execute(true));

    // Status remains IDLE after failed execute
    status = ackerman.status(goalIndex);
    EXPECT_EQ(status, pfms::PlatformStatus::IDLE);

    // execute(false) should succeed — safe to stop when already idle
    EXPECT_TRUE(ackerman.execute(false));

    // setTolerance should succeed
    EXPECT_TRUE(ackerman.setTolerance(0.5));

    // getOdometry should return a valid odometry struct
    pfms::nav_msgs::Odometry odo = ackerman.getOdometry();
    EXPECT_FALSE(std::isnan(odo.position.x));
    EXPECT_FALSE(std::isnan(odo.position.y));
    EXPECT_FALSE(std::isnan(odo.yaw));
}

/**
 * @brief Validates Ackerman setGoals with empty vector.
 *
 * An empty goal list should be handled gracefully without crashing.
 */
TEST(AckermanConstructor, EmptyGoals) {
    Ackerman ackerman;

    std::vector<pfms::geometry_msgs::Point> emptyGoals;
    // setGoals with empty vector — should not crash, execute(true) should still fail
    ackerman.setGoals(emptyGoals);

    EXPECT_FALSE(ackerman.execute(true));

    unsigned int goalIndex = 0;
    EXPECT_EQ(ackerman.status(goalIndex), pfms::PlatformStatus::IDLE);
}

/**
 * @brief Validates Ackerman checkOriginToDestination before any goals set.
 *
 * Should return a valid reachability result using current odometry as origin.
 */
TEST(AckermanConstructor, CheckOriginToDestination) {
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0, 2, 0);
        pfmsHogPtr->teleport(odo);
    }

    Ackerman ackerman;

    pfms::nav_msgs::Odometry origin = ackerman.getOdometry();
    pfms::geometry_msgs::Point goal{20.0, 0.0, 0.0};
    double distance = 0.0, time = 0.0;
    pfms::nav_msgs::Odometry estimatedGoalPose;

    // Should not crash and should return a valid result
    bool reachable = ackerman.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);

    if (reachable) {
        EXPECT_GT(distance, 0.0);
        EXPECT_GT(time, 0.0);
    } else {
        EXPECT_DOUBLE_EQ(distance, -1.0);
        EXPECT_DOUBLE_EQ(time, -1.0);
    }
}

///////////////////////////////////////////////////////////
// Skidsteer Constructor Tests
///////////////////////////////////////////////////////////

/**
 * @brief Validates Skidsteer initial state immediately after construction.
 *
 * No goals are supplied. All public methods must be safely callable.
 * Status must be IDLE, all counters zero, execute(true) must reject (no goals).
 */
TEST(SkidsteerConstructor, InitialState) {
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(0, -2, 0);
        pfmsHogPtr->teleport(odo);
    }

    Skidsteer skidsteer;

    // Platform type correct
    EXPECT_EQ(skidsteer.getPlatformType(), pfms::PlatformType::SKIDSTEER);

    // Status is IDLE with goalIndex 0
    unsigned int goalIndex = 99;
    pfms::PlatformStatus status = skidsteer.status(goalIndex);
    EXPECT_EQ(status, pfms::PlatformStatus::IDLE);
    EXPECT_EQ(goalIndex, 0u);

    // All counters initialised to zero
    EXPECT_DOUBLE_EQ(skidsteer.distanceTravelled(), 0.0);
    EXPECT_DOUBLE_EQ(skidsteer.timeTravelled(), 0.0);
    EXPECT_DOUBLE_EQ(skidsteer.distanceToGoal(), 0.0);
    EXPECT_DOUBLE_EQ(skidsteer.timeToGoal(), 0.0);

    // execute(true) should fail — no goals set
    EXPECT_FALSE(skidsteer.execute(true));

    // Status remains IDLE after failed execute
    status = skidsteer.status(goalIndex);
    EXPECT_EQ(status, pfms::PlatformStatus::IDLE);

    // execute(false) should succeed — safe to stop when already idle
    EXPECT_TRUE(skidsteer.execute(false));

    // setTolerance should succeed
    EXPECT_TRUE(skidsteer.setTolerance(0.5));

    // getOdometry should return a valid odometry struct
    pfms::nav_msgs::Odometry odo = skidsteer.getOdometry();
    EXPECT_FALSE(std::isnan(odo.position.x));
    EXPECT_FALSE(std::isnan(odo.position.y));
    EXPECT_FALSE(std::isnan(odo.yaw));
}

/**
 * @brief Validates Skidsteer setGoals with empty vector.
 *
 * An empty goal list should be handled gracefully without crashing.
 */
TEST(SkidsteerConstructor, EmptyGoals) {
    Skidsteer skidsteer;

    std::vector<pfms::geometry_msgs::Point> emptyGoals;
    skidsteer.setGoals(emptyGoals);

    EXPECT_FALSE(skidsteer.execute(true));

    unsigned int goalIndex = 0;
    EXPECT_EQ(skidsteer.status(goalIndex), pfms::PlatformStatus::IDLE);
}

/**
 * @brief Validates Skidsteer checkOriginToDestination before any goals set.
 *
 * Skidsteer can always reach any 2D point — should always return true.
 */
TEST(SkidsteerConstructor, CheckOriginToDestination) {
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(0, -2, 0);
        pfmsHogPtr->teleport(odo);
    }

    Skidsteer skidsteer;

    pfms::nav_msgs::Odometry origin = skidsteer.getOdometry();
    pfms::geometry_msgs::Point goal{10.0, -10.0, 0.0};
    double distance = 0.0, time = 0.0;
    pfms::nav_msgs::Odometry estimatedGoalPose;

    // Skidsteer can always reach any 2D point
    bool reachable = skidsteer.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);
    EXPECT_TRUE(reachable);
    EXPECT_GT(distance, 0.0);
    EXPECT_GT(time, 0.0);

    // Estimated goal pose should match goal position
    EXPECT_NEAR(estimatedGoalPose.position.x, goal.x, 0.01);
    EXPECT_NEAR(estimatedGoalPose.position.y, goal.y, 0.01);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}