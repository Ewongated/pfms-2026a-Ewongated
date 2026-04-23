/**
 * @file test_constructor.cpp
 * @brief Unit tests for Ackerman and Skidsteer constructors.
 *
 * Validates correct initialisation state, default behaviour with no goals,
 * consistency of status/configuration values, and safe invocation of all
 * public methods before any goals are supplied.
 */
#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <thread>
#include "pfms_types.h"
#include "ackerman.h"
#include "skidsteer.h"
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;

// ============================================================
// Ackerman constructor tests
// ============================================================

TEST(ConstructorTests, AckermanInitialStatus) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    hog->teleport(populateOdo(0, 2, 0));

    Ackerman ack;

    // Before any goals: status must be IDLE
    unsigned int goalIdx = 99;
    pfms::PlatformStatus ps = ack.status(goalIdx);
    EXPECT_EQ(ps, pfms::PlatformStatus::IDLE);
    EXPECT_EQ(goalIdx, 0u); // index resets to 0 at construction
}

TEST(ConstructorTests, AckermanPlatformType) {
    Ackerman ack;
    EXPECT_EQ(ack.getPlatformType(), pfms::PlatformType::ACKERMAN);
}

TEST(ConstructorTests, AckermanZeroDistanceAndTimeBeforeGoals) {
    Ackerman ack;
    EXPECT_NEAR(ack.distanceTravelled(), 0.0, 1e-6);
    EXPECT_NEAR(ack.timeTravelled(), 0.0, 1e-6);
}

TEST(ConstructorTests, AckermanSetToleranceBeforeGoals) {
    Ackerman ack;
    // setTolerance should succeed even before any goals are set
    EXPECT_TRUE(ack.setTolerance(0.5));
    EXPECT_TRUE(ack.setTolerance(1.0));
    EXPECT_TRUE(ack.setTolerance(0.1));
}

TEST(ConstructorTests, AckermanExecuteFalseBeforeGoals) {
    Ackerman ack;
    // execute(false) on an IDLE controller should be safe
    EXPECT_NO_FATAL_FAILURE(ack.execute(false));
    unsigned int idx = 0;
    EXPECT_EQ(ack.status(idx), pfms::PlatformStatus::IDLE);
}

TEST(ConstructorTests, AckermanExecuteTrueWithNoGoals) {
    Ackerman ack;
    // execute(true) with no goals should fail gracefully (no thread, still IDLE)
    bool result = ack.execute(true);
    EXPECT_FALSE(result); // cannot start with no goals
    unsigned int idx = 0;
    EXPECT_EQ(ack.status(idx), pfms::PlatformStatus::IDLE);
}

TEST(ConstructorTests, AckermanOdometryReadableAtConstruction) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    hog->teleport(populateOdo(5, 5, 0));

    Ackerman ack;
    // getOdometry() must be callable and return a valid (non-zero) position
    // since the platform has been teleported
    pfms::nav_msgs::Odometry odo = ack.getOdometry();
    // We just verify the call does not crash; position may not have updated yet
    EXPECT_NO_FATAL_FAILURE(ack.getOdometry());
}

TEST(ConstructorTests, AckermanSetEmptyGoals) {
    Ackerman ack;
    // Setting an empty goal list should succeed without crashing
    std::vector<pfms::geometry_msgs::Point> empty;
    EXPECT_NO_FATAL_FAILURE(ack.setGoals(empty));
    unsigned int idx = 0;
    EXPECT_EQ(ack.status(idx), pfms::PlatformStatus::IDLE);
}

TEST(ConstructorTests, AckermanCheckOriginToDestinationCallable) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    hog->teleport(populateOdo(0, 0, 0));

    Ackerman ack;
    pfms::nav_msgs::Odometry origin = populateOdo(0, 0, 0);
    pfms::geometry_msgs::Point goal{10, 0, 0};
    double dist = 0.0, time = 0.0;
    pfms::nav_msgs::Odometry estPose;

    // Should be callable and return a valid distance
    bool reachable = ack.checkOriginToDestination(origin, goal, dist, time, estPose);
    EXPECT_TRUE(reachable);
    EXPECT_GT(dist, 0.0);
    EXPECT_GT(time, 0.0);
}

// ============================================================
// Skidsteer constructor tests
// ============================================================

TEST(ConstructorTests, SkidsteerInitialStatus) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    hog->teleport(populateOdo(0, -2, 0));

    Skidsteer skid;

    unsigned int goalIdx = 99;
    pfms::PlatformStatus ps = skid.status(goalIdx);
    EXPECT_EQ(ps, pfms::PlatformStatus::IDLE);
    EXPECT_EQ(goalIdx, 0u);
}

TEST(ConstructorTests, SkidsteerPlatformType) {
    Skidsteer skid;
    EXPECT_EQ(skid.getPlatformType(), pfms::PlatformType::SKIDSTEER);
}

TEST(ConstructorTests, SkidsteerZeroDistanceAndTimeBeforeGoals) {
    Skidsteer skid;
    EXPECT_NEAR(skid.distanceTravelled(), 0.0, 1e-6);
    EXPECT_NEAR(skid.timeTravelled(), 0.0, 1e-6);
}

TEST(ConstructorTests, SkidsteerSetToleranceBeforeGoals) {
    Skidsteer skid;
    EXPECT_TRUE(skid.setTolerance(0.5));
    EXPECT_TRUE(skid.setTolerance(1.0));
}

TEST(ConstructorTests, SkidsteerExecuteFalseBeforeGoals) {
    Skidsteer skid;
    EXPECT_NO_FATAL_FAILURE(skid.execute(false));
    unsigned int idx = 0;
    EXPECT_EQ(skid.status(idx), pfms::PlatformStatus::IDLE);
}

TEST(ConstructorTests, SkidsteerExecuteTrueWithNoGoals) {
    Skidsteer skid;
    bool result = skid.execute(true);
    EXPECT_FALSE(result);
    unsigned int idx = 0;
    EXPECT_EQ(skid.status(idx), pfms::PlatformStatus::IDLE);
}

TEST(ConstructorTests, SkidsteerCheckOriginToDestinationCallable) {
    Skidsteer skid;
    pfms::nav_msgs::Odometry origin = populateOdo(0, 0, 0);
    pfms::geometry_msgs::Point goal{10, 0, 0};
    double dist = 0.0, time = 0.0;
    pfms::nav_msgs::Odometry estPose;

    bool reachable = skid.checkOriginToDestination(origin, goal, dist, time, estPose);
    EXPECT_TRUE(reachable);
    EXPECT_NEAR(dist, 10.0, 0.01);
    EXPECT_GT(time, 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
