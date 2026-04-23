/**
 * @file test_obstacle_detection.cpp
 * @brief Unit tests for laser-based obstacle detection in Mission (ADVANCED mode).
 *
 * Tests both the unobstructed and obstructed cases for Ackerman and Skidsteer.
 * Uses PfmsHog to teleport boxes into goal locations to create obstructions.
 */
#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "pfms_types.h"
#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"
#include "laser.h"
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;

// ============================================================
// Ackerman — unobstructed path completes all goals
// ============================================================

TEST(ObstacleDetection, AckermanUnobstructed) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    hog->teleport(populateOdo(0, 2, 0));

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Laser(pfms::PlatformType::ACKERMAN));

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({10,  0, 0});
    goals.push_back({15, -5, 0});
    goals.push_back({10, -10, 0});
    goals.push_back({ 0,  2, 0});

    hog->setGoals(goals);

    MissionInterface* mission = new Mission(controllers, rangers);
    mission->setMissionObjective(pfms::MissionObjective::ADVANCED);
    mission->setGoals(goals, pfms::PlatformType::ACKERMAN);

    bool reachable = mission->execute(true);
    ASSERT_TRUE(reachable);

    auto start = std::chrono::steady_clock::now();
    double maxTime = 120.0;
    bool done = false;
    bool timeExceeded = false;

    while (!done) {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > maxTime) { timeExceeded = true; done = true; break; }

        std::vector<unsigned int> st = mission->status();
        if (st.front() >= 100 || mission->isAbandoned()) done = true;
        else std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded);
    EXPECT_FALSE(mission->isAbandoned()) << "No obstacle placed — mission should not be abandoned";

    std::vector<double> distances;
    bool reached = hog->checkGoalsReached(distances);
    EXPECT_TRUE(reached) << "All goals should be reached on clear path";

    delete mission;
    for (auto* r : rangers) delete r;
    for (auto* c : controllers) delete c;
}

// ============================================================
// Ackerman — obstructed path triggers abandonment
// ============================================================

TEST(ObstacleDetection, AckermanObstructedAbandonsMission) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    hog->teleport(populateOdo(0, 2, 0));

    // Place a box at goal 2 to obstruct the path
    {
        bool placed = hog->teleportObject(pfms::geometry_msgs::Point{15, -10, 0}, "box1");
        if (!placed) { GTEST_SKIP() << "teleportObject not supported — run with pre-placed obstacle in Gazebo"; }
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Laser(pfms::PlatformType::ACKERMAN));

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({10,  0, 0});   // Goal 0 — clear
    goals.push_back({15, -5, 0});   // Goal 1 — clear
    goals.push_back({15, -10, 0});  // Goal 2 — box placed here
    goals.push_back({ 0,  2, 0});   // Goal 3 — never reached

    hog->setGoals(goals);

    MissionInterface* mission = new Mission(controllers, rangers);
    mission->setMissionObjective(pfms::MissionObjective::ADVANCED);
    mission->setGoals(goals, pfms::PlatformType::ACKERMAN);

    bool reachable = mission->execute(true);
    ASSERT_TRUE(reachable);

    auto start = std::chrono::steady_clock::now();
    double maxTime = 150.0;
    bool done = false;
    bool timeExceeded = false;

    while (!done) {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > maxTime) { timeExceeded = true; done = true; break; }

        if (mission->isAbandoned()) done = true;
        std::vector<unsigned int> st = mission->status();
        if (st.front() >= 100) done = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded);
    EXPECT_TRUE(mission->isAbandoned()) << "Mission should be abandoned due to obstacle";

    std::vector<bool> abandoned = mission->getAbandonedControllers();
    ASSERT_EQ(abandoned.size(), 1u);
    EXPECT_TRUE(abandoned.at(0)) << "Ackerman controller should be flagged as abandoned";

    // Goals 0 and 1 should be reached; goal 2 (box) and beyond should not
    std::vector<double> distances;
    hog->checkGoalsReached(distances);
    ASSERT_GE(distances.size(), 3u);
    EXPECT_LT(distances.at(0), 0.5) << "Goal 0 should be reached";
    EXPECT_LT(distances.at(1), 0.5) << "Goal 1 should be reached";
    EXPECT_GT(distances.at(2), 0.5) << "Goal 2 (obstructed) should not be reached";

    delete mission;
    for (auto* r : rangers) delete r;
    for (auto* c : controllers) delete c;
}

// ============================================================
// Skidsteer — unobstructed path completes all goals
// ============================================================

TEST(ObstacleDetection, SkidsteerUnobstructed) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    hog->teleport(populateOdo(0, -2, 0));

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.5);

    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Laser(pfms::PlatformType::SKIDSTEER));

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ 8, -5, 0});
    goals.push_back({12, -10, 0});
    goals.push_back({ 0, -2, 0});

    hog->setGoals(goals);

    MissionInterface* mission = new Mission(controllers, rangers);
    mission->setMissionObjective(pfms::MissionObjective::ADVANCED);
    mission->setGoals(goals, pfms::PlatformType::SKIDSTEER);

    bool reachable = mission->execute(true);
    ASSERT_TRUE(reachable);

    auto start = std::chrono::steady_clock::now();
    double maxTime = 120.0;
    bool done = false;
    bool timeExceeded = false;

    while (!done) {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > maxTime) { timeExceeded = true; done = true; break; }

        std::vector<unsigned int> st = mission->status();
        if (st.front() >= 100 || mission->isAbandoned()) done = true;
        else std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded);
    EXPECT_FALSE(mission->isAbandoned()) << "No obstacle — Skidsteer mission should complete";

    std::vector<double> distances;
    bool reached = hog->checkGoalsReached(distances);
    EXPECT_TRUE(reached) << "All Skidsteer goals should be reached on clear path";

    delete mission;
    for (auto* r : rangers) delete r;
    for (auto* c : controllers) delete c;
}

// ============================================================
// Skidsteer — obstructed path triggers abandonment
// ============================================================

TEST(ObstacleDetection, SkidsteerObstructedAbandonsMission) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    hog->teleport(populateOdo(0, -2, 0));

    // Place a box at goal 1
    {
        bool placed = hog->teleportObject(pfms::geometry_msgs::Point{12, -10, 0}, "box2");
        if (!placed) { GTEST_SKIP() << "teleportObject not supported — run with pre-placed obstacle in Gazebo"; }
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.5);

    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Laser(pfms::PlatformType::SKIDSTEER));

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ 8, -5, 0});   // Goal 0 — clear
    goals.push_back({12, -10, 0});  // Goal 1 — box placed here
    goals.push_back({ 0, -2, 0});   // Goal 2 — never reached

    hog->setGoals(goals);

    MissionInterface* mission = new Mission(controllers, rangers);
    mission->setMissionObjective(pfms::MissionObjective::ADVANCED);
    mission->setGoals(goals, pfms::PlatformType::SKIDSTEER);

    bool reachable = mission->execute(true);
    ASSERT_TRUE(reachable);

    auto start = std::chrono::steady_clock::now();
    double maxTime = 120.0;
    bool done = false;
    bool timeExceeded = false;

    while (!done) {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > maxTime) { timeExceeded = true; done = true; break; }

        if (mission->isAbandoned()) done = true;
        std::vector<unsigned int> st = mission->status();
        if (st.front() >= 100) done = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded);
    EXPECT_TRUE(mission->isAbandoned()) << "Mission should be abandoned due to obstacle";

    std::vector<bool> abandoned = mission->getAbandonedControllers();
    ASSERT_EQ(abandoned.size(), 1u);
    EXPECT_TRUE(abandoned.at(0)) << "Skidsteer controller should be flagged as abandoned";

    std::vector<double> distances;
    hog->checkGoalsReached(distances);
    ASSERT_GE(distances.size(), 2u);
    EXPECT_LT(distances.at(0), 0.5) << "Goal 0 should be reached";
    EXPECT_GT(distances.at(1), 0.5) << "Goal 1 (obstructed) should not be reached";

    delete mission;
    for (auto* r : rangers) delete r;
    for (auto* c : controllers) delete c;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
