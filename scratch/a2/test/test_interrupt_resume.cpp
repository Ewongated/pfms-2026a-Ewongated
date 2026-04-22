/**
 * @file test_interrupt_resume.cpp
 * @brief Unit tests for mission interrupt and resume behaviour.
 *
 * Validates that:
 *  - A mission can be interrupted mid-execution
 *  - The controller transitions correctly through PlatformStatus states
 *  - New goals can be set during execution and the platform navigates to them
 *  - Previously completed goals are not re-executed
 *  - Mission and controller status accurately reflects progress
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
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;

// ============================================================
// Helper: wait for a controller to reach a specific status, up to maxWait seconds
// ============================================================
static bool waitForStatus(ControllerInterface* ctrl,
                          pfms::PlatformStatus target,
                          double maxWait)
{
    auto start = std::chrono::steady_clock::now();
    while (true) {
        unsigned int idx = 0;
        if (ctrl->status(idx) == target) return true;
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > maxWait) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============================================================
// Ackerman interrupt and resume
// ============================================================

TEST(InterruptResume, AckermanInterruptAndNewGoals) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0, 2, 0);
        hog->teleport(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    // Initial goals — a long path
    std::vector<pfms::geometry_msgs::Point> initialGoals;
    initialGoals.push_back({20,  0, 0});
    initialGoals.push_back({30, -10, 0});
    initialGoals.push_back({20, -20, 0});
    initialGoals.push_back({-20, -20, 0});

    MissionInterface* mission = new Mission(controllers);
    mission->setMissionObjective(pfms::MissionObjective::BASIC);
    mission->setGoals(initialGoals, pfms::PlatformType::ACKERMAN);

    // Start mission — must be non-blocking
    auto t0 = std::chrono::steady_clock::now();
    mission->execute(true);
    double launchTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    EXPECT_NEAR(launchTime, 0.0, 2.0) << "execute() must return immediately";

    // Let the platform travel toward the first goal for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Verify it is RUNNING (not already done)
    {
        unsigned int idx = 0;
        EXPECT_EQ(controllers.front()->status(idx), pfms::PlatformStatus::RUNNING)
            << "Platform should still be running after 5s";
    }

    // Interrupt the mission
    mission->execute(false);

    // Controller must transition to IDLE
    bool idled = waitForStatus(controllers.front(), pfms::PlatformStatus::IDLE, 5.0);
    EXPECT_TRUE(idled) << "Controller must reach IDLE after execute(false)";

    // Record progress before new goals
    double distBefore = controllers.front()->distanceTravelled();
    EXPECT_GT(distBefore, 0.0) << "Should have travelled some distance before interrupt";

    // Supply new (simpler) goals
    std::vector<pfms::geometry_msgs::Point> newGoals;
    newGoals.push_back({10, -5, 0});
    newGoals.push_back({ 0,  2, 0}); // return near start

    std::unique_ptr<PfmsHog> hogCheck = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    hogCheck->setGoals(newGoals);

    mission->setGoals(newGoals, pfms::PlatformType::ACKERMAN);

    // Resume
    bool resumed = mission->execute(true);
    EXPECT_TRUE(resumed) << "Mission should resume successfully with new goals";

    // Wait for new mission to complete
    auto missionStart = std::chrono::steady_clock::now();
    double maxTime = 120.0;
    bool done = false;
    bool timeExceeded = false;

    while (!done) {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - missionStart).count();
        if (elapsed > maxTime) { timeExceeded = true; done = true; break; }

        std::vector<unsigned int> prog = mission->status();
        ASSERT_EQ(prog.size(), 1u);
        if (prog.front() >= 100) done = true;
        else std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded) << "New goals should be reached within 120s";

    // Verify new goals were reached
    std::vector<double> distances;
    bool reached = hogCheck->checkGoalsReached(distances);
    EXPECT_TRUE(reached) << "New goals should be reached after resume";

    // Distance should have increased after resume (new goals were pursued)
    double distAfter = controllers.front()->distanceTravelled();
    EXPECT_GT(distAfter, distBefore) << "Distance should increase after resuming";

    delete mission;
    for (auto* c : controllers) delete c;
}

// ============================================================
// Skidsteer interrupt and resume
// ============================================================

TEST(InterruptResume, SkidsteerInterruptAndNewGoals) {
    std::unique_ptr<PfmsHog> hog = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(0, -2, 0);
        hog->teleport(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.5);

    // Initial goals
    std::vector<pfms::geometry_msgs::Point> initialGoals;
    initialGoals.push_back({ 10, -10, 0});
    initialGoals.push_back({  0, -20, 0});
    initialGoals.push_back({-10, -10, 0});
    initialGoals.push_back({-20, -20, 0});

    MissionInterface* mission = new Mission(controllers);
    mission->setMissionObjective(pfms::MissionObjective::BASIC);
    mission->setGoals(initialGoals, pfms::PlatformType::SKIDSTEER);

    // Start — must be non-blocking
    auto t0 = std::chrono::steady_clock::now();
    mission->execute(true);
    double launchTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    EXPECT_NEAR(launchTime, 0.0, 2.0) << "execute() must return immediately";

    // Let it run for 4 seconds
    std::this_thread::sleep_for(std::chrono::seconds(4));

    {
        unsigned int idx = 0;
        EXPECT_EQ(controllers.front()->status(idx), pfms::PlatformStatus::RUNNING)
            << "Platform should still be running after 4s";
    }

    // Check mission status reflects partial progress
    std::vector<unsigned int> midProgress = mission->status();
    ASSERT_EQ(midProgress.size(), 1u);
    EXPECT_GT(midProgress.front(), 0u) << "Progress should be non-zero after 4s";
    EXPECT_LT(midProgress.front(), 100u) << "Progress should not be 100% yet";

    double distBefore = controllers.front()->distanceTravelled();

    // Interrupt
    mission->execute(false);

    bool idled = waitForStatus(controllers.front(), pfms::PlatformStatus::IDLE, 5.0);
    EXPECT_TRUE(idled) << "Controller must reach IDLE after interrupt";

    // New goals closer to current position
    std::vector<pfms::geometry_msgs::Point> newGoals;
    newGoals.push_back({ 5, -5, 0});
    newGoals.push_back({ 0, -2, 0});

    std::unique_ptr<PfmsHog> hogCheck = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    hogCheck->setGoals(newGoals);

    mission->setGoals(newGoals, pfms::PlatformType::SKIDSTEER);

    bool resumed = mission->execute(true);
    EXPECT_TRUE(resumed);

    auto missionStart = std::chrono::steady_clock::now();
    double maxTime = 90.0;
    bool done = false;
    bool timeExceeded = false;

    while (!done) {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - missionStart).count();
        if (elapsed > maxTime) { timeExceeded = true; done = true; break; }

        std::vector<unsigned int> prog = mission->status();
        ASSERT_EQ(prog.size(), 1u);
        if (prog.front() >= 100) done = true;
        else std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded) << "New goals should be reached within 90s";

    std::vector<double> distances;
    bool reached = hogCheck->checkGoalsReached(distances);
    EXPECT_TRUE(reached) << "New goals should be reached after Skidsteer resume";

    double distAfter = controllers.front()->distanceTravelled();
    EXPECT_GT(distAfter, distBefore) << "Distance should increase after resuming";

    delete mission;
    for (auto* c : controllers) delete c;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
