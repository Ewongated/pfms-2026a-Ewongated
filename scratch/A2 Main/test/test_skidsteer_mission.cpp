#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include "pfms_types.h"

// Student defined libraries
#include "skidsteer.h"
#include "mission.h"

// Helper
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;

///////////////////////////////////////////////////////////
// Skidsteer Mission Interrupt/Resume Test
///////////////////////////////////////////////////////////

/**
 * @brief Validates correct behaviour when an active Skidsteer mission is
 * interrupted and resumed with new goals via the Mission interface.
 *
 * Test flow:
 *  1.  Teleport Skidsteer to start, create controller + Mission (BASIC)
 *  2.  Set initial goals, execute(true) — verify non-blocking + RUNNING
 *  3.  Poll mission status until at least one goal is completed
 *  4.  execute(false) — verify controller transitions to IDLE
 *  5.  Set new goals (reverse of initial) via mission->setGoals, execute(true) — verify RUNNING
 *  6.  Verify goalIdx resets to 0 (new goals, not re-executing old ones)
 *  7.  Poll mission status until 100% or timeout
 *  8.  Verify new goals reached via PfmsHog
 *  9.  Print summary of all assertion values for debugging
 */
TEST(SkidsteerMission, InterruptAndResume) {

    // ── Setup ─────────────────────────────────────────────────────────────────
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(0, -2, 0);
        pfmsHogPtr->teleport(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.5);

    // Initial goals — matching existing skidsteer test style
    std::vector<pfms::geometry_msgs::Point> initialGoals;
    initialGoals.push_back({ 5.0, -2.0, 2.0});
    initialGoals.push_back({ 7.0, -4.0, 2.0});
    initialGoals.push_back({ 5.0, -6.0, 2.0});

    MissionInterface* mission = new Mission(controllers);
    mission->setMissionObjective(pfms::MissionObjective::BASIC);
    mission->setGoals(initialGoals, pfms::PlatformType::SKIDSTEER);

    // ── Tracking variables for summary ────────────────────────────────────────
    bool executeNonBlocking             = false;
    bool controllerRunningAfterStart    = false;
    bool goalCompletedBeforeInterrupt   = false;
    bool controllerIdleAfterInterrupt   = false;
    bool missionBelow100AfterInterrupt  = false;
    bool controllerRunningAfterResume   = false;
    bool goalIdxResetToZero             = false;
    bool newGoalsCompletedInTime        = false;
    bool pfmsHogGoalsReached            = false;

    double distBeforeInterrupt          = 0.0;
    double distAfterResume              = 0.0;
    double executeElapsed               = 0.0;
    unsigned int goalIdxAtInterrupt     = 0;
    unsigned int goalIdxAfterResume     = 0;
    unsigned int missionPctAfterInterrupt = 0;
    std::vector<double> hogDistances;

    // ── Phase 1: Start mission ────────────────────────────────────────────────
    auto executeStart = std::chrono::system_clock::now();
    bool started = mission->execute(true);
    auto executeEnd = std::chrono::system_clock::now();
    executeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        executeEnd - executeStart).count();

    executeNonBlocking = (executeElapsed < 2.0);

    // Allow mission thread time to start controller before checking status
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    unsigned int goalIdx = 0;
    pfms::PlatformStatus ctrlStatus = controllers.front()->status(goalIdx);
    controllerRunningAfterStart = (ctrlStatus == pfms::PlatformStatus::RUNNING);

    // ── Phase 2: Wait for at least one goal to complete ───────────────────────
    auto waitStart = std::chrono::system_clock::now();
    double maxWait = 90.0;

    while (true) {
        auto now = std::chrono::system_clock::now();
        double waited = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - waitStart).count();
        if (waited > maxWait) break;

        controllers.front()->status(goalIdx);
        if (goalIdx >= 1) {
            goalCompletedBeforeInterrupt = true;
            goalIdxAtInterrupt = goalIdx;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    distBeforeInterrupt = controllers.front()->distanceTravelled();

    // ── Phase 3: Interrupt mission ────────────────────────────────────────────
    mission->execute(false);

    // Allow time for thread join and status to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ctrlStatus = controllers.front()->status(goalIdx);
    controllerIdleAfterInterrupt = (ctrlStatus == pfms::PlatformStatus::IDLE);

    std::vector<unsigned int> missionSt = mission->status();
    if (!missionSt.empty()) {
        missionPctAfterInterrupt      = missionSt.front();
        missionBelow100AfterInterrupt = (missionPctAfterInterrupt < 100u);
    }

    // ── Phase 4: Set new goals (reverse of initial) and resume ───────────────
    std::vector<pfms::geometry_msgs::Point> newGoals;
    newGoals.push_back({ 5.0, -6.0, 2.0});
    newGoals.push_back({ 7.0, -4.0, 2.0});
    newGoals.push_back({ 5.0, -2.0, 2.0});

    pfmsHogPtr->setGoals(newGoals);
    mission->setGoals(newGoals, pfms::PlatformType::SKIDSTEER);

    bool resumed = mission->execute(true);

    // Allow time for thread to start and controller to transition
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ctrlStatus = controllers.front()->status(goalIdxAfterResume);
    controllerRunningAfterResume = (ctrlStatus == pfms::PlatformStatus::RUNNING);
    goalIdxResetToZero           = (goalIdxAfterResume == 0u);

    // ── Phase 5: Wait for new goals to complete ───────────────────────────────
    auto resumeStart = std::chrono::system_clock::now();
    double maxResumeTime = 120.0;
    bool timeExceeded = false;

    while (true) {
        auto now = std::chrono::system_clock::now();
        double waited = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - resumeStart).count();
        if (waited > maxResumeTime) {
            timeExceeded = true;
            break;
        }

        std::vector<unsigned int> st = mission->status();
        if (!st.empty() && st.front() >= 100u) {
            newGoalsCompletedInTime = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    distAfterResume = controllers.front()->distanceTravelled();

    // ── Phase 6: Verify via PfmsHog ──────────────────────────────────────────
    pfmsHogGoalsReached = pfmsHogPtr->checkGoalsReached(hogDistances);

    // ── Assertions ────────────────────────────────────────────────────────────
    EXPECT_TRUE(started)                     << "mission->execute(true) should return true";
    EXPECT_TRUE(executeNonBlocking)          << "execute(true) should be non-blocking (<2s)";
    EXPECT_TRUE(controllerRunningAfterStart) << "Controller should be RUNNING after execute(true)";
    ASSERT_TRUE(goalCompletedBeforeInterrupt)<< "At least one goal must complete before interrupt";
    EXPECT_TRUE(controllerIdleAfterInterrupt)<< "Controller should be IDLE after execute(false)";
    EXPECT_TRUE(missionBelow100AfterInterrupt)<< "Mission should not be 100% after interruption";
    EXPECT_TRUE(resumed)                     << "mission->execute(true) should succeed on resume";
    EXPECT_TRUE(controllerRunningAfterResume)<< "Controller should be RUNNING after resume";
    EXPECT_TRUE(goalIdxResetToZero)          << "goalIdx should reset to 0 for new goals";
    EXPECT_FALSE(timeExceeded)               << "New goals should complete within time limit";
    EXPECT_TRUE(pfmsHogGoalsReached)         << "PfmsHog should confirm new goals reached";

    for (unsigned int i = 0; i < newGoals.size(); ++i) {
        if (i < hogDistances.size()) {
            EXPECT_LT(hogDistances.at(i), 0.5)
                << "New goal " << i << " distance from goal: " << hogDistances.at(i);
        }
    }

    // ── Summary ───────────────────────────────────────────────────────────────
    std::cout << "\n========== TEST SUMMARY: SkidsteerMission/InterruptAndResume ==========\n";
    std::cout << "[phase1] execute(true) returned:         " << (started                      ? "true  PASS" : "false FAIL") << "\n";
    std::cout << "[phase1] Non-blocking (<2s):             " << (executeNonBlocking            ? "PASS" : "FAIL") << "  (" << executeElapsed << "s)\n";
    std::cout << "[phase1] Controller RUNNING after start: " << (controllerRunningAfterStart   ? "PASS" : "FAIL") << "\n";
    std::cout << "[phase2] Goal completed before int:      " << (goalCompletedBeforeInterrupt  ? "PASS" : "FAIL") << "  (goalIdx=" << goalIdxAtInterrupt << ")\n";
    std::cout << "[phase2] Distance before interrupt:      " << distBeforeInterrupt << " m\n";
    std::cout << "[phase3] Controller IDLE after int:      " << (controllerIdleAfterInterrupt  ? "PASS" : "FAIL") << "\n";
    std::cout << "[phase3] Mission pct after interrupt:    " << missionPctAfterInterrupt << "%  " << (missionBelow100AfterInterrupt ? "PASS" : "FAIL") << "\n";
    std::cout << "[phase4] execute(true) resumed:          " << (resumed                       ? "PASS" : "FAIL") << "\n";
    std::cout << "[phase4] Controller RUNNING after resume:" << (controllerRunningAfterResume  ? "PASS" : "FAIL") << "\n";
    std::cout << "[phase4] goalIdx reset to 0:             " << (goalIdxResetToZero            ? "PASS" : "FAIL") << "  (goalIdx=" << goalIdxAfterResume << ")\n";
    std::cout << "[phase5] New goals completed in time:    " << (newGoalsCompletedInTime        ? "PASS" : "FAIL") << "\n";
    std::cout << "[phase5] Distance after resume:          " << distAfterResume << " m\n";
    std::cout << "[phase6] PfmsHog goals reached:          " << (pfmsHogGoalsReached           ? "PASS" : "FAIL") << "\n";
    for (unsigned int i = 0; i < newGoals.size(); ++i) {
        if (i < hogDistances.size()) {
            std::cout << "[phase6] New goal " << i << " distance:           "
                      << hogDistances.at(i) << " m  "
                      << (hogDistances.at(i) < 0.5 ? "PASS" : "FAIL") << "\n";
        }
    }
    std::cout << "========================================================================\n\n";

    // ── Cleanup ───────────────────────────────────────────────────────────────
    delete mission;
    for (auto* ctrl : controllers) delete ctrl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}