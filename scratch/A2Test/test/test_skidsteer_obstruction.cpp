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
#include "laser.h"

// Helper
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;

///////////////////////////////////////////////////////////
// Skidsteer Obstruction Test
///////////////////////////////////////////////////////////

/**
 * @brief Validates that a Skidsteer mission in ADVANCED mode is abandoned
 * when an obstacle is detected on the path to a subsequent goal.
 *
 * Goal 0 ({5, 20, 0}) is unobstructed. A box is placed at {0, 20, 0}
 * which lies on the path from goal 0 to goal 1 ({-10, 20, 0}).
 * After reaching goal 0 the laser detects the obstacle and the mission
 * is abandoned.
 */
TEST(ObstructionTest, SkidsteerObstructed) {

    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(20, 20, 0);
        pfmsHogPtr->teleport(odo);
        pfmsHogPtr->teleportObject(pfms::geometry_msgs::Point{-5, 20, -0.5}, "box1");
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    double tolerance = 0.5;
    controllers.front()->setTolerance(tolerance);

    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Laser(pfms::PlatformType::SKIDSTEER));

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({  5, 20, 0});   // Goal 0 — unobstructed
    goals.push_back({ -10, 20, 0});  // Goal 1 — box at {-5,20,0} is on this path, triggers abandonment

    pfmsHogPtr->setGoals(goals);

    MissionInterface* mission = new Mission(controllers, rangers);
    mission->setMissionObjective(pfms::MissionObjective::ADVANCED);
    mission->setGoals(goals, pfms::PlatformType::SKIDSTEER);

    bool reachable = mission->execute(true);
    ASSERT_TRUE(reachable);

    auto start_time = std::chrono::system_clock::now();
    double maxTime = 180.0;
    bool OK = false;
    bool timeExceeded = false;

    while (!OK) {
        auto current_time = std::chrono::system_clock::now();
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - start_time);

        if (time_taken.count() > maxTime) {
            timeExceeded = true;
            OK = true;
        }

        std::vector<unsigned int> status = mission->status();
        if (status.front() >= 100 && status.back() >= 100) {
            OK = true;
        }
        if (mission->isAbandoned()) {
            OK = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    EXPECT_FALSE(timeExceeded);

    EXPECT_TRUE(mission->isAbandoned()) << "Mission should be abandoned due to obstacle at goal 1";

    std::vector<bool> abandonedControllers = mission->getAbandonedControllers();
    ASSERT_EQ(abandonedControllers.size(), 1) << "Should have one controller";
    EXPECT_TRUE(abandonedControllers.at(0)) << "Controller 0 should be marked as abandoned";

    std::vector<double> distances;
    bool reached = pfmsHogPtr->checkGoalsReached(distances);
    EXPECT_FALSE(reached) << "Not all goals should be reached due to obstacle";

    ASSERT_GE(distances.size(), 2) << "Should have distance info for at least 2 goals";
    EXPECT_LT(distances.at(0), tolerance) << "Goal 0 should be reached";
    EXPECT_GT(distances.at(1), tolerance) << "Goal 1 should NOT be reached (obstacle present)";

    delete mission;
    for (auto* ctrl : controllers) delete ctrl;
    for (auto* r : rangers) delete r;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}