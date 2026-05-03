#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "mission.h"
#include "laser.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(MissionObstruction, AckermanObstructed) {

    //We create the PfmHog object pointer and use it to set initial
    //pose of Ackerman for test as well as teleporting a box to goal location 
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0,2,0);
        pfmsHogPtr->teleport(odo);
        // We teleport an obstacle (box) to the location of a goal which should cause the mission to be 
        // abandoned when the controller detects the obstacle and stops
        pfmsHogPtr->teleportObject(pfms::geometry_msgs::Point{0,20,0}, "box1");
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    double tolerance = 0.5;
    controllers.front()->setTolerance(tolerance);

    std::vector<RangerInterface*> rangers; // We create a ranger on ackerman platform to detect the obstacle
    rangers.push_back(new Laser(pfms::PlatformType::ACKERMAN));

    std::vector<pfms::geometry_msgs::Point> goalsAck;
    goalsAck.push_back({ 20, 0});    // Goal 0
    goalsAck.push_back({ 30, 10});   // Goal 1
    goalsAck.push_back({ 20, 20});   // Goal 2
    goalsAck.push_back({ 0, 20});    // Goal 3 <- Box is here, should trigger abandonment
    goalsAck.push_back({ -20, 20});
    goalsAck.push_back({ -30, 10});
    goalsAck.push_back({ -20, 0});
    goalsAck.push_back({ 0, 2});

    pfmsHogPtr->setGoals(goalsAck);

    // We now have controller and goals, let's set up mission
    MissionInterface* mission = new Mission(controllers,rangers);
    mission->setMissionObjective(pfms::MissionObjective::ADVANCED);
    mission->setGoals(goalsAck,pfms::PlatformType::ACKERMAN);

    bool reachable = mission->execute(true);

    ASSERT_TRUE(reachable);

    if(reachable){
        std::cout << "All goals reachable" << std::endl;
    }
    else{
        std::cout << "Goals NOT reachable" << std::endl;
    }


    auto start_time = std::chrono::system_clock::now();
    double maxTime =180.0;
    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded

    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //This has exceeded the time we set for this test, we should fail the test
            timeExceeded=true;
            OK=true;
        }

        std::vector<unsigned int> status = mission->status();

        if (status.front() >= 100 && status.back() >= 100)
        { // we check progess, at 100 mission is acomplished
            // mission accomplished
            OK = true;
        }
        if(mission->isAbandoned())
        {
            OK = true; // Mission is abandoned, we can stop the test loop and check results
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // std::cout << "\r";
    }

    EXPECT_FALSE(timeExceeded); // time should not be exceeded

    // Check that mission was abandoned due to obstacle
    EXPECT_TRUE(mission->isAbandoned()) << "Mission should be abandoned due to obstacle at goal 3";
    
    // Check that the first (and only) controller was stopped
    std::vector<bool> abandonedControllers = mission->getAbandonedControllers();
    ASSERT_EQ(abandonedControllers.size(), 1) << "Should have one controller";
    EXPECT_TRUE(abandonedControllers.at(0)) << "Controller 0 should be marked as abandoned";

    std::vector<double> distances;
    bool reached = pfmsHogPtr->checkGoalsReached(distances);
    EXPECT_FALSE(reached) << "Not all goals should be reached due to obstacle"; // Mission abandoned, not all goals reached
    
    // Verify that we reached at least the first few goals before the obstruction
    // Goals 0, 1, 2 should be reached, goal 3 (where box is) should not be reached
    ASSERT_GE(distances.size(), 4) << "Should have distance info for at least 4 goals";
    EXPECT_LT(distances.at(0), tolerance) << "Goal 0 should be reached";
    EXPECT_LT(distances.at(1), tolerance) << "Goal 1 should be reached";
    EXPECT_LT(distances.at(2), tolerance) << "Goal 2 should be reached";
    EXPECT_GT(distances.at(3), tolerance) << "Goal 3 should NOT be reached (obstacle present)";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
