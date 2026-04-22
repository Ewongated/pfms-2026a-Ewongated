#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(MissionReachGoals, ThreeGoalsEach) {

    // We create two PfmsHog objects, one for each platform
    // We teleport the platforms to starting locations
    std::unique_ptr<PfmsHog> pfmsHogAudiPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0,2,0);
        pfmsHogAudiPtr->teleport(odo);
    }
    std::unique_ptr<PfmsHog> pfmsHogHuskyPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    {
        Odometry odo = populateOdo(0,-2,0);
        pfmsHogHuskyPtr->teleport(odo);
    }

    // We create two controllers, one for each platform
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.5);
    controllers.back()->setTolerance(0.5);

    // The goals for the Ackerman platform
    std::vector<pfms::geometry_msgs::Point> goalsAck;
    goalsAck.push_back({ 20, 0});
    goalsAck.push_back({ 30, -10});
    goalsAck.push_back({ 20, -20});
    goalsAck.push_back({ 10, -20});
    goalsAck.push_back({ -20, -20});
    goalsAck.push_back({ -30, -10});
    goalsAck.push_back({ -20, 0});
    goalsAck.push_back({ 0, 2});
    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogAudiPtr->setGoals(goalsAck);

    // The goals for the Skidsteer platform
    std::vector<pfms::geometry_msgs::Point> goalsQuad;
    goalsQuad.push_back({ 10, -10, 2});
    goalsQuad.push_back({ 0, -20, 2});
    goalsQuad.push_back({ -10, -10, 2});
    goalsQuad.push_back({ 0, -2, 2});
    pfmsHogHuskyPtr->setGoals(goalsQuad);

    // We now have controller and goals, let's set up mission
    MissionInterface* mission = new Mission(controllers);
    mission->setMissionObjective(pfms::MissionObjective::BASIC);
    mission->setGoals(goalsAck,pfms::PlatformType::ACKERMAN);
    mission->setGoals(goalsQuad,pfms::PlatformType::SKIDSTEER);

    auto start_call_mission_time = std::chrono::system_clock::now();
    // We run mission  (non blocking)
    mission->execute(true);
    auto current_time = std::chrono::system_clock::now();
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_call_mission_time);

    EXPECT_NEAR(time_taken.count(),0,2.0);//This checks if run mission is a blocking call, should return quickly

    // We have an estimated time to reach goal, which is conservative as vehicles should reach it in
    // less time. 
    //
    // Just in case students have not implemented this correctly, we have an upper bound of 90s
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();
    double maxTime =180.0; // We impose a maximum time of 180s

    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded

    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //We have now taken time
            timeExceeded=true;
            OK=true;
        }

        //Let's check the status
        std::vector<unsigned int> status = mission->status();
        //Status shoud have two elements
        ASSERT_EQ(status.size(),controllers.size());


        // The status should be less than 100% for both platforms
        // Thus indicating still completing mission
        bool completed = true;
        for (auto st:status){
            if(st < 100){
                //mission accomplished
                completed=false;
            }
        }

        // If both platforms have completed the mission, we are done
        if(completed){
            OK=true;
        }
        else{
            //Let's slow down this loop to 200ms (5Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    ASSERT_FALSE(timeExceeded); // time should not be exceeded

    // We now check that the goal has been reached using the PfmsHog
    std::vector<double> distances;
    double timeTakenAck = maxTime; // We will use this variable to check the time taken for the Ackerman platform 
    double timeTakenSkid = maxTime; // We will use this variable to check the time taken for the Ackerman platform 
    {
        bool reachedCheck = pfmsHogAudiPtr->checkGoalsReached(distances,timeTakenAck);
        ASSERT_TRUE(reachedCheck);
    }
    {
        bool reachedCheck = pfmsHogHuskyPtr->checkGoalsReached(distances,timeTakenSkid);
        ASSERT_TRUE(reachedCheck);
    }

    // In race mode we will enable the next check which will determine if the max time was below 100s, which is a reasonable time to reach the goals given the distances and platforms   
    EXPECT_LT(timeTakenAck,100.0);
    EXPECT_LT(timeTakenSkid,100.0);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
