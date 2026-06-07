#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "skidsteer.h" // The skidsteer
#include "pfms_types.h" //A1 types
#include "test_helper.h" // Helper header that assembled the message
#include "pfmshog.h" // Controlling the simulator
#include "laserprocessing.h" // Processing the laser scan data


using namespace pfms::nav_msgs;


TEST(SkidSteerTest, CheckObstaclesMoving) {

    //We create the PfmHog object pointer and use it to set initial pose of Skidsteer for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    Odometry initial_odo = populateOdo(-1.5, -71.0, 0.1825, 0);
    pfmsHogPtr->teleport(initial_odo);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    //Setting up the scene
    pfms::geometry_msgs::Point p1 {3, -65.0, 1.3};
    pfmsHogPtr->teleportObject(p1, "box1");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // We create the controller and set the goal to move in straight line in X direction
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.5);

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ initial_odo.position.x + 2.0, initial_odo.position.y, 0.0});
    
    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoals(goals);

    //We send the goals to the controller
    controllers.front()->setGoals(goals);

    // We will loop until that time and if the goal is not reached until then (or we have status)
    // indicating IDLE, we know it has been reached, we use a max time of 180s to reach it
    auto start_time = std::chrono::system_clock::now();
    double maxTime = 180.0;

    // The below should not block and we will be back and can check progress
    controllers.front()->execute(true);
    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded
    unsigned int currentGoalIndex;

    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //We have now taken time too much time, and we terminate 
            timeExceeded=true;
            break;
        }

        pfms::PlatformStatus status = controllers.front()->status(currentGoalIndex);

        if(status == pfms::PlatformStatus::IDLE){
            OK=true; // mission accomplished
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    ASSERT_FALSE(timeExceeded); // time should not be exceeded
    //We now check that the goal has been reached using the PfmsHog
    std::vector<double> distances;
    bool reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
    ASSERT_TRUE(reachedCheck);

    //We can also check the distance to the goal reported by PfmsHog
    ASSERT_GE(distances.size(),1);
    ASSERT_NEAR(distances.at(0),0,1.0);

    std::vector<pfms::geometry_msgs::Point> obstacles = controllers.front()->getObstacles();

    ASSERT_EQ(obstacles.size(),1); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).x,p1.x,0.2); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).y,p1.y,0.2); // This will be the size of the laser scan of obstacles


    // The below should not block, a call to run() should return promptly.
    controllers.front()->execute(false);  

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
