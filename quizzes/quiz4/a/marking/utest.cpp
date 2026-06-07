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

TEST(SkidsteerTest, Constructor) {

    //We create the PfmHog object pointer and use it to set initial pose of Skidsteer for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    Odometry initial_odo = populateOdo(0,0,0,0);
    pfmsHogPtr->teleport(initial_odo);
    
    //Create a skidsteer and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    double tolerance =0.2;
    unsigned int currentGoalIndex;

    
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceToGoal(),0);//Should be zero, even if we have no goals
    EXPECT_FLOAT_EQ(controllers.at(0)->timeToGoal(),0); //Should be zero, as we have no goals
    EXPECT_FLOAT_EQ(controllers.at(0)->timeTravelled(),0); //Should be zero, we have not travelled
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceTravelled(),0); //Should be zero, we hae not travelled
    EXPECT_EQ(controllers.at(0)->getPlatformType(),pfms::PlatformType::SKIDSTEER);//Skidsteer type
    EXPECT_EQ(controllers.at(0)->status(currentGoalIndex),pfms::PlatformStatus::IDLE);//Should be IDLE, we have not started
    EXPECT_TRUE(controllers.at(0)->setTolerance(0.2)); //Should be true, we can set this tolerance
    EXPECT_FALSE(controllers.at(0)->setTolerance(-1)); //Should be false, we can not set negative tolerance

    //If we call getOdometry we expect it to retrun the location we set as initial odometry
    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    EXPECT_NEAR(odo.position.x,initial_odo.position.x,tolerance); 
    EXPECT_NEAR(odo.position.y,initial_odo.position.y,tolerance);
    EXPECT_NEAR(odo.position.z,initial_odo.position.z,tolerance);
}

TEST(SkidsteerTest, CheckObstacles) {

    //We create the PfmHog object pointer and use it to set initial pose of Skidsteer for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    Odometry initial_odo = populateOdo(0.5, -71.0, 0.1825, 0);
    pfmsHogPtr->teleport(initial_odo);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    //Setting up the scene
    pfms::geometry_msgs::Point p1 {3, -65.0, 1.3};
    pfmsHogPtr->teleportObject(p1, "box1");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));


    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<pfms::geometry_msgs::Point> obstacles = controllers.front()->getObstacles();

    //Print the obstacles
    std::cout << "Obstacles: " << obstacles.size() <<  std::endl;
    for (const auto& obstacle : obstacles) {
        std::cout << "Obstacle at (" << obstacle.x << ", " << obstacle.y << ", " << obstacle.z << ")" << std::endl;
    }

    ASSERT_EQ(obstacles.size(),1); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).x,p1.x,0.2); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).y,p1.y,0.2); // This will be the size of the laser scan of obstacles

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

}

TEST(SkidsteerTest, StartingLogic) {

    //We create the PfmHog object pointer and use it to set initial pose of Skidsteer for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::SKIDSTEER);
    Odometry initial_odo = populateOdo(0,0,0,0);
    pfmsHogPtr->teleport(initial_odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.front()->setTolerance(0.7);
    unsigned int currentGoalIndex;

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ 20, -2.0, 0});
    
    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoals(goals);

    // Before we set the goal the platform shoudl be IDLE
    EXPECT_EQ(controllers.at(0)->status(currentGoalIndex),pfms::PlatformStatus::IDLE);//Should be IDLE, we have not started

    // We send the goals to the controller
    bool reachable = controllers.front()->setGoals(goals);

    // The goal should be reachable
    EXPECT_TRUE(reachable);

    // The platform should still be IDLE as we have not started the controller via run()
    EXPECT_EQ(controllers.at(0)->status(currentGoalIndex),pfms::PlatformStatus::IDLE);

    // As we are employing threading, we want to check that the run call does not block
    auto start_time = std::chrono::system_clock::now();

    // The below should not block, a call to run() should return promptly.
    controllers.front()->execute(true);

    // We can check that the run() call has returned by checking the time taken
    auto current_time = std::chrono::system_clock::now();
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

    // We should have taken less than 2 seconds to start the controller, this is a conservative estimate
    EXPECT_LT(time_taken.count(),2.0);

    // Now we should be in RUNNING state
    EXPECT_EQ(controllers.at(0)->status(currentGoalIndex),pfms::PlatformStatus::RUNNING);

    //Let's pause for two seconds to allow the controller to start and it moving
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Given we called run(), two seconds ago, we should have a goal to reach and have started moving
    EXPECT_GT(controllers.at(0)->distanceToGoal(),0); // Should be greater than 0
    EXPECT_GT(controllers.at(0)->timeToGoal(),0); // Should be greater than 0
    EXPECT_GT(controllers.at(0)->distanceTravelled(),0); // Should be greater than 0
    EXPECT_GT(controllers.at(0)->timeTravelled(),0); // Should be greater than 0
    
    // The paltform should be in motion
    //If we call getOdometry we expect it to retrun the location different to initial odometry
    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    double tolerance =0.2;//It should have moved at least 0.2m in X direction

    std::cout << "Initial Odo: (" << initial_odo.position.x << ", " << initial_odo.position.y << ", " << initial_odo.position.z << ")" << std::endl;
    std::cout << "Current Odo: (" << odo.position.x << ", " << odo.position.y << ", " << odo.position.z << ")" << std::endl;

    //We check that we have moved at least tolerance distance
    double distance = sqrt(pow(odo.position.x - initial_odo.position.x, 2) + 
                          pow(odo.position.y - initial_odo.position.y, 2));
    std::cout << "Distance travelled: " << distance << "m" << std::endl;
    EXPECT_GT(distance, tolerance);   

    // // The below should not block, a call to run() should return promptly.
    controllers.front()->execute(false);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
