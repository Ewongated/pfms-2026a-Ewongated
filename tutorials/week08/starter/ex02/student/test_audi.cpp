#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "pfms_types.h"
#include <cmath>


// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "ackerman_test.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////


TEST_F(AckermanTest, NotPossible) {

    // Starting point for vehicle
    Odometry odo = populateOdo(0,2,0);
    pfmsHogPtr_->teleport(odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({0,-6.0});

    bool reachable = controllers.at(0)->setGoals(goals);
    ASSERT_FALSE(reachable);

    std::cout << "Ackerman: can not reach goal " << std::endl;

}


TEST_F(AckermanTest, ForwardFacing) {

    // Starting point for vehicle
    Odometry odo = populateOdo(0.225,-5.306,127.825*M_PI/180);
    pfmsHogPtr_->teleport(odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=1,y=4;//This shoudl possible
    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({1,4});

    bool reachable = controllers.at(0)->setGoals(goals);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;


    ASSERT_NEAR(dist,10.2568,0.5);
    ASSERT_NEAR(t,3.53684,1.0);
}


TEST_F(AckermanTest, StraightLine) {

    //! @todo
    //! Create a unit test that checks straight line
    // Starting point for vehicle
    Odometry odo = populateOdo(0,0,0);
    pfmsHogPtr_->teleport(odo);

}

TEST_F(AckermanTest, YourTest) {

    //! @todo
    //! Create a unit test 
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
