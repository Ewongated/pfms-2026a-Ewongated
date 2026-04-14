#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "laser.h"
#include "sonar.h"
#include "pfms_types.h"
#include <cmath>

using namespace pfms::nav_msgs;


TEST(ConstructorTest, SonarSkidSteer) {
    //Create a quadcopter and push back to controllers
    Sonar* sonar = new Sonar(pfms::PlatformType::SKIDSTEER);
    // What functions should we test after creating an object of Sonar class?
    // What are the expected values for the functions that we are testing?

    delete sonar;
}

TEST(ConstructorTest, SonarAckerman) {

    //Create a quadcopter and push back to controllers
    Sonar* sonar = new Sonar(pfms::PlatformType::ACKERMAN);

    // What functions should we test after creating an object of Sonar class?
    // What are the expected values for the functions that we are testing?

    delete sonar;
}

TEST(ConstructorTest, LaserSkidSteer) {
    //Create a quadcopter and push back to controllers
    Laser* laser = new Laser(pfms::PlatformType::SKIDSTEER);
    // What functions should we test after creating an object of Laser class?
    // What are the expected values for the functions that we are testing?

    delete laser;
}

TEST(ConstructorTest, LaserAckerman) {
    //Create a quadcopter and push back to controllers
    Laser* laser = new Laser(pfms::PlatformType::ACKERMAN);
    // What functions should we test after creating an object of Laser class?
    // What are the expected values for the functions that we are testing?

    delete laser;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
