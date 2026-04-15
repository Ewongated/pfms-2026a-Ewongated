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
    ASSERT_EQ(sonar->getSensingMethod(),pfms::RangerType::CONE);
    ASSERT_NEAR(sonar->getFieldOfView(),3*M_PI/180.0, 0.001f);
    ASSERT_FLOAT_EQ(sonar->getMinRange(),0.02f);
    ASSERT_FLOAT_EQ(sonar->getMaxRange(),20.0f);
    ASSERT_FLOAT_EQ(sonar->getAngularResolution(),0.0f);
    delete sonar;
}

TEST(ConstructorTest, SonarAckerman) {

    //Create a quadcopter and push back to controllers
    Sonar* sonar = new Sonar(pfms::PlatformType::ACKERMAN);

    ASSERT_EQ(sonar->getSensingMethod(),pfms::RangerType::CONE);
    ASSERT_NEAR(sonar->getFieldOfView(),3*M_PI/180.0, 0.001f);
    ASSERT_FLOAT_EQ(sonar->getMinRange(),0.02f);
    ASSERT_FLOAT_EQ(sonar->getMaxRange(),20.0f);
    ASSERT_FLOAT_EQ(sonar->getAngularResolution(),0.0f);
    delete sonar;
}

TEST(ConstructorTest, LaserSkidSteer) {
    //Create a quadcopter and push back to controllers
    Laser* laser = new Laser(pfms::PlatformType::SKIDSTEER);
    ASSERT_EQ(laser->getSensingMethod(),pfms::RangerType::POINT);
    ASSERT_NEAR(laser->getFieldOfView(),360.0, 0.1f);
    ASSERT_FLOAT_EQ(laser->getMinRange(),0.2f);
    ASSERT_FLOAT_EQ(laser->getMaxRange(),40.0f);
    ASSERT_NEAR(laser->getAngularResolution(),1.0f,0.01f);
    delete laser;
}

TEST(ConstructorTest, LaserAckerman) {
    //Create a quadcopter and push back to controllers
    Laser* laser = new Laser(pfms::PlatformType::ACKERMAN);
    ASSERT_EQ(laser->getSensingMethod(),pfms::RangerType::POINT);
    ASSERT_NEAR(laser->getFieldOfView(),180.0, 0.1f);
    ASSERT_FLOAT_EQ(laser->getMinRange(),0.1f);
    ASSERT_FLOAT_EQ(laser->getMaxRange(),30.0f);
    ASSERT_NEAR(laser->getAngularResolution(),0.28f,0.01f);
    delete laser;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
