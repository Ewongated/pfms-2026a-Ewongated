#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

// Some helper header for assembling messages and testing
#include "test_helper.h"

//Student defined libraries
#include "laser.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"
#include "pfmshog.h"

using namespace std;

class SensorsTest : public ::testing::Test {
protected:
    void SetUp() override {
        pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    }

    void TearDown() override {
        // Clean up if needed
    }

    std::unique_ptr<PfmsHog> pfmsHogPtr;
};

TEST_F(SensorsTest, Sonar) {

  Odometry odo = populateOdo(0, 2, 0);
  pfmsHogPtr->teleport(odo);

  pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
  Sonar s(type);

  auto rawdata = s.getData();

  ASSERT_EQ(1, rawdata.size());
  pfms::nav_msgs::Odometry pose =  s.getSensorPose();
  EXPECT_NEAR(3.725,pose.position.x,0.05);
  EXPECT_NEAR(2.0,pose.position.y,0.05);
  EXPECT_NEAR(0.0,pose.yaw,0.05);
  EXPECT_NEAR(0.0, s.getAngularResolution(), 0.0);
  EXPECT_NEAR(0.052, s.getFieldOfView(), 0.01);
  EXPECT_NEAR(20.0, s.getMaxRange(), 0.05);
  EXPECT_NEAR(0.02, s.getMinRange(), 0.01);
  EXPECT_EQ(pfms::RangerType::CONE, s.getSensingMethod());

}

TEST_F (SensorsTest, Laser1) {
  Odometry odo = populateOdo(0, 2, 0);
  pfmsHogPtr->teleport(odo);

  pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
  Laser l(type);

  auto rawdata = l.getData();

  ASSERT_EQ(640, rawdata.size());
  pfms::nav_msgs::Odometry pose =  l.getSensorPose();
  EXPECT_NEAR(3.725,pose.position.x,0.05);
  EXPECT_NEAR(2.0,pose.position.y,0.05);
  EXPECT_NEAR(0.0,pose.yaw,0.05);
  EXPECT_NEAR(0.281, l.getAngularResolution(), 0.01);
  EXPECT_NEAR(180.0, l.getFieldOfView(), 0.1);
  EXPECT_NEAR(30.0, l.getMaxRange(), 0.05);
  EXPECT_NEAR(0.1, l.getMinRange(), 0.05);
  EXPECT_EQ(pfms::RangerType::POINT, l.getSensingMethod());
}

TEST_F (SensorsTest, Laser2) {
  Odometry odo = populateOdo(10, 7, -M_PI_4);
  pfmsHogPtr->teleport(odo);

  pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
  Laser l(type);

  auto rawdata = l.getData();

  ASSERT_EQ(640, rawdata.size());
  pfms::nav_msgs::Odometry pose =  l.getSensorPose();
  EXPECT_NEAR(12.639,pose.position.x,0.05);
  EXPECT_NEAR(4.368,pose.position.y,0.05);
  EXPECT_NEAR(-M_PI_4,pose.yaw,0.05);
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
