#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "sonar.h"
#include "laser.h"
#include "pfms_types.h"
#include <cmath>
#include "test_helper.h"
#include "pfmshog.h"


using namespace std;
using namespace pfms::nav_msgs;



///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(RangerInterface, Simple) {

    {
       pfms::PlatformType platform = pfms::PlatformType::ACKERMAN;
       //! Created a pointer to PfmsHog 
       std::unique_ptr<PfmsHog> pfmsHogAckPtr = std::make_unique<PfmsHog>(platform);
       Odometry odo = populateOdo(1,4,0);
       pfmsHogAckPtr->teleport(odo);
       std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }


    {
       pfms::PlatformType platform = pfms::PlatformType::SKIDSTEER;
       //! Created a pointer to PfmsHog 
       std::unique_ptr<PfmsHog> pfmsHogSkidPtr = std::make_unique<PfmsHog>(platform);
       Odometry odo = populateOdo(-1,-4,0);
       pfmsHogSkidPtr->teleport(odo);
       std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }



    //Create a quadcopter and push back to controllers
    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Sonar(pfms::PlatformType::ACKERMAN));
    rangers.push_back(new Sonar(pfms::PlatformType::SKIDSTEER));

    ASSERT_EQ(rangers.size(),2);

    ASSERT_EQ(rangers.front()->getSensingMethod(),pfms::RangerType::CONE);
    ASSERT_NEAR(rangers.front()->getFieldOfView(),3*M_PI/180.0, 0.001f);
    ASSERT_FLOAT_EQ(rangers.front()->getMinRange(),0.02f);
    ASSERT_FLOAT_EQ(rangers.front()->getMaxRange(),20.0f);
    ASSERT_FLOAT_EQ(rangers.front()->getAngularResolution(),0.0f);
    Odometry odoA = rangers.front()->getSensorPose();
    ASSERT_NEAR(odoA.position.x,1.0+3.725,0.1);
    ASSERT_NEAR(odoA.position.y,4.0,0.1);

    ASSERT_EQ(rangers.back()->getSensingMethod(),pfms::RangerType::CONE);
    ASSERT_NEAR(rangers.back()->getFieldOfView(),3*M_PI/180.0, 0.001f);
    ASSERT_FLOAT_EQ(rangers.back()->getMinRange(),0.02f);
    ASSERT_FLOAT_EQ(rangers.back()->getMaxRange(),20.0f);
    ASSERT_FLOAT_EQ(rangers.back()->getAngularResolution(),0.0f);
    Odometry odoS = rangers.back()->getSensorPose();
    ASSERT_NEAR(odoS.position.x,-1.0,0.1);
    ASSERT_NEAR(odoS.position.y,-4.0,0.1);


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

