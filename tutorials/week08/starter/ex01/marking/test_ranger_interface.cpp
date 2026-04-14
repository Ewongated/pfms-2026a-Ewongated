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

   // What functions should we test after creating an object Sonar class on two platforms?
   // What are the expected values for the functions that we are testing?
   // Refer to all functions in RangerInterface class and test them for both the Sonar objects that we have created. 

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

