#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "sonar.h"
#include "laser.h"
#include "fusion.h"

#include "pfms_types.h"
#include <cmath>

///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(FusionInterfaceTest, MultiSensors) {

    //Setting up the scene
    Laser l1(pfms::PlatformType::ACKERMAN);
    Sonar s1(pfms::PlatformType::ACKERMAN);
    Laser l2(pfms::PlatformType::SKIDSTEER);
    Sonar s2(pfms::PlatformType::SKIDSTEER);

    std::vector<RangerInterface *> sensors = { &l1, &l2, &s1, &s2 };

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    Fusion fusion(sensors);

    // What functions should we test after creating an object of Fusion class?
    // What are the expected values for the functions that we are testing?
    // Look at FusionInterface and test all the functions for the fusion object that we have created.
    
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
