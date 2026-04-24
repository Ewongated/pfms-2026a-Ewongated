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

    std::vector<std::vector<double>> data = fusion.getRawRangeData();

    // Data should be empty before grabAndFuseData() is called
    EXPECT_TRUE(data.empty());


    std::vector<pfms::Cell*> cells;
    cells.push_back(new pfms::Cell(9.8,2,1));
    cells.push_back(new pfms::Cell(5,-2,1));
    cells.push_back(new pfms::Cell(9,9,1));
    cells.push_back(new pfms::Cell(10.0,10.0,1));
    cells.push_back(new pfms::Cell(10.7,11,1));
    cells.push_back(new pfms::Cell(8.8,2,1));
    fusion.setCells(cells);
    fusion.grabAndFuseData();
    
    // After grabAndFuseData(), data should not be empty
    data = fusion.getRawRangeData();
    EXPECT_FALSE(data.empty());
    EXPECT_EQ(4, data.size()); // Should have data from all 4 sensors
    
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
