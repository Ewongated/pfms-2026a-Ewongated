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
#include "fusion.h"

using namespace std;

class FusionTests : public ::testing::Test {
protected:
    void SetUp() override {
        pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    }

    void TearDown() override {
        // Clean up if needed
    }

    std::unique_ptr<PfmsHog> pfmsHogPtr;
};

TEST_F(FusionTests, RawData) {
  Odometry odo = populateOdo(0, 2, 0);
  pfmsHogPtr->teleport(odo);
  
  pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
  Laser l(type);
  Sonar s(type);

  std::vector<RangerInterface *> sensors = { &s, &l };

  Fusion fusion(sensors);

  fusion.grabAndFuseData();
  auto rawdata = fusion.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(1, rawdata.at(0).size());
  EXPECT_EQ(640, rawdata.at(1).size());

  std::vector<pfms::Cell*> cells;
  cells.push_back(new pfms::Cell(-10,0,1));
  cells.push_back(new pfms::Cell(-11,0,1));

  fusion.setCells(cells);
  fusion.grabAndFuseData();

  ASSERT_EQ(2, cells.size());
  ASSERT_EQ(pfms::cell::UNKNOWN, cells.at(0)->getState());
  ASSERT_EQ(pfms::cell::UNKNOWN, cells.at(1)->getState());

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
