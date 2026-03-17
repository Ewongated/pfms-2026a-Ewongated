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

class FusionTest : public ::testing::Test {
protected:
    void SetUp() override {
        pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    }

    void TearDown() override {
        // Clean up if needed
    }

    std::unique_ptr<PfmsHog> pfmsHogPtr;
};

TEST_F(FusionTest, SinglePlatform) {

    //Setting up the scene
    Odometry odo = populateOdo(0, 2, 0);
    pfmsHogPtr->teleport(odo);
    pfms::geometry_msgs::Point pX {100, 100, 1.05};
    pfmsHogPtr->teleportObject(pX, "box2");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pfms::geometry_msgs::Point p1 {10, 2.0, 0.25};
    pfmsHogPtr->teleportObject(p1, "box1");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pfms::geometry_msgs::Point p2 {10, 2.0, 1.05};
    pfmsHogPtr->teleportObject(p2, "box2");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
    Laser l(type);
    Sonar s(type);

    std::vector<RangerInterface *> sensors = { &s, &l };

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    Fusion fusion(sensors);
    std::vector<pfms::Cell*> cells;
    cells.push_back(new pfms::Cell(9.8,2,1));
    cells.push_back(new pfms::Cell(5,-2,1));
    cells.push_back(new pfms::Cell(9,9,1));
    cells.push_back(new pfms::Cell(11,12,1));
    cells.push_back(new pfms::Cell(8.8,2,1));
    cells.push_back(new pfms::Cell(4,4,1));
    fusion.setCells(cells);
    fusion.grabAndFuseData();
    
    EXPECT_EQ(pfms::cell::OCCUPIED, cells.at(0)->getState());
    EXPECT_EQ(pfms::cell::FREE, cells.at(1)->getState());
    EXPECT_EQ(pfms::cell::OCCUPIED, cells.at(2)->getState());
    EXPECT_EQ(pfms::cell::UNKNOWN, cells.at(3)->getState());
    EXPECT_EQ(pfms::cell::OCCUPIED, cells.at(4)->getState());
    EXPECT_EQ(pfms::cell::FREE, cells.at(5)->getState());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
