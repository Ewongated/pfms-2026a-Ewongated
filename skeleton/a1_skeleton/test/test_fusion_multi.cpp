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

TEST_F(FusionTest, TwoPlatforms) {

    //Setting up the scene
    Odometry odo = populateOdo(8.5, 20.5, -M_PI_2);
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

    
    Laser l1(pfms::PlatformType::ACKERMAN);
    Sonar s(pfms::PlatformType::ACKERMAN);
    Laser l2(pfms::PlatformType::SKIDSTEER);

    std::vector<RangerInterface *> sensors = { &s, &l1, &l2 };

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    Fusion fusion(sensors);
    std::vector<pfms::Cell*> cells;
    cells.push_back(new pfms::Cell(9.8,2,1));
    cells.push_back(new pfms::Cell(5,-2,1));
    cells.push_back(new pfms::Cell(9,9,1));
    cells.push_back(new pfms::Cell(10.0,10.0,1));
    cells.push_back(new pfms::Cell(10.7,11,1));
    cells.push_back(new pfms::Cell(8.8,2,1));
    fusion.setCells(cells);
    fusion.grabAndFuseData();
    
    EXPECT_EQ(pfms::cell::FREE, cells.at(0)->getState());// Corrected from OCCUPIED to FREE, Audi Laser sees it as free
    EXPECT_EQ(pfms::cell::FREE, cells.at(1)->getState());   
    EXPECT_EQ(pfms::cell::OCCUPIED, cells.at(2)->getState());
    EXPECT_EQ(pfms::cell::UNKNOWN, cells.at(3)->getState());
    EXPECT_EQ(pfms::cell::OCCUPIED, cells.at(4)->getState());
    EXPECT_EQ(pfms::cell::OCCUPIED, cells.at(5)->getState());// Corrected from FREE to OCCUPIED, Husky Laser sees it as Occupied
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
