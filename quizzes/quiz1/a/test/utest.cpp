#include "gtest/gtest.h"
#include <iostream> // Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

// Header files needed from our libraries
#include "../charger.h"
#include "../processing.h"
using namespace std;

TEST(ClassTest, CreateObject) {
    Charger charger("ModelX", 50, false);
    EXPECT_EQ(charger.getModel(), "ModelX");
    EXPECT_EQ(charger.getBatteryLevel(), 50);
    EXPECT_EQ(charger.getChargingStatus(), false);
}

TEST(FunctionTest, CheckRecharge) {
    Charger charger("ModelX", 65, false);
    EXPECT_EQ(charger.getChargingStatus(), false);
    EXPECT_EQ(charger.recharge(), true);
    EXPECT_EQ(charger.getBatteryLevel(), 90);
    EXPECT_EQ(charger.getChargingStatus(), true);
    EXPECT_EQ(charger.recharge(), false);
    EXPECT_EQ(charger.getBatteryLevel(), 100);
    EXPECT_EQ(charger.getChargingStatus(), false);
}

TEST(FunctionTest, DetectLowestBatteryLevel) {
    std::vector<Charger> fleet;
    fleet.push_back(Charger("ModelA", 32, true));
    fleet.push_back(Charger("ModelB", 62, false));
    fleet.push_back(Charger("ModelC", 72, true));
    fleet.push_back(Charger("ModelD", 82, false));
    fleet.push_back(Charger("ModelE", 42, true));

    std::vector<Charger> lowestBattery = lowestBatteryLevel(fleet);

    ASSERT_EQ(lowestBattery.size(), 1);
    EXPECT_EQ(lowestBattery.at(0).getModel(), "ModelA");

    // What happens if 2 chargers have the same battery level?
    fleet.push_back(Charger("ModelF", 32, true));
    lowestBattery = lowestBatteryLevel(fleet);

    ASSERT_EQ(lowestBattery.size(), 2);
    EXPECT_EQ(lowestBattery.at(0).getModel(), "ModelA");
    EXPECT_EQ(lowestBattery.at(1).getModel(), "ModelF");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
