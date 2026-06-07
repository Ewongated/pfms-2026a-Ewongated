#include "gtest/gtest.h"
#include <iostream> // Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

// Header files needed from our libraries
#include "../charger.h"
#include "../processing.h"
using namespace std;
 
TEST(FunctionTest, DetectEligibleForRecharge) {
    std::vector<Charger> fleet;
    fleet.push_back(Charger("ModelA", 49, true));
    fleet.push_back(Charger("ModelB", 62, false));
    fleet.push_back(Charger("ModelC", 72, true));
    fleet.push_back(Charger("ModelD", 82, false));
    fleet.push_back(Charger("ModelE", 42, false));

    // Now we can test the eligibleForRecharge function.
    unsigned int batteryCutoff = 50;
    std::vector<Charger> eligible = eligibleForRecharge(fleet, batteryCutoff);

    ASSERT_EQ(eligible.size(), 1);
    EXPECT_EQ(eligible.at(0).getModel(), "ModelE");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
