#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

// header files needed from our libraries, because of include_directories in CMakeLists.txt we don't need the ..
// before these filenames
#include "tf2.h"
#include "tf.h"
#include "types.h"
#include "analysis.h"
using namespace std;
using geometry_msgs::Pose;
using geometry_msgs::RangeBearingStamped;
using std::vector;


TEST (Analysis, ExportGraph ) {

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(M_PI/4);

    vector<Point> goals;
    goals.push_back({771.59,1451.77,0});
    goals.push_back({1715.77,-1527.37,0});
    goals.push_back({-3026,464.671,0});
    goals.push_back({-2066.27,-3301.39,0});
    goals.push_back({-4100.31,-1899.05,0});

    Analysis analysis(goals);
    AdjacencyList graph = analysis.exportGraph();

    ASSERT_EQ(graph.size(),goals.size());
    ASSERT_EQ(graph.at(0).size(),goals.size()-1);
    ASSERT_EQ(graph.at(1).size(),goals.size()-1);
    ASSERT_EQ(graph.at(2).size(),goals.size()-1);
    ASSERT_EQ(graph.at(3).size(),goals.size()-1);
    ASSERT_EQ(graph.at(4).size(),goals.size()-1);

    //Random checks here 0 connects to 1 and 1 to zero
    ASSERT_EQ(graph.at(0).at(0).second,1);
    ASSERT_EQ(graph.at(1).at(0).second,0);
    ASSERT_NEAR(graph.at(1).at(0).first,3125.18,1.0);
    ASSERT_NEAR(graph.at(0).at(0).first,3125.18,1.0);

    //Random checks here 0 connects to 4 and 4 to zero
    ASSERT_EQ(graph.front().back().second,4);
    ASSERT_EQ(graph.back().front().second,0);
    ASSERT_NEAR(graph.front().back().first,5912.99,1.0);
    ASSERT_NEAR(graph.back().front().first,5912.99,1.0);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
