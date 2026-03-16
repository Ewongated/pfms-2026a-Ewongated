#include "pfmshog.h"
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {

    pfms::PlatformType platform = pfms::PlatformType::ACKERMAN;
    std::vector<double> distances;

    //! Created a pointer to PfmsCommander 
    std::shared_ptr<PfmsHog> pfmsHogPtr = std::make_shared<PfmsHog>(platform,true);

    pfms::geometry_msgs::Point p1;
    p1.x = 3.0;
    p1.y = 2.0;
    p1.z = 0.0;

    // We teleport the object to the location specified in the point
    // the change is instateneous.
    pfmsHogPtr->teleportObject(p1, "orange_audibot");


    pfms::geometry_msgs::Point p2;
    p2.x = 3.725;
    p2.y = 5.0;
    p2.z = 0.2;

    // We can also move box1, box2 and box3 as well as husky (if it is available)
    pfmsHogPtr->teleportObject(p2, "crash_dummy");

    return 0;
}
