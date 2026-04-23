#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(std::shared_ptr<PfmsConnector> pfmsConnectorPtr):
    pfmsConnectorPtr_(pfmsConnectorPtr),
    seq_(0), minRange_(0.25), maxRange_(15.0)
{
   
}


std::vector<pfms::geometry_msgs::Point> LaserProcessing::getObstacles(void)
{
    // We need to read the laser scan data
    newScan();

    std::vector<pfms::geometry_msgs::Point> obstacles;

    for (unsigned int i=0; i<laserScan_.ranges.size(); i++) {
        if (laserScan_.ranges.at(i) > minRange_ &&
            laserScan_.ranges.at(i) < maxRange_) {
            double angle = laserScan_.angle_min + i * laserScan_.angle_increment;
            double x = laserScan_.ranges.at(i) * cos(angle);
            double y = laserScan_.ranges.at(i) * sin(angle);
            obstacles.push_back({x, y, 0});
        }
    }

    //! @todo TASK 1: Write the code to detect the obstacles in the laser scan (which is now in obstacles)
    //! Recall that this is part of a square and you may see one side of it or part of two sides

    
    obstacles.clear();
    return obstacles;
}


void LaserProcessing::newScan(){
    // We need to read the laser scan data
    bool success = false;
    for (int attempts = 0; attempts < 3 && !success; attempts++) {
        success = pfmsConnectorPtr_->read(laserScan_);
        // When we use ROS we can send error messages to the ROS console
    }
}

