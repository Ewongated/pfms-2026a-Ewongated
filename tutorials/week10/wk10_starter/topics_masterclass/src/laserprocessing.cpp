#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan)
{
}

//! @todo
//! TASK 1 - Refer to Header file for full description
geometry_msgs::msg::Point LaserProcessing::closestPoint()
{
  geometry_msgs::msg::Point pt;


  return pt;
}


void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    laserScan_=laserScan;
}

