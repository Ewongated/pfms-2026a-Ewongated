#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <iostream>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan)
{
}



//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countObjectReadings()
{

  unsigned int count=0;
  return count;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
  unsigned int count=0;
  return count;
}



//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::msg::Point LaserProcessing::detectClosestCone(){

    geometry_msgs::msg::Point point;
    return point;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::msg::Point LaserProcessing::detectRoadCentre(){

    geometry_msgs::msg::Point point;
    return point;
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    laserScan_=laserScan;
    cones_.clear();
}


geometry_msgs::msg::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}
