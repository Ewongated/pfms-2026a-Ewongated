#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <cmath>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan), objectReadings_(0)
{
}



unsigned int LaserProcessing::countSegments()
{
    std::unique_lock<std::mutex> lck(mtx);
    sensor_msgs::msg::LaserScan laserScan = laserScan_;   
    lck.unlock();

    unsigned int segmentCount = 0;
    bool inSegment = false;
    geometry_msgs::msg::Point previousPoint;
    const double DISTANCE_THRESHOLD = 1.0; // 0.5 meters
    
    for (unsigned int i = 0; i < laserScan.ranges.size(); ++i)
    {
        if ((laserScan.ranges.at(i) > laserScan.range_min) &&
            (laserScan.ranges.at(i) < laserScan.range_max) &&
            !isnan(laserScan.ranges.at(i)) &&
            isfinite(laserScan.ranges.at(i)))
        {
            // Valid reading - convert to Cartesian
            geometry_msgs::msg::Point currentPoint = polarToCart(i);
            bool startNewSegment = false;
            
            if (!inSegment) {
                // First valid reading after invalid - start new segment
                startNewSegment = true;
            }
            else {
                // Check Euclidean distance from previous point
                double dx = currentPoint.x - previousPoint.x;
                double dy = currentPoint.y - previousPoint.y;
                double distance = std::sqrt(dx * dx + dy * dy);
                
                if (distance > DISTANCE_THRESHOLD) {
                    // Large jump detected - start new segment
                    startNewSegment = true;
                    inSegment = false;
                }
            }
            
            if (startNewSegment) {
                segmentCount++;
                inSegment = true;
            }
            
            previousPoint = currentPoint;
        }
        else
        {
            // Invalid reading (nan, infinity, zero, or out of range) - end current segment
            inSegment = false;
        }
    }

    return segmentCount;
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    std::unique_lock<std::mutex> lck(mtx);
    laserScan_ = laserScan;    
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
