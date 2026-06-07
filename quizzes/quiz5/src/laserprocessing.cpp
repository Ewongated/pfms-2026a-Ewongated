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
  unsigned int count = 0;
  for (const auto& range : laserScan_.ranges) {
    if (std::isfinite(range) && range < laserScan_.range_max) {
      count++;
    }
  }
  return count;
}
//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
  unsigned int count = 0;
  bool inSegment = false;
  geometry_msgs::msg::Point prevPoint;

  for (unsigned int i = 0; i < laserScan_.ranges.size(); i++) {
    float range = laserScan_.ranges.at(i);

    if (!std::isfinite(range) || range >= laserScan_.range_max) {
      inSegment = false;
      continue;
    }

    geometry_msgs::msg::Point currPoint = polarToCart(i);

    if (!inSegment) {
      // Start of a new segment
      count++;
      inSegment = true;
    } else {
      // Check distance to previous point
      double dx = currPoint.x - prevPoint.x;
      double dy = currPoint.y - prevPoint.y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist >= 0.3) {
        // Gap — start a new segment
        count++;
      }
    }

    prevPoint = currPoint;
  }

  return count;
}



//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::msg::Point LaserProcessing::detectClosestCone()
{
    geometry_msgs::msg::Point point;
    
    // First, build segments (same logic as countSegments)
    std::vector<std::vector<geometry_msgs::msg::Point>> segments;
    std::vector<geometry_msgs::msg::Point> currentSegment;
    geometry_msgs::msg::Point prevPoint;
    bool inSegment = false;

    for (unsigned int i = 0; i < laserScan_.ranges.size(); i++) {
        float range = laserScan_.ranges.at(i);

        if (!std::isfinite(range) || range >= laserScan_.range_max) {
            if (inSegment && !currentSegment.empty()) {
                segments.push_back(currentSegment);
                currentSegment.clear();
            }
            inSegment = false;
            continue;
        }

        geometry_msgs::msg::Point currPoint = polarToCart(i);

        if (!inSegment) {
            currentSegment.push_back(currPoint);
            inSegment = true;
        } else {
            double dx = currPoint.x - prevPoint.x;
            double dy = currPoint.y - prevPoint.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist >= 0.3) {
                segments.push_back(currentSegment);
                currentSegment.clear();
            }
            currentSegment.push_back(currPoint);
        }
        prevPoint = currPoint;
    }
    if (!currentSegment.empty()) {
        segments.push_back(currentSegment);
    }

    // Compute mean position of each segment, find closest
    double minDist = std::numeric_limits<double>::max();

    for (const auto& seg : segments) {
        geometry_msgs::msg::Point mean;
        for (const auto& p : seg) {
            mean.x += p.x;
            mean.y += p.y;
        }
        mean.x /= seg.size();
        mean.y /= seg.size();

        double dist = std::sqrt(mean.x*mean.x + mean.y*mean.y);
        if (dist < minDist) {
            minDist = dist;
            point = mean;
        }
    }

    return point;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::msg::Point LaserProcessing::detectRoadCentre()
{
    geometry_msgs::msg::Point point;

    // Build segments (same logic)
    std::vector<std::vector<geometry_msgs::msg::Point>> segments;
    std::vector<geometry_msgs::msg::Point> currentSegment;
    geometry_msgs::msg::Point prevPoint;
    bool inSegment = false;

    for (unsigned int i = 0; i < laserScan_.ranges.size(); i++) {
        float range = laserScan_.ranges.at(i);

        if (!std::isfinite(range) || range >= laserScan_.range_max) {
            if (inSegment && !currentSegment.empty()) {
                segments.push_back(currentSegment);
                currentSegment.clear();
            }
            inSegment = false;
            continue;
        }

        geometry_msgs::msg::Point currPoint = polarToCart(i);

        if (!inSegment) {
            currentSegment.push_back(currPoint);
            inSegment = true;
        } else {
            double dx = currPoint.x - prevPoint.x;
            double dy = currPoint.y - prevPoint.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist >= 0.3) {
                segments.push_back(currentSegment);
                currentSegment.clear();
            }
            currentSegment.push_back(currPoint);
        }
        prevPoint = currPoint;
    }
    if (!currentSegment.empty()) {
        segments.push_back(currentSegment);
    }

    // Compute cone centres
    std::vector<geometry_msgs::msg::Point> cones;
    for (const auto& seg : segments) {
        geometry_msgs::msg::Point mean;
        for (const auto& p : seg) {
            mean.x += p.x;
            mean.y += p.y;
        }
        mean.x /= seg.size();
        mean.y /= seg.size();
        cones.push_back(mean);
    }

    // Find two cones on opposite sides of road (~8m apart)
    geometry_msgs::msg::Point c1, c2;
    double bestDist = std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < cones.size(); i++) {
        for (unsigned int j = i + 1; j < cones.size(); j++) {
            // Must be on opposite sides (y values have opposite signs)
            if (cones[i].y * cones[j].y >= 0) continue;

            double dx = cones[i].x - cones[j].x;
            double dy = cones[i].y - cones[j].y;
            double dist = std::sqrt(dx*dx + dy*dy);

            // Road is ~8m wide, allow some tolerance
            if (std::abs(dist - 8.0) < bestDist) {
                bestDist = std::abs(dist - 8.0);
                c1 = cones[i];
                c2 = cones[j];
            }
        }
    }

    // Road centre is midpoint
    point.x = (c1.x + c2.x) / 2.0;
    point.y = (c1.y + c2.y) / 2.0;

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
