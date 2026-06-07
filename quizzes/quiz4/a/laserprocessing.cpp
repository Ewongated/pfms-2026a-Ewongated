#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

LaserProcessing::LaserProcessing(std::shared_ptr<PfmsConnector> pfmsConnectorPtr):
    pfmsConnectorPtr_(pfmsConnectorPtr),
    seq_(0), minRange_(0.25), maxRange_(15.0)
{
}

std::vector<pfms::geometry_msgs::Point> LaserProcessing::getObstacles(void)
{
    newScan();
    std::vector<pfms::geometry_msgs::Point> validPoints;

    for (unsigned int i = 0; i < laserScan_.ranges.size(); i++) {
        float r = laserScan_.ranges.at(i);
        if (r > minRange_ && r < maxRange_) {
            double angle = laserScan_.angle_min + i * laserScan_.angle_increment;
            validPoints.push_back({ r * cos(angle), r * sin(angle), 0 });
        }
    }

    std::vector<pfms::geometry_msgs::Point> obstacles;
    if (validPoints.empty()) return obstacles;

    double xMin = validPoints[0].x, xMax = validPoints[0].x;
    double yMin = validPoints[0].y, yMax = validPoints[0].y;
    for (auto& p : validPoints) {
        xMin = std::min(xMin, p.x);
        xMax = std::max(xMax, p.x);
        yMin = std::min(yMin, p.y);
        yMax = std::max(yMax, p.y);
    }

    obstacles.push_back({ (xMin + xMax) / 2.0,
                          (yMin + yMax) / 2.0,
                          0 });
    return obstacles;
}

void LaserProcessing::newScan()
{
    bool success = false;
    for (int attempts = 0; attempts < 3 && !success; attempts++) {
        success = pfmsConnectorPtr_->read(laserScan_);
    }
}