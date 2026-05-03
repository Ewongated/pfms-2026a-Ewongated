#include "laser.h"
#include <cmath>

/**
 * @brief Constructs a Laser for the given platform and sets sensor geometry.
 */
Laser::Laser(pfms::PlatformType type)
    : Ranger(type)
{
    fov_               = FOV_DEG;
    angularResolution_ = ANG_RES_DEG;
    maxRange_          = MAX_RANGE_M;
    minRange_          = MIN_RANGE_M;
    sensingMethod_     = pfms::RangerType::POINT;
}

/**
 * @brief Reads one laser scan from the simulator.
 *
 * Calls PfmsConnector::read(LaserScan). Uses the accompanying platform
 * odometry to compute the sensor pose in world frame:
 * - Husky: sensor at platform centre.
 * - Audi R8: sensor 3.725 m forward of rear-axle centre along platform heading.
 *
 * Ranges outside [minRange_, maxRange_] are replaced with 0.0 so callers
 * can easily discard them (< minRange_ or == 0).
 *
 * @return Vector of range readings [m] (0.0 = invalid)
 */
std::vector<double> Laser::getData()
{
    pfms::sensor_msgs::LaserScan scan;
    pfmsConnectorPtr_->read(scan);

    // Update geometry from actual scan data so isPathObstructed uses correct values
    // and compute sensor pose — both under a single lock
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);

    pfms::nav_msgs::Odometry pose = odo;

    if (type_ == pfms::PlatformType::ACKERMAN) {
        pose.position.x += ACKERMAN_OFFSET * std::cos(odo.yaw);
        pose.position.y += ACKERMAN_OFFSET * std::sin(odo.yaw);
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        angularResolution_ = scan.angle_increment * 180.0 / M_PI;
        fov_               = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;
        maxRange_          = scan.range_max;
        minRange_          = scan.range_min;
        angleMin_          = scan.angle_min;
        sensorPose_        = pose;
    }

    std::vector<double> ranges;
    ranges.reserve(scan.ranges.size());
    for (const float r : scan.ranges) {
        if (r >= static_cast<float>(scan.range_min) && r <= static_cast<float>(scan.range_max)) {
            ranges.push_back(static_cast<double>(r));
        } else {
            ranges.push_back(0.0);
        }
    }

    // Debug: print raw scan values around centre
    std::cout << "[getData] ranges.size()=" << ranges.size()
              << " angle_min=" << scan.angle_min
              << " angle_inc=" << scan.angle_increment
              << " range_min=" << scan.range_min
              << " range_max=" << scan.range_max << std::endl;
    unsigned int centre = ranges.size() / 2;
    for (unsigned int i = centre - 10; i <= centre + 10 && i < ranges.size(); ++i) {
        std::cout << "[getData] ray[" << i << "]=" << scan.ranges[i]
                  << " filtered=" << ranges[i] << std::endl;
    }

    return ranges;
}