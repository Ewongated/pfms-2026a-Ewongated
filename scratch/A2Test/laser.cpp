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

    // Compute world-frame sensor pose from current platform odometry
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);

    pfms::nav_msgs::Odometry pose = odo;

    if (type_ == pfms::PlatformType::ACKERMAN) {
        // Offset sensor forward of rear axle
        pose.position.x += ACKERMAN_OFFSET * std::cos(odo.yaw);
        pose.position.y += ACKERMAN_OFFSET * std::sin(odo.yaw);
    }
    // Husky: sensor at platform centre — pose == odo

    {
        std::lock_guard<std::mutex> lock(mutex_);
        sensorPose_ = pose;
    }

    std::vector<double> ranges;
    ranges.reserve(scan.ranges.size());
    for (const float r : scan.ranges) {
        if (r >= static_cast<float>(minRange_) && r <= static_cast<float>(maxRange_)) {
            ranges.push_back(static_cast<double>(r));
        } else {
            ranges.push_back(0.0);
        }
    }
    return ranges;
}
