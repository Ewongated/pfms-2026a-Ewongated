#include "laser.h"
#include <cmath>

/**
 * @brief Constructs a Laser for the given platform and sets default geometry.
 *
 * Geometry constants are set here as defaults. getData() overwrites them
 * with values read directly from the simulator's scan message so the object
 * always reflects the actual sensor configuration.
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

std::vector<double> Laser::getData()
{
    pfms::sensor_msgs::LaserScan scan;
    pfmsConnectorPtr_->read(scan);

    // Read the current platform odometry to compute the sensor's world-frame pose.
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);

    pfms::nav_msgs::Odometry pose = odo;

    if (type_ == pfms::PlatformType::ACKERMAN) {
        // Offset the sensor forward of the rear axle along the platform heading.
        pose.position.x += ACKERMAN_OFFSET * std::cos(odo.yaw);
        pose.position.y += ACKERMAN_OFFSET * std::sin(odo.yaw);
    }
    // Husky: sensor is at the platform centre — no offset required.

    {
        // Update all geometry fields from the raw scan so isPathObstructed()
        // uses the correct angleMin_ and angular resolution.
        std::lock_guard<std::mutex> lock(mutex_);
        angularResolution_ = scan.angle_increment * 180.0 / M_PI;
        fov_               = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;
        maxRange_          = scan.range_max;
        minRange_          = scan.range_min;
        angleMin_          = scan.angle_min;
        sensorPose_        = pose;
    }

    // Build the output range vector. Readings outside [minRange_, maxRange_]
    // are set to 0.0 so callers can identify them with a simple zero-check.
    std::vector<double> ranges;
    ranges.reserve(scan.ranges.size());
    for (const float r : scan.ranges) {
        if (r >= static_cast<float>(scan.range_min) && r <= static_cast<float>(scan.range_max)) {
            ranges.push_back(static_cast<double>(r));
        } else {
            ranges.push_back(0.0);
        }
    }
    return ranges;
}
