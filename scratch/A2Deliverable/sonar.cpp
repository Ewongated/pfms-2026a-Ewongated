#include "sonar.h"
#include <cmath>

/**
 * @brief Constructs a Sonar for the given platform and sets sensor geometry.
 *
 * The angular resolution is set equal to the full FOV because the sonar is a
 * single-beam cone sensor — there is only one ray spanning the entire cone.
 */
Sonar::Sonar(pfms::PlatformType type)
    : Ranger(type)
{
    fov_               = FOV_DEG;
    angularResolution_ = FOV_DEG; // Single-beam cone: one ray spans the full FOV
    maxRange_          = MAX_RANGE_M;
    minRange_          = MIN_RANGE_M;
    sensingMethod_     = pfms::RangerType::CONE;
}

std::vector<double> Sonar::getData()
{
    pfms::sensor_msgs::Sonar sonar;
    pfmsConnectorPtr_->read(sonar);

    // Read the current platform odometry to compute the sensor's world-frame pose.
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);

    pfms::nav_msgs::Odometry pose = odo;

    if (type_ == pfms::PlatformType::ACKERMAN) {
        // Offset the sensor forward of the rear axle along the platform heading.
        pose.position.x += ACKERMAN_OFFSET * std::cos(odo.yaw);
        pose.position.y += ACKERMAN_OFFSET * std::sin(odo.yaw);
    }
    // The sonar is always HEIGHT_OFFSET above the laser on both platforms.
    pose.position.z += HEIGHT_OFFSET;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        sensorPose_ = pose;
    }

    if (sonar.range < minRange_ || sonar.range > maxRange_) {
        return {}; // Reading is outside valid range — return empty vector
    }
    return { sonar.range };
}
