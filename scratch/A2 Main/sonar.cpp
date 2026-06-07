#include "sonar.h"
#include <cmath>

/**
 * @brief Constructs a Sonar for the given platform and sets sensor geometry.
 * Angular resolution is set to the full FOV (single-beam cone sensor).
 */
Sonar::Sonar(pfms::PlatformType type)
    : Ranger(type)
{
    fov_               = FOV_DEG;
    angularResolution_ = FOV_DEG; // single cone beam — resolution equals FOV
    maxRange_          = MAX_RANGE_M;
    minRange_          = MIN_RANGE_M;
    sensingMethod_     = pfms::RangerType::CONE;
}

/**
 * @brief Reads one sonar measurement from the simulator.
 *
 * Computes sensor pose in world frame (same offset rules as Laser but
 * 0.2 m higher — the z offset is stored in sensorPose_.position.z).
 *
 * @return Single-element vector with range [m], or empty if invalid
 */
std::vector<double> Sonar::getData()
{
    pfms::sensor_msgs::Sonar sonar;
    pfmsConnectorPtr_->read(sonar);

    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);

    pfms::nav_msgs::Odometry pose = odo;

    if (type_ == pfms::PlatformType::ACKERMAN) {
        pose.position.x += ACKERMAN_OFFSET * std::cos(odo.yaw);
        pose.position.y += ACKERMAN_OFFSET * std::sin(odo.yaw);
    }
    pose.position.z += HEIGHT_OFFSET;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        sensorPose_ = pose;
    }

    if (sonar.range < minRange_ || sonar.range > maxRange_) {
        return {};
    }
    return { sonar.range };
}
