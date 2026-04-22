#include "laser.h"
#include "pfmsconnector.h"
#include <thread>
#include <chrono>

/**
 * @brief Constructs a Laser sensor connected to the given platform.
 *
 * Sets all sensor constants and computes the static sensor-to-vehicle
 * transform. No odometry is read here — the pose is computed lazily
 * inside getData() using the platform odometry at the time of the scan.
 *
 * @param type Platform the sensor is mounted on (ACKERMAN or SKIDSTEER)
 */
Laser::Laser(pfms::PlatformType type)
{
    platformType_      = type;
    fieldOfView_       = 180.0;                // [deg]
    angularResolution_ = fieldOfView_ / 640.0; // [deg/ray] ~0.28125
    maxRange_          = 30.0;                 // [m]
    minRange_          = 0.1;                  // [m]
    sensingMethod_     = pfms::RangerType::POINT;

    // Static sensor offset relative to platform origin (vehicle reference frame)
    // ACKERMAN (Audi R8): laser is 3.725m ahead of the rear-axle centre
    // SKIDSTEER (Husky): laser is co-located at platform centre
    if (type == pfms::PlatformType::ACKERMAN) {
        sensorForwardOffset_ = 3.725;
    } else {
        sensorForwardOffset_ = 0.0;
    }
    sensorLateralOffset_  = 0.0;
    sensorVerticalOffset_ = 0.0;

    scanAngleMin_ = -fieldOfView_ / 2.0 * M_PI / 180.0;
    sensorPose_   = pfms::nav_msgs::Odometry();

    // Create the connector once — subscriptions are ready for getData()
    connector_ = std::make_shared<PfmsConnector>(platformType_);
}

/**
 * @brief Reads a complete laser scan from the simulator.
 *
 * Waits 200ms for a fresh scan, then reads both the laser data and the
 * current platform odometry. Updates sensorPose_ and angular parameters
 * (fieldOfView_, angularResolution_, scanAngleMin_) from actual scan data.
 *
 * @return Vector of range readings [m], one per ray (typically 640)
 */
std::vector<double> Laser::getData()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    pfms::sensor_msgs::LaserScan scan;
    connector_->read(scan);

    pfms::nav_msgs::Odometry platformOdo;
    connector_->read(platformOdo);
    computeSensorPose(platformOdo);

    // Sync angular metadata from actual scan (handles any platform variation)
    scanAngleMin_      = scan.angle_min;
    angularResolution_ = scan.angle_increment * 180.0 / M_PI;
    fieldOfView_       = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;

    std::vector<double> result;
    result.reserve(scan.ranges.size());
    for (float r : scan.ranges) {
        result.push_back(static_cast<double>(r));
    }
    return result;
}

/**
 * @brief Sets the angular resolution.
 * @param angularResolution Desired resolution [deg]
 * @return true on success, false if value <= 0
 */
bool Laser::setAngularResolution(double angularResolution)
{
    if (angularResolution <= 0.0) {
        return false;
    }
    angularResolution_ = angularResolution;
    return true;
}
