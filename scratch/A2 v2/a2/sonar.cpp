#include "sonar.h"
#include "pfmsconnector.h"

/**
 * @brief Constructs a Sonar sensor connected to the given platform.
 *
 * Sets sensor constants and the static sensor-to-vehicle transform.
 * The sonar is 0.2m above the laser (sensorVerticalOffset_ = 0.2).
 *
 * @param type Platform the sensor is mounted on (ACKERMAN or SKIDSTEER)
 */
Sonar::Sonar(pfms::PlatformType type)
{
    platformType_      = type;
    angularResolution_ = 0.0;    // CONE sensors have no angular resolution
    fieldOfView_       = 0.052;  // [deg] — narrow forward-facing cone
    maxRange_          = 20.0;   // [m]
    minRange_          = 0.02;   // [m]
    sensingMethod_     = pfms::RangerType::CONE;

    // Static sensor offset (vehicle reference frame)
    // ACKERMAN: 3.725m forward of rear axle; SKIDSTEER: at centre
    if (type == pfms::PlatformType::ACKERMAN) {
        sensorForwardOffset_ = 3.725;
    } else {
        sensorForwardOffset_ = 0.0;
    }
    sensorLateralOffset_  = 0.0;
    sensorVerticalOffset_ = 0.2; // Sonar is 0.2m above the laser

    sensorPose_ = pfms::nav_msgs::Odometry();
    connector_  = std::make_shared<PfmsConnector>(platformType_);
}

/**
 * @brief Reads one sonar measurement from the simulator.
 *
 * Reads a fresh odometry, fresh sonar data, then reads odometry again
 * to ensure the pose matches the sensor reading. Updates sensorPose_.
 *
 * @return Vector of size 1: the measured range [m]
 */
std::vector<double> Sonar::getData()
{
    pfms::nav_msgs::Odometry platformOdo;
    connector_->read(platformOdo); // flush any stale odometry

    pfms::sensor_msgs::Sonar sonarData;
    connector_->read(sonarData);   // fresh sensor reading

    connector_->read(platformOdo); // fresh odometry matching this reading
    computeSensorPose(platformOdo);

    return { static_cast<double>(sonarData.range) };
}
