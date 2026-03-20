#include "laser.h"
#include "pfmsconnector.h"

Laser::Laser(pfms::PlatformType type)
{
    platformType_      = type;
    fieldOfView_       = 180.0;                // [deg]
    angularResolution_ = fieldOfView_ / 640.0; // [deg] ~0.28125
    maxRange_          = 30.0;                 // [m]
    minRange_          = 0.1;                  // [m]
    sensingMethod_     = pfms::RangerType::POINT;

    // Sensor offset depends on platform:
    // ACKERMAN (Audi R8): sensor is 3.725m ahead of the rear-axle centre
    // SKIDSTEER (Husky):  sensor is at the platform centre
    if (type == pfms::PlatformType::ACKERMAN) {
        sensorForwardOffset_ = 3.725;
    } else {
        sensorForwardOffset_ = 0.0;
    }
    sensorLateralOffset_ = 0.0;
    sensorVerticalOffset_ = 0.0;  // laser is at platform height

    // Initialise pose to zero so getSensorPose() is safe before getData()
    sensorPose_ = pfms::nav_msgs::Odometry();
    // Create persistent connector so subscriptions are ready when getData() called
    connector_ = std::make_shared<PfmsConnector>(platformType_);
}

// getData() reads a full laser scan from the simulator.
// Returns a vector of 640 range values (doubles).
// Updates sensorPose_ so getSensorPose() reflects the current reading.
std::vector<double> Laser::getData()
{
    // First read flushes stale odometry from before teleport
    pfms::nav_msgs::Odometry platformOdo;
    connector_->read(platformOdo);

    // Now read the scan — blocks until a fresh scan arrives
    pfms::sensor_msgs::LaserScan scan;
    connector_->read(scan);

    // Read odometry again — now guaranteed to be current
    connector_->read(platformOdo);
    computeSensorPose(platformOdo);

    std::vector<double> result;
    result.reserve(scan.ranges.size());
    for (float r : scan.ranges) {
        result.push_back(static_cast<double>(r));
    }
    return result;
}

// Setter for angular resolution - Laser supports this setting.
// Returns false if the requested resolution is not achievable (<=0).
bool Laser::setAngularResolution(double angularResolution)
{
    if (angularResolution <= 0.0) {
        return false;
    }
    angularResolution_ = angularResolution;
    return true;
}