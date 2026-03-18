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

    // Initialise pose to zero so getSensorPose() is safe before getData()
    sensorPose_ = pfms::nav_msgs::Odometry();
}

// getData() reads a full laser scan from the simulator.
// Returns a vector of 640 range values (doubles).
// Updates sensorPose_ so getSensorPose() reflects the current reading.
std::vector<double> Laser::getData()
{
    PfmsConnector connector(platformType_);

    pfms::nav_msgs::Odometry platformOdo;
    connector.read(platformOdo);
    computeSensorPose(platformOdo);  // updates sensorPose_

    pfms::sensor_msgs::LaserScan scan;
    connector.read(scan);

    std::vector<double> result;
    result.reserve(scan.ranges.size());
    for (float r : scan.ranges) {
        result.push_back(static_cast<double>(r));
    }
    return result;
}