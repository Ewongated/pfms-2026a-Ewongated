#include "sonar.h"
#include "pfmsconnector.h"

Sonar::Sonar(pfms::PlatformType type)
{
    platformType_      = type;
    angularResolution_ = 0.0;
    fieldOfView_       = 0.052;  // [rad] ~3 degrees
    maxRange_          = 20.0;   // [m]
    minRange_          = 0.02;   // [m]
    sensingMethod_     = pfms::RangerType::CONE;

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

// getData() reads one sonar measurement from the simulator.
// Returns a vector of size 1 containing the range value.
// Updates sensorPose_ so getSensorPose() reflects the current reading.
std::vector<double> Sonar::getData()
{
    PfmsConnector connector(platformType_);

    pfms::nav_msgs::Odometry platformOdo;
    connector.read(platformOdo);
    computeSensorPose(platformOdo);  // updates sensorPose_

    pfms::sensor_msgs::Sonar sonarData;
    connector.read(sonarData);

    std::vector<double> result;
    result.push_back(static_cast<double>(sonarData.range));
    return result;
}