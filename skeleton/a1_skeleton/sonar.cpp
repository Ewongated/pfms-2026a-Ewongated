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
    sensorVerticalOffset_ = 0.2;  // sonar is 0.2m above laser

    // Initialise pose to zero so getSensorPose() is safe before getData()
    sensorPose_ = pfms::nav_msgs::Odometry();
    // Create persistent connector so subscriptions are ready when getData() called
    connector_ = std::make_shared<PfmsConnector>(platformType_);
}

// getData() reads one sonar measurement from the simulator.
// Returns a vector of size 1 containing the range value.
// Updates sensorPose_ so getSensorPose() reflects the current reading.
std::vector<double> Sonar::getData()
{
    pfms::nav_msgs::Odometry platformOdo;
    connector_->read(platformOdo);  // flush stale

    pfms::sensor_msgs::Sonar sonarData;
    connector_->read(sonarData);    // fresh sensor read

    connector_->read(platformOdo);  // fresh odometry
    computeSensorPose(platformOdo);

    std::vector<double> result;
    result.push_back(static_cast<double>(sonarData.range));
    return result;
}