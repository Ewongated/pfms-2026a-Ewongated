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
    sensorLateralOffset_  = 0.0;
    sensorVerticalOffset_ = 0.0;

    // Initialise scanAngleMin_ to default
    scanAngleMin_ = -fieldOfView_ / 2.0 * M_PI / 180.0;

    // Initialise pose to default
    sensorPose_ = pfms::nav_msgs::Odometry();
    
    // Create persistent connector so subscriptions are ready when getData() called
    connector_ = std::make_shared<PfmsConnector>(platformType_);
}

// getData() reads a full laser scan from the simulator.
// Returns a vector of range values (doubles).
// Updates sensorPose_ so getSensorPose() reflects the current reading.
// Also updates fieldOfView_, angularResolution_, and scanAngleMin_ from
// the actual scan data so ray angles are correct for any platform.
std::vector<double> Laser::getData()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    pfms::sensor_msgs::LaserScan scan;
    connector_->read(scan);

    pfms::nav_msgs::Odometry platformOdo;
    connector_->read(platformOdo);
    computeSensorPose(platformOdo);

    // Update angular data from actual scan
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