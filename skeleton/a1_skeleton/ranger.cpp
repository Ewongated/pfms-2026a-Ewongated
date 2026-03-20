#include "ranger.h"
#include "pfmsconnector.h"

double Ranger::getAngularResolution()
{
    return angularResolution_;
}

double Ranger::getFieldOfView()
{
    return fieldOfView_;
}

double Ranger::getMaxRange()
{
    return maxRange_;
}

double Ranger::getMinRange()
{
    return minRange_;
}

pfms::RangerType Ranger::getSensingMethod()
{
    return sensingMethod_;
}

// Returns the sensor pose computed during the most recent getData() call.
// getData() updates sensorPose_ via computeSensorPose() so this is always
// current after getData() has been called. Does NOT trigger a new read,
// avoiding an unnecessary blocking call to the connector.
pfms::nav_msgs::Odometry Ranger::getSensorPose()
{
    return sensorPose_;
}