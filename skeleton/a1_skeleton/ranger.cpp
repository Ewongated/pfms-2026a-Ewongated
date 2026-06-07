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

pfms::nav_msgs::Odometry Ranger::getSensorPose()
{
    return sensorPose_;
}