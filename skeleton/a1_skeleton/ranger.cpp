#include "ranger.h"

// These methods are identical for all sensors - they simply return the
// protected member variables set by each derived class constructor.
// Only getData() varies per sensor and stays in sonar.cpp / laser.cpp.

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
// Does NOT trigger a new read - pose is updated inside getData() so that
// all getters work correctly regardless of call order.
pfms::nav_msgs::Odometry Ranger::getSensorPose()
{
    return sensorPose_;
}