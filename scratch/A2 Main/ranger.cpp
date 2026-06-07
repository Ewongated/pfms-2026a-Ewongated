#include "ranger.h"

/**
 * @brief Constructs a Ranger and opens a PfmsConnector to the given platform.
 * Geometry fields are zero-initialised; derived constructors set them.
 */
Ranger::Ranger(pfms::PlatformType type)
    : type_(type)
    , fov_(0.0)
    , angularResolution_(0.0)
    , maxRange_(0.0)
    , minRange_(0.0)
    , angleMin_(0.0)
    , sensingMethod_(pfms::RangerType::POINT)
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(type);
    sensorPose_       = pfms::nav_msgs::Odometry{};
}

double Ranger::getAngularResolution()
{
    return angularResolution_;
}

pfms::nav_msgs::Odometry Ranger::getSensorPose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return sensorPose_;
}

double Ranger::getFieldOfView()
{
    return fov_;
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

double Ranger::getAngleMin()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return angleMin_;
}