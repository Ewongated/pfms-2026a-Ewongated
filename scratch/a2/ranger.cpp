#include "ranger.h"
#include <cmath>

/**
 * @brief Returns the angular resolution of the sensor.
 * @return Angular resolution [deg]
 */
double Ranger::getAngularResolution()
{
    return angularResolution_;
}

/**
 * @brief Returns the field of view of the sensor.
 * @return Field of view [deg]
 */
double Ranger::getFieldOfView()
{
    return fieldOfView_;
}

/**
 * @brief Returns the maximum sensing range.
 * @return Maximum range [m]
 */
double Ranger::getMaxRange()
{
    return maxRange_;
}

/**
 * @brief Returns the minimum sensing range.
 * @return Minimum range [m]
 */
double Ranger::getMinRange()
{
    return minRange_;
}

/**
 * @brief Returns the sensing method (POINT or CONE).
 * @return RangerType enum value
 */
pfms::RangerType Ranger::getSensingMethod()
{
    return sensingMethod_;
}

/**
 * @brief Returns the most recently computed sensor pose in world frame.
 *
 * This value is updated every time getData() is called on a derived class.
 * The pose is computed from the platform odometry plus the static sensor offset.
 *
 * @return Sensor pose as pfms::nav_msgs::Odometry
 */
pfms::nav_msgs::Odometry Ranger::getSensorPose()
{
    return sensorPose_;
}

/**
 * @brief Computes world-frame sensor pose from platform odometry.
 *
 * Applies the fixed static forward/lateral/vertical offsets (set in the
 * derived class constructor) to transform from the vehicle reference frame
 * to the world frame. Should be called inside getData() after each fresh
 * odometry read — never in the constructor.
 *
 * @param platformOdo Current platform odometry from the simulator
 */
void Ranger::computeSensorPose(const pfms::nav_msgs::Odometry& platformOdo)
{
    double yaw = platformOdo.yaw;

    sensorPose_.position.x = platformOdo.position.x
                            + sensorForwardOffset_ * std::cos(yaw)
                            - sensorLateralOffset_ * std::sin(yaw);
    sensorPose_.position.y = platformOdo.position.y
                            + sensorForwardOffset_ * std::sin(yaw)
                            + sensorLateralOffset_ * std::cos(yaw);
    sensorPose_.position.z = platformOdo.position.z + sensorVerticalOffset_;
    sensorPose_.time       = platformOdo.time;

    while (yaw >  M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    sensorPose_.yaw = yaw;
}