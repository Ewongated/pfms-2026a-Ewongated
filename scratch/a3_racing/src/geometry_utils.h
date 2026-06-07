/**
 * @file geometry_utils.h
 * @brief Shared geometry helpers used by LaserProcessing and RacingNode.
 *
 * Centralising these here removes the duplicate yawFromOdom() and euclidean()
 * implementations that previously existed in both classes.
 */
#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

namespace geom
{

/**
 * @brief Extracts the yaw angle from an odometry quaternion.
 * @param odom Odometry message.
 * @return Yaw in [-pi, pi] radians.
 */
inline double yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    const auto& q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/**
 * @brief Euclidean distance from the odometry position to a point.
 * @param odom Odometry providing the current position.
 * @param pt   Target point in world coordinates.
 * @return Distance in metres.
 */
inline double euclidean(const nav_msgs::msg::Odometry& odom,
                        const geometry_msgs::msg::Point& pt)
{
    const double dx = pt.x - odom.pose.pose.position.x;
    const double dy = pt.y - odom.pose.pose.position.y;
    return std::hypot(dx, dy);
}

} // namespace geom

#endif // GEOMETRY_UTILS_H
