/**
 * @file laserprocessing.cpp
 * @brief Implementation of LaserProcessing.
 */

#include "laserprocessing.h"
#include <algorithm>
#include <cmath>

// ── Construction / data updates ───────────────────────────────────────────────

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan)
    : laserScan_(laserScan), hasOdom_(false)
{
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan)
{
    std::lock_guard<std::mutex> lock(mtx_);
    laserScan_ = laserScan;
}

void LaserProcessing::newOdom(nav_msgs::msg::Odometry odom)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_    = odom;
    hasOdom_ = true;
}

// ── Public analysis ───────────────────────────────────────────────────────────

bool LaserProcessing::obstacleInFront() const
{
    // Snapshot under lock, then work on the local copy.
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    lock.unlock();

    if (scan.ranges.empty()) return false;

    const double halfConeRad  = OBSTACLE_HALF_ANGLE_DEG * M_PI / 180.0;
    unsigned int consecutive  = 0;

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
        const double angle = scan.angle_min + i * scan.angle_increment;

        // Only examine the narrow forward cone.
        if (std::abs(angle) > halfConeRad) { consecutive = 0; continue; }

        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
            consecutive = 0; continue;
        }

        // Walls sit at ~HALF_TRACK_WIDTH_M; anything closer is a non-wall obstacle.
        if (r < OBSTACLE_MAX_RANGE_M) {
            if (++consecutive >= MIN_OBSTACLE_RAYS) return true;
        } else {
            consecutive = 0;
        }
    }
    return false;
}

bool LaserProcessing::goalInCorridor(const geometry_msgs::msg::Point& goal) const
{
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan    = laserScan_;
    const nav_msgs::msg::Odometry     odom    = odom_;
    const bool                        haveOdom = hasOdom_;
    lock.unlock();

    if (!haveOdom || scan.ranges.empty()) return false;

    // Transform goal into the car body frame.
    const double yaw    = geom::yawFromOdom(odom);
    const double dx     = goal.x - odom.pose.pose.position.x;
    const double dy     = goal.y - odom.pose.pose.position.y;
    const double localX =  dx * std::cos(-yaw) - dy * std::sin(-yaw);
    const double localY =  dx * std::sin(-yaw) + dy * std::cos(-yaw);

    if (localX <= -2.0) return false; // Goal is well behind the car.

    std::vector<double> leftY, rightY;
    collectWallReadings(scan, leftY, rightY);

    // Fall back to nominal track width when walls are not clearly visible.
    if (leftY.size() < MIN_WALL_READINGS_PER_SIDE ||
        rightY.size() < MIN_WALL_READINGS_PER_SIDE)
    {
        return std::abs(localY) < HALF_TRACK_WIDTH_M;
    }

    const double minY = median(rightY) - CORRIDOR_TOLERANCE_M;
    const double maxY = median(leftY)  + CORRIDOR_TOLERANCE_M;
    return (localY >= minY && localY <= maxY);
}

std::optional<geometry_msgs::msg::Point> LaserProcessing::trackCentreAhead() const
{
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan    = laserScan_;
    const nav_msgs::msg::Odometry     odom    = odom_;
    const bool                        haveOdom = hasOdom_;
    lock.unlock();

    if (!haveOdom || scan.ranges.empty()) return std::nullopt;

    std::vector<double> leftY, rightY;
    collectWallReadings(scan, leftY, rightY);

    if (leftY.size() < MIN_WALL_READINGS_PER_SIDE ||
        rightY.size() < MIN_WALL_READINGS_PER_SIDE) return std::nullopt;

    const double centreY = (median(leftY) + median(rightY)) / 2.0;

    // Place the centre point 5 m ahead in the laser frame, then transform.
    geometry_msgs::msg::Point laserPt;
    laserPt.x = 5.0;
    laserPt.y = centreY;
    laserPt.z = 0.0;
    return laserToWorld(laserPt, odom);
}

unsigned int LaserProcessing::countSegments() const
{
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    lock.unlock();

    unsigned int segments = 0;
    bool inSegment = false;
    geometry_msgs::msg::Point prev;
    constexpr double JUMP = 1.0; // m — gap larger than this starts a new segment

    for (unsigned int i = 0; i < scan.ranges.size(); ++i) {
        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
            inSegment = false; continue;
        }

        const geometry_msgs::msg::Point cur = polarToCart(scan, i);
        if (!inSegment) {
            ++segments;
            inSegment = true;
        } else {
            if (std::hypot(cur.x - prev.x, cur.y - prev.y) > JUMP)
                ++segments;
        }
        prev = cur;
    }
    return segments;
}

// ── Private helpers ───────────────────────────────────────────────────────────

void LaserProcessing::collectWallReadings(const sensor_msgs::msg::LaserScan& scan,
                                          std::vector<double>& leftY,
                                          std::vector<double>& rightY)
{
    leftY.reserve(32);
    rightY.reserve(32);

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
        const double angle = scan.angle_min + i * scan.angle_increment;
        const float  r     = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
        if (r * std::cos(angle) <= 0.0) continue; // Ignore backward-facing rays.

        const double py = r * std::sin(angle);
        if (py > 0.0) leftY.push_back(py);
        else          rightY.push_back(py);
    }
}

geometry_msgs::msg::Point LaserProcessing::polarToCart(
    const sensor_msgs::msg::LaserScan& scan, unsigned int index)
{
    const double angle = scan.angle_min + scan.angle_increment * index;
    const float  r     = scan.ranges[index];
    geometry_msgs::msg::Point pt;
    pt.x = static_cast<double>(r) * std::cos(angle);
    pt.y = static_cast<double>(r) * std::sin(angle);
    pt.z = 0.0;
    return pt;
}

geometry_msgs::msg::Point LaserProcessing::laserToWorld(
    const geometry_msgs::msg::Point& laserPt,
    const nav_msgs::msg::Odometry&   odom)
{
    const double yaw    = geom::yawFromOdom(odom);
    const double laserX = odom.pose.pose.position.x + LASER_OFFSET_M * std::cos(yaw);
    const double laserY = odom.pose.pose.position.y + LASER_OFFSET_M * std::sin(yaw);

    geometry_msgs::msg::Point world;
    world.x = laserX + laserPt.x * std::cos(yaw) - laserPt.y * std::sin(yaw);
    world.y = laserY + laserPt.x * std::sin(yaw) + laserPt.y * std::cos(yaw);
    world.z = 0.0;
    return world;
}

double LaserProcessing::median(std::vector<double>& v)
{
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return (n % 2 == 0) ? (v[n/2 - 1] + v[n/2]) / 2.0 : v[n/2];
}
