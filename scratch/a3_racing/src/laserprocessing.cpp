#include "laserprocessing.h"

#include <algorithm>
#include <cmath>
#include <numeric>

// ?? Constructor / scan management ????????????????????????????????????????????

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

// ?? Public analysis functions ?????????????????????????????????????????????????

bool LaserProcessing::obstacleInFront() const
{
    // ?? Algorithm overview ????????????????????????????????????????????????????
    // The track has walls on both sides at ~8m total width (~4m each side).
    // Walls appear as long continuous segments spanning many rays.
    // A real obstacle (box, person, stopped vehicle) appears as a SHORT cluster
    // of close readings in the CENTRE of the scan, clearly separated from the
    // wall segments.
    //
    // Strategy:
    //  1. Estimate the expected wall range by taking the median of readings in
    //     the outer thirds of the scan (left and right side sectors).
    //  2. In the forward centre sector, look for a cluster of readings that are
    //     significantly closer than the estimated wall range.
    //  3. A "cluster" requires MIN_OBSTACLE_RAYS consecutive close readings --
    //     this rejects single noisy returns from the wall edge at corners.

    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    lock.unlock();

    if (scan.ranges.empty()) return false;

    const int    nRays = static_cast<int>(scan.ranges.size());
    const double aMin  = scan.angle_min;
    const double aInc  = scan.angle_increment;

    // ?? Step 1: Estimate wall range from side sectors ?????????????????????????
    // Use rays exclusively in the far side sectors (beyond ?75 deg from forward).
    // At ?75 deg the laser is pointing nearly sideways at the wall -- these readings
    // represent the true wall proximity and are unaffected by forward obstacles.
    constexpr double SIDE_SECTOR_DEG = 75.0;
    const double sideSectorRad = SIDE_SECTOR_DEG * M_PI / 180.0;

    std::vector<float> sideRanges;
    sideRanges.reserve(64);

    for (int i = 0; i < nRays; ++i) {
        const double angle = aMin + i * aInc;
        if (std::abs(angle) < sideSectorRad) continue; // skip forward sector
        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
        sideRanges.push_back(r);
    }

    // Need enough side readings to estimate wall range reliably.
    if (sideRanges.size() < 6) {
        // Not enough data -- fail safe (assume no obstacle).
        return false;
    }

    std::sort(sideRanges.begin(), sideRanges.end());
    const float wallRange = sideRanges[sideRanges.size() / 2]; // median

    // Obstacle threshold: anything closer than (wallRange * fraction) in the
    // forward sector that the wall cannot explain.
    // Apply a minimum absolute floor so the threshold never drops dangerously
    // low even when walls are very close (e.g. tight corners).
    const float obstacleThreshold = std::max(
        wallRange * static_cast<float>(OBSTACLE_WALL_FRACTION),
        static_cast<float>(OBSTACLE_MIN_RANGE_M));

    // ?? Step 2: Scan forward centre sector for close cluster ?????????????????
    const double halfConeRad = OBSTACLE_HALF_ANGLE_DEG * M_PI / 180.0;
    unsigned int consecutiveClose = 0;

    for (int i = 0; i < nRays; ++i) {
        const double angle = aMin + i * aInc;
        if (std::abs(angle) > halfConeRad) {
            consecutiveClose = 0; // reset counter outside forward cone
            continue;
        }

        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
            consecutiveClose = 0;
            continue;
        }

        if (r < obstacleThreshold) {
            ++consecutiveClose;
            // Require a cluster of consecutive close rays to confirm obstacle.
            if (consecutiveClose >= MIN_OBSTACLE_RAYS) {
                return true;
            }
        } else {
            consecutiveClose = 0;
        }
    }

    return false;
}

bool LaserProcessing::goalInCorridor(const geometry_msgs::msg::Point& goal) const
{
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    const nav_msgs::msg::Odometry     odom = odom_;
    const bool haveOdom                    = hasOdom_;
    lock.unlock();

    if (!haveOdom || scan.ranges.empty()) return false;

    // Compute laser-frame position of the goal.
    // Laser world pose: offset forward from odom by LASER_OFFSET_M.
    const double yaw       = yawFromOdom(odom);
    const double laserX    = odom.pose.pose.position.x + LASER_OFFSET_M * std::cos(yaw);
    const double laserY    = odom.pose.pose.position.y + LASER_OFFSET_M * std::sin(yaw);

    // Rotate goal into laser frame.
    const double dx        = goal.x - laserX;
    const double dy        = goal.y - laserY;
    const double localX    =  dx * std::cos(-yaw) - dy * std::sin(-yaw);
    const double localY    =  dx * std::sin(-yaw) + dy * std::cos(-yaw);

    // Goal must be ahead of the laser (positive local X).
    if (localX <= 0.0) return false;

    // Collect left-wall (positive local Y) and right-wall (negative local Y)
    // range readings in the forward half of the scan.
    const int    nRays         = static_cast<int>(scan.ranges.size());
    double       leftSumY      = 0.0;
    double       rightSumY     = 0.0;
    unsigned int leftCount     = 0;
    unsigned int rightCount    = 0;

    for (int i = 0; i < nRays; ++i) {
        const double angle = scan.angle_min + i * scan.angle_increment;
        const float  r     = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;

        const double px = r * std::cos(angle); // laser frame X
        const double py = r * std::sin(angle); // laser frame Y

        if (px <= 0.0) continue; // only consider forward readings

        if (py > 0.0) { leftSumY  += py; ++leftCount;  }
        else          { rightSumY += py; ++rightCount; }
    }

    if (leftCount  < MIN_WALL_READINGS_PER_SIDE ||
        rightCount < MIN_WALL_READINGS_PER_SIDE) return false;

    const double leftEdge  = leftSumY  / leftCount;
    const double rightEdge = rightSumY / rightCount;

    // Goal's lateral position in laser frame must fall between the two walls.
    const double minY = rightEdge - CORRIDOR_TOLERANCE_M;
    const double maxY = leftEdge  + CORRIDOR_TOLERANCE_M;

    return (localY >= minY && localY <= maxY);
}

std::optional<geometry_msgs::msg::Point> LaserProcessing::trackCentreAhead() const
{
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    const nav_msgs::msg::Odometry     odom = odom_;
    const bool haveOdom                    = hasOdom_;
    lock.unlock();

    if (!haveOdom || scan.ranges.empty()) return std::nullopt;

    const int nRays = static_cast<int>(scan.ranges.size());

    std::vector<double> leftY, rightY;
    leftY.reserve(32);
    rightY.reserve(32);

    for (int i = 0; i < nRays; ++i) {
        const double angle = scan.angle_min + i * scan.angle_increment;
        const float  r     = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;

        // Only use readings in the forward half of the scan.
        if (std::cos(angle) <= 0.0) continue;

        const double py = r * std::sin(angle);
        if (py > 0.0) leftY.push_back(py);
        else          rightY.push_back(py);
    }

    if (leftY.size()  < MIN_WALL_READINGS_PER_SIDE ||
        rightY.size() < MIN_WALL_READINGS_PER_SIDE) return std::nullopt;

    // Use median to resist outliers.
    auto median = [](std::vector<double>& v) -> double {
        std::sort(v.begin(), v.end());
        const std::size_t n = v.size();
        return (n % 2 == 0) ? (v[n/2 - 1] + v[n/2]) / 2.0 : v[n/2];
    };

    const double leftMedY  = median(leftY);
    const double rightMedY = median(rightY);
    const double centreY   = (leftMedY + rightMedY) / 2.0; // laser frame

    // Project the centre point 5 m ahead in the laser frame, then convert
    // to world coordinates.
    geometry_msgs::msg::Point laserPt;
    laserPt.x = 5.0;  // look-ahead distance [m]
    laserPt.y = centreY;
    laserPt.z = 0.0;

    return laserToWorld(laserPt);
}

unsigned int LaserProcessing::countSegments() const
{
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    lock.unlock();

    unsigned int segmentCount = 0;
    bool         inSegment    = false;
    geometry_msgs::msg::Point prevPt;
    constexpr double JUMP_THRESHOLD = 1.0; // [m]

    for (unsigned int i = 0; i < scan.ranges.size(); ++i) {
        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
            inSegment = false;
            continue;
        }

        const geometry_msgs::msg::Point curPt = polarToCart(i);

        if (!inSegment) {
            ++segmentCount;
            inSegment = true;
        } else {
            const double dx = curPt.x - prevPt.x;
            const double dy = curPt.y - prevPt.y;
            if (std::sqrt(dx*dx + dy*dy) > JUMP_THRESHOLD) {
                ++segmentCount;
            }
        }
        prevPt = curPt;
    }
    return segmentCount;
}

// ?? Private helpers ??????????????????????????????????????????????????????????

geometry_msgs::msg::Point LaserProcessing::polarToCart(unsigned int index) const
{
    const double angle = laserScan_.angle_min + laserScan_.angle_increment * index;
    const float  r     = laserScan_.ranges[index];
    geometry_msgs::msg::Point pt;
    pt.x = static_cast<double>(r) * std::cos(angle);
    pt.y = static_cast<double>(r) * std::sin(angle);
    pt.z = 0.0;
    return pt;
}

geometry_msgs::msg::Point LaserProcessing::laserToWorld(
    const geometry_msgs::msg::Point& laserPt) const
{
    const double yaw    = yawFromOdom(odom_);
    const double laserX = odom_.pose.pose.position.x + LASER_OFFSET_M * std::cos(yaw);
    const double laserY = odom_.pose.pose.position.y + LASER_OFFSET_M * std::sin(yaw);

    geometry_msgs::msg::Point worldPt;
    worldPt.x = laserX + laserPt.x * std::cos(yaw) - laserPt.y * std::sin(yaw);
    worldPt.y = laserY + laserPt.x * std::sin(yaw) + laserPt.y * std::cos(yaw);
    worldPt.z = 0.0;
    return worldPt;
}

double LaserProcessing::yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    // Extract yaw from quaternion using the standard formula.
    const auto& q  = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}