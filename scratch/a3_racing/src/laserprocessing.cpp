#include "laserprocessing.h"

#include <algorithm>
#include <cmath>
#include <numeric>

// ── Constructor / scan management ────────────────────────────────────────────

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

// ── Public analysis functions ─────────────────────────────────────────────────

bool LaserProcessing::obstacleInFront() const
{
    // ── Algorithm overview ────────────────────────────────────────────────────
    // The track has walls on either side at ~8m total width (~4m each side),
    // with a tolerance applied per the project spec. Any forward return
    // significantly closer than (HALF_TRACK_WIDTH_M - TRACK_TOLERANCE_M)
    // cannot be a wall and is therefore a non-wall obstacle.
    //
    // A cluster of MIN_OBSTACLE_RAYS consecutive close readings is required
    // to confirm an obstacle, rejecting single noisy returns.

    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    lock.unlock();

    if (scan.ranges.empty()) return false;

    const int    nRays       = static_cast<int>(scan.ranges.size());
    const double aMin        = scan.angle_min;
    const double aInc        = scan.angle_increment;
    const double halfConeRad = OBSTACLE_HALF_ANGLE_DEG * M_PI / 180.0;

    unsigned int consecutiveClose = 0;

    for (int i = 0; i < nRays; ++i) {
        const double angle = aMin + i * aInc;

        if (std::abs(angle) > halfConeRad) {
            consecutiveClose = 0;
            continue;
        }

        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
            consecutiveClose = 0;
            continue;
        }

        // Wall returns are at approximately HALF_TRACK_WIDTH_M or beyond.
        // Anything closer than the tolerance-adjusted threshold is a non-wall
        // obstacle -- walls cannot physically be this close in the forward cone.
        if (r < OBSTACLE_MAX_RANGE_M) {
            ++consecutiveClose;
            if (consecutiveClose >= MIN_OBSTACLE_RAYS)
                return true;
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
    const double yaw    = yawFromOdom(odom);
    const double laserX = odom.pose.pose.position.x + LASER_OFFSET_M * std::cos(yaw);
    const double laserY = odom.pose.pose.position.y + LASER_OFFSET_M * std::sin(yaw);

    // Rotate goal into laser frame.
    const double dx     = goal.x - laserX;
    const double dy     = goal.y - laserY;
    const double localX =  dx * std::cos(-yaw) - dy * std::sin(-yaw);
    const double localY =  dx * std::sin(-yaw) + dy * std::cos(-yaw);

    // Goal must be ahead of the laser (positive local X).
    if (localX <= 0.0) return false;

    // Collect left-wall (positive local Y) and right-wall (negative local Y)
    // lateral positions for all forward-facing valid readings.
    //
    // Fix: use median (not mean) for edge estimation to match trackCentreAhead()
    // and to be robust against noisy or out-of-range wall returns.
    const int nRays = static_cast<int>(scan.ranges.size());
    std::vector<double> leftY, rightY;
    leftY.reserve(32);
    rightY.reserve(32);

    for (int i = 0; i < nRays; ++i) {
        const double angle = scan.angle_min + i * scan.angle_increment;
        const float  r     = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;

        const double px = r * std::cos(angle);
        const double py = r * std::sin(angle);

        if (px <= 0.0) continue;

        if (py > 0.0) leftY.push_back(py);
        else          rightY.push_back(py);
    }

    if (leftY.size()  < MIN_WALL_READINGS_PER_SIDE ||
        rightY.size() < MIN_WALL_READINGS_PER_SIDE) return false;

    const double leftEdge  = median(leftY);
    const double rightEdge = median(rightY);

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

        if (std::cos(angle) <= 0.0) continue;

        const double py = r * std::sin(angle);
        if (py > 0.0) leftY.push_back(py);
        else          rightY.push_back(py);
    }

    if (leftY.size()  < MIN_WALL_READINGS_PER_SIDE ||
        rightY.size() < MIN_WALL_READINGS_PER_SIDE) return std::nullopt;

    const double leftMedY  = median(leftY);
    const double rightMedY = median(rightY);
    const double centreY   = (leftMedY + rightMedY) / 2.0;

    geometry_msgs::msg::Point laserPt;
    laserPt.x = 5.0;
    laserPt.y = centreY;
    laserPt.z = 0.0;

    // Pass the odom snapshot explicitly -- laserToWorld() no longer touches odom_.
    return laserToWorld(laserPt, odom);
}

unsigned int LaserProcessing::countSegments() const
{
    // Take a snapshot under the lock, then operate entirely on the local copy.
    // This prevents a data race: the original code called polarToCart() after
    // releasing the lock, which read laserScan_ without protection.
    std::unique_lock<std::mutex> lock(mtx_);
    const sensor_msgs::msg::LaserScan scan = laserScan_;
    lock.unlock();

    unsigned int segmentCount = 0;
    bool         inSegment    = false;
    geometry_msgs::msg::Point prevPt;
    constexpr double JUMP_THRESHOLD = 1.0;

    for (unsigned int i = 0; i < scan.ranges.size(); ++i) {
        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
            inSegment = false;
            continue;
        }

        // Pass the local scan snapshot; no member access after lock release.
        const geometry_msgs::msg::Point curPt = polarToCart(scan, i);

        if (!inSegment) {
            ++segmentCount;
            inSegment = true;
        } else {
            const double dx = curPt.x - prevPt.x;
            const double dy = curPt.y - prevPt.y;
            if (std::sqrt(dx*dx + dy*dy) > JUMP_THRESHOLD)
                ++segmentCount;
        }
        prevPt = curPt;
    }
    return segmentCount;
}

// ── Private helpers ───────────────────────────────────────────────────────────

// Fix: accepts a scan snapshot so the caller can pass a locally-held copy
// rather than relying on the member laserScan_, which may be updated by
// newScan() on another thread after the lock is released.
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

// Fix: accepts an odom snapshot so the caller can pass a locally-held copy
// rather than relying on the member odom_, which may be updated by
// newOdom() on another thread after the lock is released.
geometry_msgs::msg::Point LaserProcessing::laserToWorld(
    const geometry_msgs::msg::Point& laserPt,
    const nav_msgs::msg::Odometry&   odom)
{
    const double yaw    = yawFromOdom(odom);
    const double laserX = odom.pose.pose.position.x + LASER_OFFSET_M * std::cos(yaw);
    const double laserY = odom.pose.pose.position.y + LASER_OFFSET_M * std::sin(yaw);

    geometry_msgs::msg::Point worldPt;
    worldPt.x = laserX + laserPt.x * std::cos(yaw) - laserPt.y * std::sin(yaw);
    worldPt.y = laserY + laserPt.x * std::sin(yaw) + laserPt.y * std::cos(yaw);
    worldPt.z = 0.0;
    return worldPt;
}

double LaserProcessing::yawFromOdom(const nav_msgs::msg::Odometry& odom)
{
    const auto& q          = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Fix: extracted from trackCentreAhead() into a shared static helper so both
// trackCentreAhead() and goalInCorridor() use identical median logic.
double LaserProcessing::median(std::vector<double>& v)
{
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return (n % 2 == 0) ? (v[n/2 - 1] + v[n/2]) / 2.0 : v[n/2];
}