#include "racing_track_pkg/track_analyser.h"

#include <algorithm>
#include <cmath>
#include <numeric>

// ============================================================
// Constructor
// ============================================================

TrackAnalyser::TrackAnalyser(double nominalTrackWidth,
                             double goalTolerance,
                             double obstacleRange,
                             double waypointSpacing)
    : nominalTrackWidth_(nominalTrackWidth)
    , goalTolerance_(goalTolerance)
    , obstacleRange_(obstacleRange)
    , waypointSpacing_(waypointSpacing)
{}

// ── Configuration ─────────────────────────────────────────────────────────────

void TrackAnalyser::setNominalTrackWidth(double w)  { nominalTrackWidth_ = w; }
void TrackAnalyser::setGoalTolerance(double t)       { goalTolerance_     = t; }

// ============================================================
// toWorldPoints
// ============================================================

std::vector<Point2D> TrackAnalyser::toWorldPoints(
    const std::vector<double>& ranges,
    double carX, double carY, double carYaw,
    double angleMin, double angleInc) const
{
    double sx, sy;
    sensorOrigin(carX, carY, carYaw, sx, sy);

    std::vector<Point2D> pts;
    pts.reserve(ranges.size());

    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] <= 0.0) continue; // 0.0 marks an invalid ray

        const double worldAngle = carYaw + angleMin + static_cast<double>(i) * angleInc;
        pts.push_back({sx + ranges[i] * std::cos(worldAngle),
                       sy + ranges[i] * std::sin(worldAngle)});
    }
    return pts;
}

// ============================================================
// analyseCorridor
// ============================================================

CorridorResult TrackAnalyser::analyseCorridor(
    const std::vector<Point2D>& hits,
    double carX, double carY, double carYaw) const
{
    CorridorResult result;

    double sx, sy;
    sensorOrigin(carX, carY, carYaw, sx, sy);

    const double cosY = std::cos(carYaw);
    const double sinY = std::sin(carYaw);

    // Separate hits into left (+ly) and right (-ly) groups based on the car's
    // lateral axis.  Only hits that are far enough laterally to plausibly be
    // wall returns (> WALL_LATERAL_MIN_M) are considered.
    std::vector<double> leftLy, rightLy;

    for (const auto& pt : hits) {
        const double dx =  pt.x - sx;
        const double dy =  pt.y - sy;
        // Rotate world-frame offset into sensor-local frame.
        // lx = forward component, ly = lateral (positive = left)
        const double ly = -dx * sinY + dy * cosY;

        if (ly >  WALL_LATERAL_MIN_M) {
            leftLy.push_back(ly);
        } else if (ly < -WALL_LATERAL_MIN_M) {
            rightLy.push_back(std::abs(ly)); // store as positive distance
        }
    }

    if (static_cast<int>(leftLy.size())  < MIN_WALL_HITS ||
        static_cast<int>(rightLy.size()) < MIN_WALL_HITS) {
        return result; // not enough data — result.valid remains false
    }

    result.dLeft  = median(leftLy);
    result.dRight = median(rightLy);
    result.trackWidth = result.dLeft + result.dRight;

    // Corridor centre offset from the sensor, in the lateral (perpendicular to heading) direction.
    // Positive = left of car.
    const double centreOffset = (result.dLeft - result.dRight) / 2.0;

    // Convert back to world frame: move perpendicularly to the car heading.
    result.centreX = sx - centreOffset * sinY;
    result.centreY = sy + centreOffset * cosY;
    result.valid   = true;
    return result;
}

// ============================================================
// isGoalInCorridor
// ============================================================

bool TrackAnalyser::isGoalInCorridor(
    double goalX, double goalY,
    const CorridorResult& corridor,
    double carX, double carY, double carYaw) const
{
    if (!corridor.valid) return false;

    double sx, sy;
    sensorOrigin(carX, carY, carYaw, sx, sy);

    const double sinY = std::sin(carYaw);
    const double cosY = std::cos(carYaw);

    // Goal lateral offset in the sensor frame.
    const double dx      = goalX - sx;
    const double dy      = goalY - sy;
    const double goalLy  = -dx * sinY + dy * cosY;

    // Corridor centre lateral offset in the sensor frame.
    const double cdx      = corridor.centreX - sx;
    const double cdy      = corridor.centreY - sy;
    const double centreLy = -cdx * sinY + cdy * cosY;

    return std::abs(goalLy - centreLy) < goalTolerance_;
}

// ============================================================
// obstacleAhead
// ============================================================

bool TrackAnalyser::obstacleAhead(
    const std::vector<double>& ranges,
    double angleMin, double angleInc) const
{
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] <= 0.0) continue;

        const double localAngle = angleMin + static_cast<double>(i) * angleInc;

        // Only consider rays within the narrow forward cone.
        if (std::abs(localAngle) > OBSTACLE_HALF_ANGLE_RAD) continue;

        const double lx = ranges[i] * std::cos(localAngle); // forward component
        const double ly = ranges[i] * std::sin(localAngle); // lateral component

        // A wall hit would be at a large lateral offset. Anything near the
        // centreline within obstacleRange_ is treated as an obstacle.
        if (std::abs(ly) < WALL_LATERAL_MIN_M && lx < obstacleRange_) {
            return true;
        }
    }
    return false;
}

// ============================================================
// computeSteering
// ============================================================

double TrackAnalyser::computeSteering(
    double carX, double carY, double carYaw,
    double goalX, double goalY) const
{
    const double bearing      = std::atan2(goalY - carY, goalX - carX);
    const double headingError = normaliseAngle(bearing - carYaw);
    const double steering     = STEERING_GAIN * headingError / M_PI;
    return std::clamp(steering, -STEERING_MAX, STEERING_MAX);
}

// ============================================================
// computeWaypoints  (D/HD)
// ============================================================

std::vector<Point2D> TrackAnalyser::computeWaypoints(
    const std::vector<Point2D>& hits,
    double carX, double carY, double carYaw,
    double maxLongitudinal) const
{
    double sx, sy;
    sensorOrigin(carX, carY, carYaw, sx, sy);

    const double cosY = std::cos(carYaw);
    const double sinY = std::sin(carYaw);

    // Pre-compute local-frame coordinates for all hits.
    struct LocalHit { double lx, ly; };
    std::vector<LocalHit> local;
    local.reserve(hits.size());
    for (const auto& pt : hits) {
        const double dx =  pt.x - sx;
        const double dy =  pt.y - sy;
        local.push_back({ dx * cosY + dy * sinY,    // lx forward
                         -dx * sinY + dy * cosY });  // ly lateral
    }

    std::vector<Point2D> waypoints;

    for (double target = waypointSpacing_; target <= maxLongitudinal; target += waypointSpacing_) {
        std::vector<double> sliceLeft, sliceRight;

        for (const auto& h : local) {
            if (std::abs(h.lx - target) > SLICE_HALF_WIDTH_M) continue;
            if (h.ly >  WALL_LATERAL_MIN_M) sliceLeft.push_back(h.ly);
            else if (h.ly < -WALL_LATERAL_MIN_M) sliceRight.push_back(std::abs(h.ly));
        }

        if (static_cast<int>(sliceLeft.size())  < MIN_WALL_HITS ||
            static_cast<int>(sliceRight.size()) < MIN_WALL_HITS) {
            continue; // insufficient data at this longitudinal slice
        }

        const double dL = median(sliceLeft);
        const double dR = median(sliceRight);
        const double centreOffset = (dL - dR) / 2.0; // lateral offset of centre from sensor

        // Convert back to world frame.
        // Move target metres forward and centreOffset metres laterally (perpendicular to heading).
        Point2D wp;
        wp.x = sx + target * cosY - centreOffset * sinY;
        wp.y = sy + target * sinY + centreOffset * cosY;
        waypoints.push_back(wp);
    }

    return waypoints;
}

// ============================================================
// Utility
// ============================================================

double TrackAnalyser::euclidean(double ax, double ay, double bx, double by)
{
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

double TrackAnalyser::normaliseAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
}

// ── Private helpers ───────────────────────────────────────────────────────────

void TrackAnalyser::sensorOrigin(double carX, double carY, double carYaw,
                                  double& sx, double& sy) const
{
    sx = carX + ACKERMAN_LASER_OFFSET * std::cos(carYaw);
    sy = carY + ACKERMAN_LASER_OFFSET * std::sin(carYaw);
}

double TrackAnalyser::median(std::vector<double> vals)
{
    if (vals.empty()) return 0.0;
    std::sort(vals.begin(), vals.end());
    return vals[vals.size() / 2];
}
