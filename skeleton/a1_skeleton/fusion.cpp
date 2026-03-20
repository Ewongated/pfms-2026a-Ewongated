#include "fusion.h"
#include <cmath>

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

Fusion::Fusion()
{
    // data_ and cells_ start empty
}

Fusion::Fusion(std::vector<RangerInterface*> rangers)
    : rangers_(rangers)
{
    // data_ and cells_ start empty; populated on grabAndFuseData()
}

// ---------------------------------------------------------------------------
// Interface methods
// ---------------------------------------------------------------------------

void Fusion::setCells(std::vector<pfms::Cell*> cells)
{
    cells_ = cells;
}

std::vector<std::vector<double>> Fusion::getRawRangeData()
{
    return data_;
}

void Fusion::grabAndFuseData()
{
    data_.clear();

    // Acquire data from all sensors
    for (auto ranger : rangers_) {
        data_.push_back(ranger->getData());
    }

    // Fuse each sensor's data against all cells
    for (size_t ri = 0; ri < rangers_.size(); ++ri) {
        RangerInterface* ranger = rangers_.at(ri);
        const std::vector<double>& readings = data_.at(ri);

        pfms::nav_msgs::Odometry pose = ranger->getSensorPose();
        double sx   = pose.position.x;
        double sy   = pose.position.y;
        double syaw = pose.yaw;

        if (ranger->getSensingMethod() == pfms::RangerType::POINT) {
            fuseLaser(readings, ranger, sx, sy, syaw);
        } else if (ranger->getSensingMethod() == pfms::RangerType::CONE) {
            fuseSonar(readings, ranger, sx, sy, syaw);
        }
    }
}

// ---------------------------------------------------------------------------
// Laser fusion
// ---------------------------------------------------------------------------
// The laser sweeps anticlockwise from (syaw - FOV/2) to (syaw + FOV/2).
// FOV and angular resolution are in degrees - converted to radians here.
// Only rays that hit an object (range < maxRange) are used:
//   - endpoint inside cell -> OCCUPIED
//   - ray passes through cell on way to endpoint -> FREE
// Rays at maxRange (no detection) leave all cells unchanged.
// Occupancy takes precedence - OCCUPIED cells are never downgraded.
// ---------------------------------------------------------------------------
void Fusion::fuseLaser(const std::vector<double>& readings,
                        RangerInterface* ranger,
                        double sx, double sy, double syaw)
{
    double fovRad     = ranger->getFieldOfView() * M_PI / 180.0;
    double angResRad  = ranger->getAngularResolution() * M_PI / 180.0;
    double startAngle = syaw - fovRad / 2.0;
    double maxRange   = ranger->getMaxRange();
    double minRange   = ranger->getMinRange();

    for (size_t i = 0; i < readings.size(); ++i) {
        double range = readings.at(i);
        double angle = startAngle + i * angResRad;

        // Discard invalid readings below minimum range
        if (range < minRange) continue;

        // Clip ray range to world boundary (MAP_SIZE = 10m per cell.h).
        // The Gazebo world is bounded at +/-MAP_SIZE so rays cannot travel
        // beyond this boundary. This prevents marking cells outside the world
        // (e.g. at (11,12)) as FREE when rays cannot physically reach them.
        double worldBound = pfms::cell::MAP_SIZE;
        double cosA = std::cos(angle), sinA = std::sin(angle);
        double boundRange = maxRange;
        // Compute ray intersection with world boundary box
        for (int axis = 0; axis < 2; ++axis) {
            double p  = (axis == 0) ? sx : sy;
            double d  = (axis == 0) ? cosA : sinA;
            if (std::abs(d) > 1e-10) {
                double t1 = (-worldBound - p) / d;
                double t2 = ( worldBound - p) / d;
                if (t1 > t2) std::swap(t1, t2);
                if (t2 > 0.0) boundRange = std::min(boundRange, t2);
            }
        }
        double effectiveRange = std::min(range, boundRange);

        double endX = sx + effectiveRange * std::cos(angle);
        double endY = sy + effectiveRange * std::sin(angle);

        // Per spec: "if a ray goes through the cell then it is free"
        // This applies regardless of whether the ray hit an object or reached
        // the world boundary. Only OCCUPIED requires an actual object hit.
        bool rayHit = (range < maxRange);

        for (auto cell : cells_) {
            // Occupancy takes precedence - never downgrade
            if (cell->getState() == pfms::cell::OCCUPIED) continue;

            if (rayHit && pointInCell(cell, endX, endY)) {
                // Ray endpoint inside cell - occupied
                cell->setState(pfms::cell::OCCUPIED);
            } else if (segmentIntersectsCell(cell, sx, sy, endX, endY)) {
                // Ray passed through cell (hit or no hit) - free
                cell->setState(pfms::cell::FREE);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Sonar fusion
// ---------------------------------------------------------------------------
// The sonar returns a single range reading for a cone sector.
// halfFov = FOV/2 in radians (FOV stored in radians for sonar).
// - If the arc at the measured range overlaps the cell -> OCCUPIED
// - If the cone sector passes through the cell          -> FREE
// - If no intersection                                  -> unchanged
// Occupancy takes precedence.
// ---------------------------------------------------------------------------
void Fusion::fuseSonar(const std::vector<double>& readings,
                        RangerInterface* ranger,
                        double sx, double sy, double syaw)
{
    double halfFov = ranger->getFieldOfView() / 2.0;  // FOV in radians for sonar
    double range   = readings.at(0);

    for (auto cell : cells_) {
        if (cell->getState() == pfms::cell::OCCUPIED) continue;

        if (arcEndpointInCell(cell, sx, sy, syaw, halfFov, range)) {
            // The sonar return arc endpoint lies inside this cell
            cell->setState(pfms::cell::OCCUPIED);
        } else if (sectorIntersectsCell(cell, sx, sy, syaw, halfFov, range)) {
            // The sonar cone passes through this cell
            cell->setState(pfms::cell::FREE);
        }
    }
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

bool Fusion::pointInCell(pfms::Cell* cell, double px, double py)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double half = cell->getSide() / 2.0;
    return (px >= cx - half && px <= cx + half &&
            py >= cy - half && py <= cy + half);
}

// Slab method AABB intersection test.
// tmin/tmax parameterize the segment [0,1] where 0=(x1,y1) and 1=(x2,y2).
// Returns true if the segment crosses the cell rectangle.
bool Fusion::segmentIntersectsCell(pfms::Cell* cell,
                                    double x1, double y1,
                                    double x2, double y2)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double half = cell->getSide() / 2.0;

    double xmin = cx - half, xmax = cx + half;
    double ymin = cy - half, ymax = cy + half;
    double dx = x2 - x1, dy = y2 - y1;
    double tmin = 0.0, tmax = 1.0;

    if (std::abs(dx) < 1e-10) {
        if (x1 < xmin || x1 > xmax) return false;
    } else {
        double t1 = (xmin - x1) / dx, t2 = (xmax - x1) / dx;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }

    if (std::abs(dy) < 1e-10) {
        if (y1 < ymin || y1 > ymax) return false;
    } else {
        double t1 = (ymin - y1) / dy, t2 = (ymax - y1) / dy;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }

    return true;
}

// Test if point (px,py) lies within the sonar cone sector:
// origin (sx,sy), pointing sensorYaw, half-angle halfFov, radius range.
bool Fusion::pointInSector(double sx, double sy, double sensorYaw,
                            double halfFov, double range,
                            double px, double py)
{
    double dx = px - sx, dy = py - sy;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > range) return false;

    double angle = std::atan2(dy, dx);
    double diff  = angle - sensorYaw;
    // Normalise diff to [-pi, pi]
    while (diff >  M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;

    return std::abs(diff) <= halfFov;
}

// Check if the sonar sector intersects a cell (used for FREE marking).
// Tests all 4 corners + centre of cell against sector,
// and the two bounding rays of the cone against the cell rectangle.
bool Fusion::sectorIntersectsCell(pfms::Cell* cell,
                                   double sx, double sy, double sensorYaw,
                                   double halfFov, double range)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double half = cell->getSide() / 2.0;

    double testX[5] = { cx-half, cx+half, cx-half, cx+half, cx };
    double testY[5] = { cy-half, cy-half, cy+half, cy+half, cy };

    for (int i = 0; i < 5; ++i) {
        if (pointInSector(sx, sy, sensorYaw, halfFov, range, testX[i], testY[i]))
            return true;
    }

    // Test the two bounding rays of the cone
    double ray1x = sx + range * std::cos(sensorYaw - halfFov);
    double ray1y = sy + range * std::sin(sensorYaw - halfFov);
    double ray2x = sx + range * std::cos(sensorYaw + halfFov);
    double ray2y = sy + range * std::sin(sensorYaw + halfFov);

    if (segmentIntersectsCell(cell, sx, sy, ray1x, ray1y)) return true;
    if (segmentIntersectsCell(cell, sx, sy, ray2x, ray2y)) return true;

    return false;
}

// Check if the sonar arc endpoint (at measured range) overlaps a cell.
// Samples the arc at fine increments (~0.1 degree steps).
// Used for OCCUPIED marking.
bool Fusion::arcEndpointInCell(pfms::Cell* cell,
                                double sx, double sy, double sensorYaw,
                                double halfFov, double range)
{
    int steps = static_cast<int>((2.0 * halfFov) / (M_PI / 1800.0)) + 1;
    for (int i = 0; i <= steps; ++i) {
        double angle = (sensorYaw - halfFov) + i * (2.0 * halfFov) / steps;
        double px = sx + range * std::cos(angle);
        double py = sy + range * std::sin(angle);
        if (pointInCell(cell, px, py)) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// getObjectDentre
// ---------------------------------------------------------------------------
// Uses POINT sensors only. Clusters consecutive laser hits (range < maxRange)
// by spatial proximity. Returns centroid of each cluster as {x, y, z, ...}.
// ---------------------------------------------------------------------------
std::vector<double> Fusion::getObjectDentre()
{
    std::vector<double> centres;

    for (size_t ri = 0; ri < rangers_.size(); ++ri) {
        RangerInterface* ranger = rangers_.at(ri);
        if (ranger->getSensingMethod() != pfms::RangerType::POINT) continue;
        if (data_.empty() || ri >= data_.size()) continue;

        const std::vector<double>& readings = data_.at(ri);
        pfms::nav_msgs::Odometry pose = ranger->getSensorPose();
        double sx   = pose.position.x;
        double sy   = pose.position.y;
        double syaw = pose.yaw;

        double fovRad     = ranger->getFieldOfView() * M_PI / 180.0;
        double angResRad  = ranger->getAngularResolution() * M_PI / 180.0;
        double startAngle = syaw - fovRad / 2.0;
        double maxRange   = ranger->getMaxRange();
        double minRange   = ranger->getMinRange();

        // Max spatial gap between points in same cluster [m]
        const double clusterThreshold = 0.5;

        std::vector<std::pair<double,double>> cluster;

        auto flushCluster = [&]() {
            if (cluster.empty()) return;
            double sumX = 0.0, sumY = 0.0;
            for (auto& pt : cluster) {
                sumX += pt.first;
                sumY += pt.second;
            }
            centres.push_back(sumX / cluster.size());
            centres.push_back(sumY / cluster.size());
            centres.push_back(pose.position.z);
            cluster.clear();
        };

        for (size_t i = 0; i < readings.size(); ++i) {
            double range = readings.at(i);
            double angle = startAngle + i * angResRad;

            if (range < minRange || range >= maxRange) {
                flushCluster();
                continue;
            }

            double px = sx + range * std::cos(angle);
            double py = sy + range * std::sin(angle);

            if (!cluster.empty()) {
                double lastX = cluster.back().first;
                double lastY = cluster.back().second;
                double gap = std::sqrt((px-lastX)*(px-lastX) + (py-lastY)*(py-lastY));
                if (gap > clusterThreshold) flushCluster();
            }
            cluster.push_back({px, py});
        }
        flushCluster();
    }

    return centres;
}

// ---------------------------------------------------------------------------
// getScanningArea
// ---------------------------------------------------------------------------
// Returns the total scanning area [m^2] covered by all sensors.
// For POINT sensors: area of the arc swept = 0.5 * r^2 * FOV (in radians).
// For CONE sensors:  area of the cone sector = 0.5 * r^2 * FOV (in radians).
// Uses the maximum range of each sensor as r.
// ---------------------------------------------------------------------------
double Fusion::getScanningArea()
{
    double totalArea = 0.0;
    for (auto ranger : rangers_) {
        double r   = ranger->getMaxRange();
        double fov = 0.0;
        if (ranger->getSensingMethod() == pfms::RangerType::POINT) {
            // FOV stored in degrees for laser
            fov = ranger->getFieldOfView() * M_PI / 180.0;
        } else {
            // FOV stored in radians for sonar
            fov = ranger->getFieldOfView();
        }
        totalArea += 0.5 * r * r * fov;
    }
    return totalArea;
}