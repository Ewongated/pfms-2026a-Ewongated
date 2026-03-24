#include "fusion.h"
#include "ranger.h"
#include <cmath>

Fusion::Fusion()
{
}

Fusion::Fusion(std::vector<RangerInterface*> rangers)
    : rangers_(rangers)
{
}

void Fusion::setCells(std::vector<pfms::Cell*> cells)
{
    cells_ = cells;
}

std::vector<std::vector<double>> Fusion::getRawRangeData()
{
    return data_;
}

//Take Data and send to either fuseLaser or fuseSonar - Sort type
void Fusion::grabAndFuseData()
{
    data_.clear();
    for (auto ranger : rangers_) {
        data_.push_back(ranger->getData());
    }
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

// Returns true if point (px,py) is inside cell bounds - Check endpoint
bool Fusion::pointInCell(pfms::Cell* cell, double px, double py)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double half = cell->getSide() / 2.0;
    return (px >= cx - half && px <= cx + half &&
            py >= cy - half && py <= cy + half);
}

// Slab (AABB) intersection test.
// Returns true if segment (x1,y1)->(x2,y2) crosses the cell rectangle.
// Also returns tEntry and tExit - the parametric values [0,1] where the
// segment enters and exits the cell. tEntry=0 means origin is inside cell.
bool Fusion::segmentIntersectsCell(pfms::Cell* cell,
                                    double x1, double y1,
                                    double x2, double y2,
                                    double& tEntry, double& tExit)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double half = cell->getSide() / 2.0;

    double xmin = cx - half, xmax = cx + half;
    double ymin = cy - half, ymax = cy + half;
    double dx = x2 - x1, dy = y2 - y1;

    tEntry = 0.0;
    tExit  = 1.0;

    // X slab
    if (std::abs(dx) < 1e-10) {
        if (x1 < xmin || x1 > xmax) return false;
    } else {
        double t1 = (xmin - x1) / dx;
        double t2 = (xmax - x1) / dx;
        if (t1 > t2) std::swap(t1, t2);
        tEntry = std::max(tEntry, t1);
        tExit  = std::min(tExit,  t2);
        if (tEntry > tExit) return false;
    }

    // Y slab
    if (std::abs(dy) < 1e-10) {
        if (y1 < ymin || y1 > ymax) return false;
    } else {
        double t1 = (ymin - y1) / dy;
        double t2 = (ymax - y1) / dy;
        if (t1 > t2) std::swap(t1, t2);
        tEntry = std::max(tEntry, t1);
        tExit  = std::min(tExit,  t2);
        if (tEntry > tExit) return false;
    }

    return true;
}


// Laser fusion
// For each ray (clamping inf/NaN to maxRange so free space is captured):
//   1. Check if ray passes through cell (segment intersection)
//   2. If yes, check where endpoint is relative to cell:
//      - tExit >= 1.0: endpoint inside cell -> OCCUPIED (always wins)
//      - tExit <  1.0: ray exited far side  -> FREE (only if not OCCUPIED)

void Fusion::fuseLaser(const std::vector<double>& readings,
                        RangerInterface* ranger,
                        double sx, double sy, double syaw)
{
    double angResRad = ranger->getAngularResolution() * M_PI / 180.0;
    double minRange  = ranger->getMinRange();
    double maxRange  = ranger->getMaxRange();

    // Use actual scan angle_min if available (via Ranger subclass),
    // otherwise fall back to symmetric -FOV/2
    double startAngle;
    Ranger* r = dynamic_cast<Ranger*>(ranger);
    if (r) {
        startAngle = syaw + r->getAngleMin();
    } else {
        startAngle = syaw - ranger->getFieldOfView() * M_PI / 180.0 / 2.0;
    }

    for (size_t i = 0; i < readings.size(); ++i) {
        double range = readings.at(i);

        // Clamp inf/NaN to maxRange — inf gives information up to maxRange of scanner
        if (!std::isfinite(range) || range > maxRange) range = maxRange;
        if (range < minRange) continue;

        double angle = startAngle + i * angResRad;
        double endX  = sx + range * std::cos(angle);
        double endY  = sy + range * std::sin(angle);

        for (auto cell : cells_) {
            double tEntry, tExit;
            if (!segmentIntersectsCell(cell, sx, sy, endX, endY, tEntry, tExit))
                continue;

            if (tExit < 1.0 - 1e-9) {
                // Ray passed completely through cell -> FREE (only if not already OCCUPIED)
                if (cell->getState() != pfms::cell::OCCUPIED)
                    cell->setState(pfms::cell::FREE);
            } else {
                // Ray endpoint is inside cell -> OCCUPIED (always wins)
                cell->setState(pfms::cell::OCCUPIED);
            }
        }
    }
}


// Sonar fusion
// Same logic as laser but applied to the cone sector:
//   1. No readings -> no change
//   2. Check if cone passes through cell
//   3. If yes, check where arc endpoint is relative to cell:
//      - Arc endpoint inside cell -> OCCUPIED (always wins)
//      - Arc endpoint past cell   -> FREE (only if not OCCUPIED)
//      - Arc endpoint before cell -> no change

void Fusion::fuseSonar(const std::vector<double>& readings,
                        RangerInterface* ranger,
                        double sx, double sy, double syaw)
{
    // No readings -> no change
    if (readings.empty()) return;

    double range = readings.at(0);
    if (!std::isfinite(range) || range < ranger->getMinRange()) return;

    double halfFov = ranger->getFieldOfView() / 2.0;  // FOV in radians for sonar

    for (auto cell : cells_) {
        // Check if any part of the cone sector intersects the cell
        if (!sectorIntersectsCell(cell, sx, sy, syaw, halfFov, range)) continue;

        // Cone intersects cell - check where arc endpoint is
        if (arcEndpointInCell(cell, sx, sy, syaw, halfFov, range)) {
            // Arc endpoint inside cell -> OCCUPIED (always wins)
            cell->setState(pfms::cell::OCCUPIED);
        } else if (arcEndpointPastCell(cell, sx, sy, syaw, halfFov, range)) {
            // Arc endpoint past cell -> FREE (only if not already OCCUPIED)
            if (cell->getState() != pfms::cell::OCCUPIED)
                cell->setState(pfms::cell::FREE);
        }
        // else arc endpoint before cell -> no change
    }
}


// Test if point (px,py) lies within the sonar cone sector
bool Fusion::pointInSector(double sx, double sy, double sensorYaw,
                            double halfFov, double range,
                            double px, double py)
{
    double dx = px - sx, dy = py - sy;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > range) return false;
    double angle = std::atan2(dy, dx);
    double diff  = angle - sensorYaw;
    while (diff >  M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return std::abs(diff) <= halfFov;
}

// Check if sonar sector intersects cell (tests corners, centre, bounding rays)
bool Fusion::sectorIntersectsCell(pfms::Cell* cell,
                                   double sx, double sy, double sensorYaw,
                                   double halfFov, double range)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double half = cell->getSide() / 2.0;

    // Test 4 corners and centre
    double testX[5] = { cx-half, cx+half, cx-half, cx+half, cx };
    double testY[5] = { cy-half, cy-half, cy+half, cy+half, cy };
    for (int i = 0; i < 5; ++i) {
        if (pointInSector(sx, sy, sensorYaw, halfFov, range, testX[i], testY[i]))
            return true;
    }

    // Test two bounding rays of cone
    double tEntry, tExit;
    double r1x = sx + range * std::cos(sensorYaw - halfFov);
    double r1y = sy + range * std::sin(sensorYaw - halfFov);
    double r2x = sx + range * std::cos(sensorYaw + halfFov);
    double r2y = sy + range * std::sin(sensorYaw + halfFov);

    if (segmentIntersectsCell(cell, sx, sy, r1x, r1y, tEntry, tExit)) return true;
    if (segmentIntersectsCell(cell, sx, sy, r2x, r2y, tEntry, tExit)) return true;

    return false;
}

// Check if the sonar arc endpoint (at measured range) lands inside the cell.
// Samples arc at fine angular increments.
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

// Check if the sonar arc endpoint is past the cell (FREE condition).
// The arc endpoint is "past" the cell if the cone passes through the cell
// but the arc (at measured range) is beyond the cell's far edge from sensor.
bool Fusion::arcEndpointPastCell(pfms::Cell* cell,
                                  double sx, double sy, double sensorYaw,
                                  double halfFov, double range)
{
    double cx, cy;
    cell->getCentre(cx, cy);
    double dx   = cx - sx, dy = cy - sy;
    double dist = std::sqrt(dx * dx + dy * dy);
    double half = cell->getSide() / 2.0;

    // Arc endpoint is past cell if measured range exceeds far edge of cell
    return range > (dist + half);
}

//Converts laser to cartesian for point checking
std::vector<double> Fusion::getObjectCentre()
{
    std::vector<double> centres;

    for (size_t ri = 0; ri < rangers_.size(); ++ri) {
        RangerInterface* ranger = rangers_.at(ri);
        if (ranger->getSensingMethod() != pfms::RangerType::POINT) continue;
        if (data_.empty() || ri >= data_.size()) continue;

        const std::vector<double>& readings = data_.at(ri);
        pfms::nav_msgs::Odometry pose = ranger->getSensorPose();
        double sx    = pose.position.x;
        double sy    = pose.position.y;
        double syaw  = pose.yaw;

        double angResRad = ranger->getAngularResolution() * M_PI / 180.0;
        double maxRange  = ranger->getMaxRange();
        double minRange  = ranger->getMinRange();

        double startAngle;
        Ranger* r = dynamic_cast<Ranger*>(ranger);
        if (r) {
            startAngle = syaw + r->getAngleMin();
        } else {
            startAngle = syaw - ranger->getFieldOfView() * M_PI / 180.0 / 2.0;
        }

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

            if (!std::isfinite(range) || range < minRange || range >= maxRange) {
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
double Fusion::getScanningArea()
{
    double totalArea = 0.0;
    for (auto ranger : rangers_) {
        double r   = ranger->getMaxRange();
        double fov = 0.0;
        if (ranger->getSensingMethod() == pfms::RangerType::POINT) {
            fov = ranger->getFieldOfView() * M_PI / 180.0;
        } else {
            fov = ranger->getFieldOfView();
        }
        totalArea += 0.5 * r * r * fov;
    }
    return totalArea;
}