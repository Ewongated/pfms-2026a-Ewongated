#include "skidsteer.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>

/**
 * @brief Constructs the Skidsteer controller and reads initial odometry.
 */
Skidsteer::Skidsteer()
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(pfms::PlatformType::SKIDSTEER);
    updateOdometry();
}

pfms::PlatformType Skidsteer::getPlatformType()
{
    return pfms::PlatformType::SKIDSTEER;
}

/**
 * @brief Estimates travel distance and time from origin to goal.
 *
 * Strategy matches driveToGoal:
 *  1. Rotate on spot to goal bearing  → time = |heading_error| / MAX_ANGULAR_VEL, dist = 0
 *  2. Drive straight to goal          → time = dist / MAX_LINEAR_VEL
 *
 * @param origin           Start odometry
 * @param goal             Target point
 * @param[out] distance    Straight-line distance [m]
 * @param[out] time        Estimated travel time [s]
 * @param[out] estimatedGoalPose  Estimated pose at goal (yaw = heading to goal)
 * @return true always (Husky can always reach any 2-D point)
 */
bool Skidsteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    const double dx      = goal.x - origin.position.x;
    const double dy      = goal.y - origin.position.y;
    distance             = std::sqrt(dx * dx + dy * dy);

    const double bearing      = std::atan2(dy, dx);
    const double headingError = std::abs(normaliseAngle(bearing - origin.yaw));

    // Time to rotate on spot + time to drive straight
    time = headingError / MAX_ANGULAR_VEL + distance / MAX_LINEAR_VEL;

    estimatedGoalPose          = origin;
    estimatedGoalPose.position = goal;
    estimatedGoalPose.yaw      = bearing;

    return true;
}

/**
 * @brief Drives the Husky to a single goal using rotate-then-drive.
 *
 * Phase 1 — Rotate on spot:
 *   Apply turn_l_r = ±MAX_ANGULAR_VEL toward goal bearing until
 *   heading error < HEADING_TOL.
 *
 * Phase 2 — Drive forward:
 *   Apply move_f_b = MAX_LINEAR_VEL (or SLOW_DIST ramp near goal)
 *   with proportional heading correction via turn_l_r.
 *   Exit when dist < tolerance_. Send zero-velocity stop command.
 */
void Skidsteer::driveToGoal(const pfms::geometry_msgs::Point& goal)
{
    unsigned long seq        = 1;
    int           debugCount = 0;

    while (running_.load()) {
        updateOdometry();

        pfms::nav_msgs::Odometry odo;
        double tol;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            odo = odometry_;
            tol = tolerance_;
        }

        const double dx      = goal.x - odo.position.x;
        const double dy      = goal.y - odo.position.y;
        const double dist    = std::sqrt(dx * dx + dy * dy);
        const double bearing = std::atan2(dy, dx);
        const double headingError = normaliseAngle(bearing - odo.yaw);

        if (++debugCount % DEBUG_INTERVAL == 0) {
            unsigned int goalIdx;
            { std::lock_guard<std::mutex> lock(mutex_); goalIdx = currentGoalIndex_; }
            std::cout << "[Skidsteer] goal[" << goalIdx << "]=("
                      << std::fixed << std::setprecision(2) << goal.x << "," << goal.y << ")"
                      << " pos=(" << odo.position.x << "," << odo.position.y << ")"
                      << " dist=" << dist << "m hdg_err=" << headingError << "rad\n";
        }

        // Goal reached
        if (dist < tol) {
            pfms::commands::SkidSteer stop{};
            stop.seq      = seq++;
            stop.move_f_b = 0.0;
            stop.turn_l_r = 0.0;
            pfmsConnectorPtr_->send(stop);
            break;
        }

        pfms::commands::SkidSteer cmd{};
        cmd.seq = seq++;

        if (std::abs(headingError) > HEADING_TOL) {
            // Phase 1: rotate on spot toward goal bearing
            cmd.move_f_b = 0.0;
            cmd.turn_l_r = (headingError > 0.0) ? MAX_ANGULAR_VEL : -MAX_ANGULAR_VEL;
        } else {
            // Phase 2: drive forward with proportional heading correction
            const double slowRatio =
                std::max(0.2, std::min(1.0, dist / SLOW_DIST));
            cmd.move_f_b = MAX_LINEAR_VEL * slowRatio;

            // Clamp heading correction to angular velocity limit
            const double correction = HEADING_KP * headingError;
            cmd.turn_l_r = std::max(-MAX_ANGULAR_VEL,
                                    std::min(MAX_ANGULAR_VEL, correction));
        }

        pfmsConnectorPtr_->send(cmd);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            distanceToGoal_ = dist;
            timeToGoal_     = dist / MAX_LINEAR_VEL;
        }

        // Accumulate in-motion time
        {
            const double dt = static_cast<double>(LOOP_PERIOD_MS) / 1000.0;
            std::lock_guard<std::mutex> lock(mutex_);
            if (std::abs(odo.linear.x) > STOP_VELOCITY) timeTravelled_ += dt;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
    }

    // Hard stop on every exit
    pfms::commands::SkidSteer stop{};
    stop.seq      = seq++;
    stop.move_f_b = 0.0;
    stop.turn_l_r = 0.0;
    pfmsConnectorPtr_->send(stop);
}
