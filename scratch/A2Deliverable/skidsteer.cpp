#include "skidsteer.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>

/**
 * @brief Constructs the Skidsteer controller and reads initial odometry.
 *
 * Creates a PfmsConnector for the SKIDSTEER platform and immediately reads
 * the current odometry so that setGoals() can use a valid origin pose.
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

bool Skidsteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    const double dx = goal.x - origin.position.x;
    const double dy = goal.y - origin.position.y;
    distance        = std::sqrt(dx * dx + dy * dy);

    const double bearing      = std::atan2(dy, dx);
    const double headingError = std::abs(normaliseAngle(bearing - origin.yaw));

    // Total time = rotation phase + straight drive phase.
    // The rotation phase does not change position, only orientation.
    time = headingError / MAX_ANGULAR_VEL + distance / MAX_LINEAR_VEL;

    estimatedGoalPose          = origin;
    estimatedGoalPose.position = goal;
    estimatedGoalPose.yaw      = bearing;

    return true; // Husky can always reach any 2-D point
}

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

        const double dx           = goal.x - odo.position.x;
        const double dy           = goal.y - odo.position.y;
        const double dist         = std::sqrt(dx * dx + dy * dy);
        const double bearing      = std::atan2(dy, dx);
        const double headingError = normaliseAngle(bearing - odo.yaw);

        // Periodic console output for progress monitoring.
        if (++debugCount % DEBUG_INTERVAL == 0) {
            unsigned int goalIdx;
            { std::lock_guard<std::mutex> lock(mutex_); goalIdx = currentGoalIndex_; }
            std::cout << "[Skidsteer] goal[" << goalIdx << "]=("
                      << std::fixed << std::setprecision(2) << goal.x << "," << goal.y << ")"
                      << " pos=(" << odo.position.x << "," << odo.position.y << ")"
                      << " dist=" << dist << "m hdg_err=" << headingError << "rad\n";
        }

        // ── Goal reached ──────────────────────────────────────────────────────
        if (dist < tol) {
            break;
        }

        pfms::commands::SkidSteer cmd{};
        cmd.seq = seq++;

        if (std::abs(headingError) > HEADING_TOL) {
            // Phase 1: rotate on spot toward the goal bearing.
            cmd.move_f_b = 0.0;
            cmd.turn_l_r = (headingError > 0.0) ? MAX_ANGULAR_VEL : -MAX_ANGULAR_VEL;
        } else {
            // Phase 2: drive forward with proportional heading correction.
            // Reduce speed linearly as the platform approaches the goal.
            const double slowRatio =
                std::max(0.2, std::min(1.0, dist / SLOW_DIST));
            cmd.move_f_b = MAX_LINEAR_VEL * slowRatio;

            // Clamp heading correction to the angular velocity limit.
            const double correction = HEADING_KP * headingError;
            cmd.turn_l_r = std::max(-MAX_ANGULAR_VEL,
                                    std::min(MAX_ANGULAR_VEL, correction));
        }

        pfmsConnectorPtr_->send(cmd);

        // ── Update telemetry ──────────────────────────────────────────────────
        {
            const double dt = static_cast<double>(LOOP_PERIOD_MS) / 1000.0;
            std::lock_guard<std::mutex> lock(mutex_);
            distanceToGoal_ = dist;
            timeToGoal_     = dist / MAX_LINEAR_VEL;
            timeTravelled_ += dt;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
    }

    // ── Zero-velocity stop — required on every exit path ─────────────────────
    pfms::commands::SkidSteer stop{};
    stop.seq      = seq++;
    stop.move_f_b = 0.0;
    stop.turn_l_r = 0.0;
    pfmsConnectorPtr_->send(stop);
}
