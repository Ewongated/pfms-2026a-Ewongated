#include "skidsteer.h"
#include <chrono>
#include <thread>
#include <cmath>

/**
 * @brief Constructs the Skidsteer controller.
 *
 * Creates a PfmsConnector for the SKIDSTEER platform and reads an initial
 * odometry sample so the platform's starting pose is known immediately.
 */
Skidsteer::Skidsteer()
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(pfms::PlatformType::SKIDSTEER);
    updateOdometry();
}

/**
 * @brief Returns the platform type.
 * @return pfms::PlatformType::SKIDSTEER
 */
pfms::PlatformType Skidsteer::getPlatformType()
{
    return pfms::PlatformType::SKIDSTEER;
}

/**
 * @brief Estimates travel distance and time from origin to destination.
 *
 * Distance is the straight-line Euclidean distance. Time accounts for
 * both the on-the-spot rotation (zero linear distance) and the forward drive.
 *
 * @param origin Starting odometry
 * @param goal Destination point
 * @param[out] distance Straight-line distance to goal [m]
 * @param[out] time Estimated travel time including rotation [s]
 * @param[out] estimatedGoalPose Estimated pose at goal (yaw pointing toward goal)
 * @return true always (Skidsteer can reach any point by rotating first)
 */
bool Skidsteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;
    distance  = std::sqrt(dx*dx + dy*dy);

    double targetYaw  = std::atan2(dy, dx);
    double angleDiff  = std::abs(normaliseAngle(targetYaw - origin.yaw));

    double rotateTime = angleDiff / MAX_ANGULAR_VEL;
    double driveTime  = distance  / MAX_LINEAR_VEL;
    time = rotateTime + driveTime;

    estimatedGoalPose            = origin;
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.position.z = goal.z;
    estimatedGoalPose.yaw        = targetYaw;

    return true;
}

/**
 * @brief Drives the Husky to a single goal using two-phase velocity control.
 *
 * Phase 1 — Rotate on spot:
 *   Send turn_l_r = ±MAX_ANGULAR_VEL with move_f_b = 0 until heading error
 *   drops below ANGLE_THRESHOLD. Position does not change during this phase.
 *
 * Phase 2 — Drive forward:
 *   Send move_f_b = MAX_LINEAR_VEL with a proportional turn_l_r correction
 *   (KP_ANGLE * error, clamped to MAX_ANGULAR_VEL) to handle drift.
 *
 * Field names per pfms_types.h pfms::commands::SkidSteer:
 *   seq      — monotonically increasing command sequence number
 *   turn_l_r — angular velocity [rad/s], left (CCW) positive
 *   move_f_b — linear velocity [m/s], forward positive
 *
 * @param goal Target waypoint
 */
void Skidsteer::driveToGoal(const pfms::geometry_msgs::Point& goal)
{
    auto loopStart = std::chrono::steady_clock::now();
    unsigned long seq = 1;

    while (running_.load()) {
        updateOdometry();
        pfms::nav_msgs::Odometry odo;
        double tol;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            odo = odometry_;
            tol = tolerance_;
        }

        double dx   = goal.x - odo.position.x;
        double dy   = goal.y - odo.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Update live estimates
        {
            std::lock_guard<std::mutex> lock(mutex_);
            distanceToGoal_ = dist;
            timeToGoal_     = dist / MAX_LINEAR_VEL;
        }

        // Goal reached?
        if (dist < tol) {
            pfms::commands::SkidSteer cmd;
            cmd.seq      = seq++;
            cmd.move_f_b = 0.0;
            cmd.turn_l_r = 0.0;
            pfmsConnectorPtr_->send(cmd);
            break;
        }

        double targetYaw  = std::atan2(dy, dx);
        double angleError = normaliseAngle(targetYaw - odo.yaw);

        pfms::commands::SkidSteer cmd;
        cmd.seq = seq++;

        if (std::abs(angleError) > ANGLE_THRESHOLD) {
            // Phase 1: rotate on spot — no forward motion
            cmd.move_f_b = 0.0;
            cmd.turn_l_r = (angleError > 0.0) ? MAX_ANGULAR_VEL : -MAX_ANGULAR_VEL;
        } else {
            // Phase 2: drive forward with proportional heading correction
            cmd.move_f_b = MAX_LINEAR_VEL;
            double angular = KP_ANGLE * angleError;
            angular = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, angular));
            cmd.turn_l_r = angular;
        }

        pfmsConnectorPtr_->send(cmd);

        // Accumulate time in motion
        auto now     = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - loopStart).count();
        loopStart = now;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (std::abs(odo.linear.x) > 0.05 || std::abs(cmd.turn_l_r) > 0.05) {
                timeTravelled_ += elapsed;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Ensure platform is fully stopped
    pfms::commands::SkidSteer stop;
    stop.seq      = seq++;
    stop.move_f_b = 0.0;
    stop.turn_l_r = 0.0;
    pfmsConnectorPtr_->send(stop);
}
