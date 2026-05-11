#include "ackerman.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>

/**
 * @brief Constructs the Ackerman controller and reads initial odometry.
 *
 * Creates a PfmsConnector for the ACKERMAN platform and immediately reads
 * the current odometry so that setGoals() can use a valid origin pose.
 */
Ackerman::Ackerman()
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(pfms::PlatformType::ACKERMAN);
    updateOdometry();
}

pfms::PlatformType Ackerman::getPlatformType()
{
    return pfms::PlatformType::ACKERMAN;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                         pfms::geometry_msgs::Point goal,
                                         double& distance,
                                         double& time,
                                         pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    const bool reachable =
        audi_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);
    if (!reachable) {
        distance = -1.0;
        time     = -1.0;
    }
    return reachable;
}

void Ackerman::driveToGoal(const pfms::geometry_msgs::Point& goal)
{
    unsigned long seq = 1;

    // Snapshot starting position to compute totalDist for the throttle ramp.
    pfms::nav_msgs::Odometry odo0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odo0 = odometry_;
    }

    double totalDist    = euclidean(odo0, goal);
    double prevDist     = std::numeric_limits<double>::max();
    bool   wasUnreachable = false;

    while (running_.load()) {
        updateOdometry();

        pfms::nav_msgs::Odometry odo;
        double tol;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            odo = odometry_;
            tol = tolerance_;
        }

        const double dist  = euclidean(odo, goal);
        const double speed = std::abs(odo.linear.x);

        // ── Goal reached ──────────────────────────────────────────────────────
        // Accept if within tolerance, or if distance has started increasing
        // after passing through the tolerance band (car overshot slightly).
        if (dist < tol || (dist > prevDist && prevDist < tol)) {
            break;
        }
        prevDist = dist;

        // ── Query Audi library for required steering angle ────────────────────
        double steering = 0.0;
        double distAudi = 0.0;
        const bool reachable = audi_.computeSteering(odo, goal, steering, distAudi);

        if (!reachable) {
            // Goal is inside the minimum turning circle — brake to a stop first.
            while (std::abs(odo.linear.x) > STOP_VELOCITY) {
                pfms::commands::Ackerman brake_cmd{};
                brake_cmd.seq      = seq++;
                brake_cmd.brake    = MAX_BRAKE_TORQUE;
                brake_cmd.steering = 0.0;
                brake_cmd.throttle = 0.0;
                pfmsConnectorPtr_->send(brake_cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
                updateOdometry();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    odo = odometry_;
                }
            }

            // Drive a tight full-lock arc toward the goal bearing.
            const double bearing      = std::atan2(goal.y - odo.position.y,
                                                    goal.x - odo.position.x);
            const double headingError = normaliseAngle(bearing - odo.yaw);
            steering = (headingError > 0.0) ? 1.0 : -1.0;

            pfms::commands::Ackerman cmd{};
            cmd.seq      = seq++;
            cmd.brake    = 0.0;
            cmd.steering = steering;
            cmd.throttle = TURNING_THROTTLE;
            pfmsConnectorPtr_->send(cmd);

            wasUnreachable = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
            continue;
        }

        // ── Recapture reference distance after a recovery arc ─────────────────
        // totalDist is reset so the throttle ramp restarts cleanly from the
        // current position rather than from the original start.
        if (wasUnreachable) {
            totalDist      = dist;
            wasUnreachable = false;
            prevDist       = std::numeric_limits<double>::max();
        }

        // ── Velocity-based throttle / brake profile ───────────────────────────
        const double distCovered  = totalDist - dist;
        const double progress     = std::clamp(distCovered / totalDist, 0.0, 1.0);
        const double stoppingDist = (speed * speed) / (2.0 * DECELERATION);

        double throttle = 0.0;
        double brake    = 0.0;

        if (dist < stoppingDist) {
            // Within predicted stopping distance — apply full brake.
            brake = MAX_BRAKE_TORQUE;
        } else {
            // Ramp throttle down linearly with progress. Hold TURNING_THROTTLE
            // at the start of each leg to prevent aggressive acceleration
            // immediately after exiting a recovery arc.
            const double rawThrottle = (1.0 - (progress * 2.0)) * MAX_THROTTLE;
            throttle = (progress < LOW_PROGRESS_THROTTLE)
                       ? TURNING_THROTTLE
                       : rawThrottle;
            throttle = std::max(0.0, throttle);
        }

        pfms::commands::Ackerman cmd{};
        cmd.seq      = seq++;
        cmd.brake    = brake;
        cmd.steering = steering;
        cmd.throttle = throttle;
        pfmsConnectorPtr_->send(cmd);

        // ── Update telemetry ──────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lock(mutex_);
            distanceToGoal_ = dist;
            timeToGoal_     = distanceToGoal_ / MAX_SPEED;
            if (speed > STOP_VELOCITY)
                timeTravelled_ += LOOP_PERIOD_MS / 1000.0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
    }

    // ── Hard stop — brake until fully stopped before returning ────────────────
    while (true) {
        updateOdometry();
        pfms::nav_msgs::Odometry odo;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            odo = odometry_;
        }
        pfms::commands::Ackerman stop{};
        stop.seq      = seq++;
        stop.brake    = MAX_BRAKE_TORQUE;
        stop.steering = 0.0;
        stop.throttle = 0.0;
        pfmsConnectorPtr_->send(stop);
        if (std::abs(odo.linear.x) < STOP_VELOCITY) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
    }
}
