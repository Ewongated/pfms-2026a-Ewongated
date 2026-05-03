#include "ackerman.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>

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

    pfms::nav_msgs::Odometry odo0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odo0 = odometry_;
    }

    double totalDist    = euclidean(odo0, goal);
    double prevDist     = std::numeric_limits<double>::max();
    bool wasUnreachable = false;

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

        // ── Goal reached check ────────────────────────────────────────────────
        if (dist < tol || (dist > prevDist && prevDist < tol)) {
            break;
        }

        // ── Update tracking variables ─────────────────────────────────────────
        prevDist = dist;

        // ── Ask the Audi model for the correct steering ───────────────────────
        double steering = 0.0;
        double distAudi = 0.0;
        const bool reachable = audi_.computeSteering(odo, goal, steering, distAudi);

        std::cout << "[driveToGoal] reachable=" << reachable
                  << " dist=" << dist
                  << " speed=" << speed
                  << std::endl;

        if (!reachable) {
            // First brake to a stop
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

            // Goal unreachable via constant steering — drive tight arc
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

        // ── Recapture totalDist if we were previously unreachable ─────────────
        if (wasUnreachable) {
            totalDist      = dist;
            wasUnreachable = false;
            prevDist       = std::numeric_limits<double>::max();
            std::cout << "[driveToGoal] Reachable again, recaptured totalDist=" << totalDist << std::endl;
        }

        // ── Velocity-based throttle/brake ─────────────────────────────────────
        const double distCovered  = totalDist - dist;
        const double progress     = std::clamp(distCovered / totalDist, 0.0, 1.0);
        const double stoppingDist = (speed * speed) / (2.0 * DECELERATION);

        double throttle = 0.0;
        double brake    = 0.0;
        if (dist < stoppingDist) {
            brake = MAX_BRAKE_TORQUE;
        } else {
            // Hold low throttle until sufficient progress — prevents full
            // acceleration immediately after exiting arc manoeuvre
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

    // ── Hard stop — keep braking until fully stopped ──────────────────────────
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