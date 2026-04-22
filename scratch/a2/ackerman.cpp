#include "ackerman.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>

/**
 * @brief Constructs the Ackerman controller.
 *
 * Creates a PfmsConnector for the ACKERMAN platform and reads an initial
 * odometry sample so the platform's starting pose is known immediately.
 */
Ackerman::Ackerman()
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(pfms::PlatformType::ACKERMAN);
    updateOdometry();
}

/**
 * @brief Returns the platform type.
 * @return pfms::PlatformType::ACKERMAN
 */
pfms::PlatformType Ackerman::getPlatformType()
{
    return pfms::PlatformType::ACKERMAN;
}

/**
 * @brief Estimates travel distance and time from origin to destination.
 *
 * Delegates to the Audi library which uses the bicycle geometric model.
 *
 * @param origin Starting odometry
 * @param goal Destination point
 * @param[out] distance Arc path distance [m], -1 if unreachable
 * @param[out] time Estimated travel time [s], -1 if unreachable
 * @param[out] estimatedGoalPose Estimated pose when goal is reached
 * @return true if the goal is reachable
 */
bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                         pfms::geometry_msgs::Point goal,
                                         double& distance,
                                         double& time,
                                         pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    bool reachable = audi_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);
    if (!reachable) {
        distance = -1.0;
        time     = -1.0;
    }
    return reachable;
}

/**
 * @brief Drives the Audi R8 to a single goal point.
 *
 * Control strategy:
 *  - Compute steering via Audi::computeSteering() (bicycle model)
 *  - If goal is unreachable via constant arc, steer proportionally toward
 *    goal bearing to reorient, then retry on the next iteration
 *  - Apply full throttle when far from goal, ramp down within BRAKE_DISTANCE
 *  - Full brake and exit when within tolerance_
 *
 * Loop runs at ~50ms per iteration (20Hz). Exits when:
 *  - Platform reaches within tolerance_ of goal, OR
 *  - running_ is set false by execute(false)
 *
 * @param goal Target waypoint
 */
void Ackerman::driveToGoal(const pfms::geometry_msgs::Point& goal)
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

        double dist = euclidean(odo, goal);

        // Debug: print every ~1s (every 20 iterations at 50ms)
        static int debugCount = 0;
        if (++debugCount % 20 == 0) {
            unsigned int goalIdx;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                goalIdx = currentGoalIndex_;
            }
            std::cout << "[Ackerman] goal[" << goalIdx << "]=("
                      << goal.x << "," << goal.y << ")"
                      << " pos=(" << odo.position.x << "," << odo.position.y << ")"
                      << " dist=" << std::fixed << std::setprecision(2) << dist << "m"
                      << "\n";
        }

        // Goal reached — full brake and exit
        if (dist < tol) {
            pfms::commands::Ackerman cmd;
            cmd.seq      = seq++;
            cmd.brake    = MAX_BRAKE_TORQUE;
            cmd.steering = 0.0;
            cmd.throttle = 0.0;
            pfmsConnectorPtr_->send(cmd);
            break;
        }

        // Compute steering via Audi bicycle model
        double steering = 0.0;
        double distAudi = 0.0;
        bool reachable = audi_.computeSteering(odo, goal, steering, distAudi);

        if (!reachable) {
            // Goal unreachable via constant arc — steer toward goal bearing
            // to reorient the vehicle, then retry next iteration
            double bearing = std::atan2(goal.y - odo.position.y,
                                        goal.x - odo.position.x);
            double headingError = normaliseAngle(bearing - odo.yaw);
            steering = std::max(-MAX_STEER_ANGLE,
                       std::min(MAX_STEER_ANGLE, headingError * 0.5));
        }

        steering = std::max(-MAX_STEER_ANGLE, std::min(MAX_STEER_ANGLE, steering));

        // Update live estimates
        {
            std::lock_guard<std::mutex> lock(mutex_);
            distanceToGoal_ = (reachable && distAudi > 0.0) ? distAudi : dist;
            timeToGoal_     = distanceToGoal_ / MAX_SPEED;
        }

        // Throttle ramp — full speed when far, reduce within BRAKE_DISTANCE
        double throttle = 0.0;
        double brake    = 0.0;
        if (dist > BRAKE_DISTANCE) {
            throttle = 1.0;
        } else {
            // Linear ramp from 1.0 at BRAKE_DISTANCE down to 0.05 at tolerance
            double ratio = (dist - tol) / (BRAKE_DISTANCE - tol);
            ratio    = std::max(0.0, std::min(1.0, ratio));
            throttle = 0.05 + 0.95 * ratio;
        }

        pfms::commands::Ackerman cmd;
        cmd.seq      = seq++;
        cmd.brake    = brake;
        cmd.steering = steering;
        cmd.throttle = throttle;
        pfmsConnectorPtr_->send(cmd);

        // Accumulate time in motion
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - loopStart).count();
        loopStart = now;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (std::abs(odo.linear.x) > 0.05) {
                timeTravelled_ += elapsed;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Hard stop
    pfms::commands::Ackerman stop;
    stop.seq      = seq++;
    stop.brake    = MAX_BRAKE_TORQUE;
    stop.steering = 0.0;
    stop.throttle = 0.0;
    pfmsConnectorPtr_->send(stop);
}