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
 *
 *  REACHABLE (Audi bicycle model returns true):
 *   - Apply steering from computeSteering()
 *   - Full throttle when dist > BRAKE_DISTANCE
 *   - Linear throttle ramp to zero within BRAKE_DISTANCE (no floor)
 *   - Once within tolerance_, brake loop until velocity < STOP_VELOCITY
 *
 *  UNREACHABLE (goal inside minimum turning circle):
 *   - Drive a corrective arc at REORIENT_THROTTLE with full-lock steering
 *     toward the goal bearing. This repositions the car until the goal
 *     exits the turning circle and computeSteering() returns true.
 *   - Divergence guard: if distance grows for DIVERGE_LIMIT consecutive
 *     iterations the car has looped past the approach corridor — apply
 *     full brake and reset the counter before trying again.
 *
 * This fixes the failure seen in the log where reachable was always false,
 * causing the car to accelerate to 36 m/s in a widening spiral.
 *
 * Loop runs at ~50ms per iteration (20Hz). Exits when:
 *  - Platform comes to rest within tolerance_ of goal, OR
 *  - running_ is set false by execute(false)
 *
 * @param goal Target waypoint
 */
void Ackerman::driveToGoal(const pfms::geometry_msgs::Point& goal)
{
    auto loopStart = std::chrono::steady_clock::now();
    unsigned long seq = 1;

    double prevDist  = 1e9;
    int divergeCount = 0;

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
                      << std::fixed << std::setprecision(2) << goal.x << ","
                      << std::fixed << std::setprecision(2) << goal.y << ")"
                      << " pos=(" << odo.position.x << "," << odo.position.y << ")"
                      << " dist=" << dist << "m"
                      << " vel=" << odo.linear.x << "m/s"
                      << "\n";
        }

        // ----------------------------------------------------------------
        // Goal reached — brake and wait until the car has fully stopped.
        // A single brake command is insufficient at speed; without this loop
        // the car overshoots and begins the next arc from the wrong position.
        // ----------------------------------------------------------------
        if (dist < tol) {
            while (running_.load()) {
                pfms::commands::Ackerman cmd;
                cmd.seq      = seq++;
                cmd.brake    = MAX_BRAKE_TORQUE;
                cmd.steering = 0.0;
                cmd.throttle = 0.0;
                pfmsConnectorPtr_->send(cmd);

                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                updateOdometry();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    odo = odometry_;
                }
                if (std::abs(odo.linear.x) < STOP_VELOCITY) break;
            }
            break;
        }

        // ----------------------------------------------------------------
        // Compute steering via Audi bicycle model
        // ----------------------------------------------------------------
        double steering = 0.0;
        double distAudi = 0.0;
        bool reachable  = audi_.computeSteering(odo, goal, steering, distAudi);

        double throttle = 0.0;
        double brake    = 0.0;

        if (!reachable) {
            // ----------------------------------------------------------------
            // UNREACHABLE: goal is inside the car's minimum turning circle.
            //
            // The previous code applied throttle = 0.3 unconditionally here.
            // This caused unbounded acceleration (observed: 36 m/s) because
            // the car spiralled away — reachable never became true and there
            // was nothing to limit speed.
            //
            // Fix: drive a slow corrective arc (REORIENT_THROTTLE) with full
            // lock toward the goal bearing. Monitor whether distance is
            // shrinking. If it grows for DIVERGE_LIMIT iterations we have
            // looped past the approach corridor — apply full brake to stop,
            // reset the counter, and try again from the new position.
            // ----------------------------------------------------------------
            double bearing      = std::atan2(goal.y - odo.position.y,
                                             goal.x - odo.position.x);
            double headingError = normaliseAngle(bearing - odo.yaw);

            // Full lock in the direction that closes the heading error fastest
            steering = (headingError > 0.0) ? MAX_STEER_ANGLE : -MAX_STEER_ANGLE;

            // Divergence detection
            if (dist > prevDist + 0.1) {
                ++divergeCount;
            } else {
                divergeCount = 0;
            }

            if (divergeCount >= DIVERGE_LIMIT) {
                // Looping away from goal — stop and reset
                brake        = MAX_BRAKE_TORQUE;
                throttle     = 0.0;
                divergeCount = 0;
            } else {
                // Slow corrective arc — steers toward goal without runaway speed
                throttle = REORIENT_THROTTLE;
                brake    = 0.0;
            }

        } else {
            // ----------------------------------------------------------------
            // REACHABLE: follow the computed arc with graduated throttle.
            // ----------------------------------------------------------------
            divergeCount = 0;
            steering = std::max(-MAX_STEER_ANGLE, std::min(MAX_STEER_ANGLE, steering));

            if (dist > BRAKE_DISTANCE) {
                throttle = 1.0;
            } else {
                // Linear ramp from 1.0 at BRAKE_DISTANCE down to 0.0 at tolerance.
                // No 0.05 floor — prevents throttle fighting the brake at goal edge.
                double ratio = (dist - tol) / (BRAKE_DISTANCE - tol);
                ratio    = std::max(0.0, std::min(1.0, ratio));
                throttle = ratio;
            }
        }

        prevDist = dist;

        // Update live estimates
        {
            std::lock_guard<std::mutex> lock(mutex_);
            distanceToGoal_ = (reachable && distAudi > 0.0) ? distAudi : dist;
            timeToGoal_     = distanceToGoal_ / MAX_SPEED;
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
            if (std::abs(odo.linear.x) > STOP_VELOCITY) {
                timeTravelled_ += elapsed;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Hard stop on exit (covers both goal-reached and running_=false paths)
    pfms::commands::Ackerman stop;
    stop.seq      = seq++;
    stop.brake    = MAX_BRAKE_TORQUE;
    stop.steering = 0.0;
    stop.throttle = 0.0;
    pfmsConnectorPtr_->send(stop);
}