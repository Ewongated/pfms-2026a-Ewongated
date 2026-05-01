#include "ackerman.h"
#include <chrono>
#include <thread>
#include <cmath>

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
    double prevDist = std::numeric_limits<double>::max();

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

        std::cout << "[driveToGoal] dist=" << dist << " prevDist=" << prevDist << " tol=" << tol << std::endl;

        // ── Goal reached — brake until stopped ───────────────────────────────
        if (dist < tol || (dist > prevDist && prevDist < tol)) {
            break;
        }

        // ── Ask the Audi model for the correct steering ───────────────────────
        double steering = 0.0;
        double distAudi = 0.0;
        const bool reachable = audi_.computeSteering(odo, goal, steering, distAudi);

        if (!reachable) {
            // Goal inside turning circle — steer full lock toward bearing,
            // let the simulator clamp to max_steer_angle_ internally
            const double bearing      = std::atan2(goal.y - odo.position.y,
                                                    goal.x - odo.position.x);
            const double headingError = normaliseAngle(bearing - odo.yaw);
            steering = (headingError > 0.0) ? 1.0 : -1.0;
        }

        // ── Throttle and brake: counter-ramp within BRAKE_DISTANCE ───────────
        // ratio: 1.0 far away, 0.0 at goal
        // throttle follows ratio, brake opposes it — actively fights momentum
        double throttle = 1.0;
        double brake    = 0.0;
        if (dist < BRAKE_DISTANCE) {
            const double ratio = dist / BRAKE_DISTANCE;  // 1.0 → 0.0 as we close in
            throttle = ratio;
            brake    = (1.0 - ratio) * MAX_BRAKE_TORQUE;
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
            distanceToGoal_ = (reachable && distAudi > 0.0) ? distAudi : dist;
            timeToGoal_     = distanceToGoal_ / MAX_SPEED;
            if (speed > STOP_VELOCITY)
                timeTravelled_ += LOOP_PERIOD_MS / 1000.0;
        }

        prevDist = dist;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
    }

    // ── Hard stop on every exit path ─────────────────────────────────────────
    pfms::commands::Ackerman stop{};
    stop.seq      = seq++;
    stop.brake    = MAX_BRAKE_TORQUE;
    stop.steering = 0.0;
    stop.throttle = 0.0;
    pfmsConnectorPtr_->send(stop);
}