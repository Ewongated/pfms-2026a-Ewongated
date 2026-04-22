#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

/**
 * @brief Controller for the Audi R8 (Ackerman steering) platform.
 *
 * Inherits all threading, goal management, and odometry logic from Controller.
 * Uses the Audi library to compute steering angles and distance/time estimates.
 *
 * Control inputs: throttle [0,1], brake [0, MAX_BRAKE_TORQUE], steering [±MAX_STEER_ANGLE]
 *
 * Sensor: Laser and Sonar co-located 3.725m forward of the rear-axle centre.
 */
class Ackerman : public Controller
{
public:
    /// @brief Constructs the Ackerman controller and connects to the simulator.
    Ackerman();
    ~Ackerman() = default;

    // ---- ControllerInterface ----

    pfms::PlatformType getPlatformType() override;

    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    /**
     * @brief Drives the Audi R8 to a single goal using throttle/steering control.
     *
     * Uses the Audi library for steering computation (bicycle model).
     * Applies proportional throttle when far from goal and brakes as it approaches.
     * Loops until the platform is within tolerance_ of the goal or running_ is false.
     *
     * @param goal Target waypoint
     */
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;

private:
    Audi audi_; //!< Audi steering geometry library

    // Ackerman constants
    static constexpr double STEERING_RATIO   = 17.3;
    static constexpr double LOCK_TO_LOCK_REVS = 3.2;
    static constexpr double MAX_STEER_ANGLE  = M_PI * LOCK_TO_LOCK_REVS / STEERING_RATIO;
    static constexpr double MAX_BRAKE_TORQUE = 8000.0; // [Nm]
    static constexpr double MAX_SPEED        = 6.0;    // [m/s] cruising speed
    static constexpr double BRAKE_DISTANCE   = 10.0;    // [m] start braking within this distance
};

#endif // ACKERMAN_H
