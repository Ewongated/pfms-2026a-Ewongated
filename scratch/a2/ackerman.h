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
     * When the goal is unreachable via a constant arc, drives a corrective arc
     * at low speed until the goal enters the reachable envelope, then switches
     * to normal arc tracking with graduated braking.
     *
     * @param goal Target waypoint
     */
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;
private:
    Audi audi_; //!< Audi steering geometry library
    // Ackerman platform constants
    static constexpr double STEERING_RATIO    = 17.3;
    static constexpr double LOCK_TO_LOCK_REVS = 3.2;
    static constexpr double MAX_STEER_ANGLE   = M_PI * LOCK_TO_LOCK_REVS / STEERING_RATIO;
    static constexpr double MAX_BRAKE_TORQUE  = 8000.0; // [Nm]
    static constexpr double MAX_SPEED         = 6.0;    // [m/s] cruising speed
    static constexpr double BRAKE_DISTANCE    = 15.0;   // [m] begin braking ramp
    static constexpr double STOP_VELOCITY     = 0.05;   // [m/s] threshold to consider stopped

    // Reorientation (unreachable goal) constants
    static constexpr double REORIENT_THROTTLE = 0.2;    // [0,1] slow corrective arc speed
    static constexpr int    DIVERGE_LIMIT     = 10;     // iterations of growing dist before braking
};
#endif // ACKERMAN_H