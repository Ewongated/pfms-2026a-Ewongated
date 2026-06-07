#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

/**
 * @brief Controller for the Clearpath Husky (skid-steer) platform.
 *
 * Inherits threading, goal management and odometry from Controller.
 *
 * ### Control inputs
 * | Signal      | Range            | Description                          |
 * |-------------|------------------|--------------------------------------|
 * | move_f_b    | [-2.0, 2.0] m/s  | Forward (+) / backward (-) velocity  |
 * | turn_l_r    | [-1.0, 1.0] rad/s| Left (+) / right (-) angular velocity |
 *
 * ### driveToGoal strategy
 * 1. Rotate on the spot until heading error < HEADING_TOL.
 * 2. Drive forward toward goal with proportional heading correction.
 * 3. Once within tolerance_: send zero velocity command (Husky stops instantly).
 *
 * Distance and time estimates use maximum linear and angular velocities.
 * Rotation-on-spot contributes to time but not to distance (position unchanged).
 */
class Skidsteer : public Controller
{
public:
    Skidsteer();
    ~Skidsteer() = default;

    pfms::PlatformType getPlatformType() override;

    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;

private:
    // Platform limits (from assignment brief)
    static constexpr double MAX_LINEAR_VEL   = 2.0;   //!< [m/s]
    static constexpr double MAX_ANGULAR_VEL  = 1.0;   //!< [rad/s]
    static constexpr double STOP_VELOCITY    = 0.05;  //!< [m/s] considered stopped

    // Control tuning
    static constexpr double HEADING_TOL      = 0.05;  //!< [rad] acceptable heading error before driving
    static constexpr double HEADING_KP       = 1.5;   //!< Proportional gain for heading correction while driving
    static constexpr double SLOW_DIST        = 1.0;   //!< [m] begin slowing
    static constexpr int    LOOP_PERIOD_MS   = 50;    //!< Control loop period [ms]
    static constexpr int    DEBUG_INTERVAL   = 20;    //!< Print every N iterations (~1 s)
};

#endif // SKIDSTEER_H
