#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

/**
 * @brief Controller for the Clearpath Husky (skid-steer) platform.
 *
 * Inherits all threading, goal management, and odometry logic from Controller.
 * Controlled by linear velocity [−2, 2] m/s and angular velocity [−1, 1] rad/s.
 *
 * Driving strategy:
 *  1. Rotate on the spot until facing the goal (angular velocity only)
 *  2. Drive forward with minor heading corrections (linear + small angular)
 *  3. Stop when within tolerance_ of the goal
 *
 * Sensor: Laser and Sonar co-located at the platform centre.
 */
class Skidsteer : public Controller
{
public:
    /// @brief Constructs the Skidsteer controller and connects to the simulator.
    Skidsteer();
    ~Skidsteer() = default;

    // ---- ControllerInterface ----

    pfms::PlatformType getPlatformType() override;

    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    /**
     * @brief Drives the Husky to a single goal using velocity control.
     *
     * Rotates on spot until heading error < ANGLE_THRESHOLD, then
     * drives forward with proportional heading correction. Stops when
     * within tolerance_ or running_ is false.
     *
     * @param goal Target waypoint
     */
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;

private:
    // Skidsteer constants
    static constexpr double MAX_LINEAR_VEL  = 2.0;   // [m/s] forward/backward
    static constexpr double MAX_ANGULAR_VEL = 1.0;   // [rad/s] turn on spot
    static constexpr double ANGLE_THRESHOLD = 0.15;  // [rad] ~8.6 deg — switch to forward drive
    static constexpr double KP_ANGLE        = 1.5;   // proportional gain for heading correction
};

#endif // SKIDSTEER_H
