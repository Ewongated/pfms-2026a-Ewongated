#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

/**
 * @brief Controller for the Clearpath Husky (skid-steer) platform.
 *
 * Inherits threading, goal management, and odometry from Controller.
 * The Husky turns by creating opposing grip forces between its left and right
 * wheel sets and can rotate on the spot without moving its centre position.
 *
 * ### Control signals
 * | Signal      | Range              | Description                           |
 * |-------------|--------------------|---------------------------------------|
 * | move_f_b    | [-2.0, +2.0] m/s   | Forward (+) / backward (-) velocity   |
 * | turn_l_r    | [-1.0, +1.0] rad/s | Left (+) / right (-) angular velocity |
 *
 * ### driveToGoal strategy
 * 1. **Phase 1 — Rotate on spot**: apply ±MAX_ANGULAR_VEL until the heading
 *    error to the goal is below HEADING_TOL. Position does not change.
 * 2. **Phase 2 — Drive forward**: apply up to MAX_LINEAR_VEL with a
 *    proportional heading correction. Speed is reduced linearly within
 *    SLOW_DIST of the goal to ensure clean arrival.
 * 3. **Stop**: send a zero-velocity command when dist < tolerance_.
 *    The Husky (200 kg) stops abruptly when commanded to zero velocity.
 *
 * ### Distance and time estimation
 * checkOriginToDestination() computes time as the sum of the rotation phase
 * (|heading_error| / MAX_ANGULAR_VEL) and the drive phase (dist / MAX_LINEAR_VEL).
 * The rotation phase contributes to time but not to distance because the
 * platform's position does not change while rotating on the spot.
 */
class Skidsteer : public Controller
{
public:
    Skidsteer();
    ~Skidsteer() = default;

    /** @brief Returns pfms::PlatformType::SKIDSTEER. */
    pfms::PlatformType getPlatformType() override;

    /**
     * @brief Estimates travel distance and time from origin to goal.
     *
     * Computes the straight-line distance and estimates travel time using
     * maximum linear and angular velocities. The Husky can always reach any
     * 2-D point, so this function always returns true.
     *
     * @param origin              Start pose.
     * @param goal                Target point.
     * @param[out] distance       Straight-line distance [m].
     * @param[out] time           Estimated travel time (rotate + drive) [s].
     * @param[out] estimatedGoalPose  Pose at the goal (yaw = bearing to goal).
     * @return true always.
     */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    /**
     * @brief Drives the Husky to a single goal using rotate-then-drive.
     *
     * Runs a 50 ms control loop executing Phase 1 then Phase 2 as described
     * in the class overview. Issues a zero-velocity stop command on every exit
     * path, including early exit when running_ becomes false.
     *
     * @param goal Target point in world coordinates.
     */
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;

private:
    // ── Platform limits (from assignment brief) ──────────────────────────────
    static constexpr double MAX_LINEAR_VEL  = 2.0;  //!< Maximum forward speed [m/s]
    static constexpr double MAX_ANGULAR_VEL = 1.0;  //!< Maximum rotation rate [rad/s]
    static constexpr double STOP_VELOCITY   = 0.05; //!< Speed below which the platform is considered stopped [m/s]

    // ── Control tuning ───────────────────────────────────────────────────────
    static constexpr double HEADING_TOL    = 0.05; //!< Heading error below which Phase 2 begins [rad]
    static constexpr double HEADING_KP     = 1.5;  //!< Proportional gain for heading correction during Phase 2
    static constexpr double SLOW_DIST      = 1.0;  //!< Distance at which speed reduction begins [m]
    static constexpr int    LOOP_PERIOD_MS = 50;   //!< Control loop period [ms]
    static constexpr int    DEBUG_INTERVAL = 20;   //!< Console print interval [loop iterations, ~1 s]
};

#endif // SKIDSTEER_H
