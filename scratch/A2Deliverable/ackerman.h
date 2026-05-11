#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

/**
 * @brief Ackerman steering controller for the Audi R8 platform.
 *
 * Inherits threading, odometry, goal management and telemetry from Controller.
 * Uses the Audi bicycle-model library to compute the correct steering angle for
 * each control cycle.
 *
 * ### Normal driving (goal reachable via constant-arc path)
 * The Audi library returns a steering angle. A velocity-based brake profile is
 * applied: full brake is commanded when the estimated stopping distance exceeds
 * the remaining distance to the goal. This produces smooth, speed-appropriate
 * deceleration regardless of journey length.
 *
 * ### Recovery driving (goal inside minimum turning circle)
 * The car first brakes to a full stop, then drives a tight full-lock arc at
 * TURNING_THROTTLE toward the goal bearing until the goal becomes reachable via
 * a constant-arc path, at which point normal driving resumes.
 */
class Ackerman : public Controller
{
public:
    Ackerman();

    /** @brief Returns pfms::PlatformType::ACKERMAN. */
    pfms::PlatformType getPlatformType() override;

    /**
     * @brief Checks reachability and estimates distance/time from origin to goal.
     *
     * Delegates to the Audi library's checkOriginToDestination(). If the goal
     * is unreachable (inside the minimum turning circle), distance and time are
     * set to -1.
     *
     * @param origin            Start pose.
     * @param goal              Target point.
     * @param[out] distance     Estimated arc-path distance [m], or -1 if unreachable.
     * @param[out] time         Estimated travel time [s], or -1 if unreachable.
     * @param[out] estimatedGoalPose  Estimated pose when the goal is reached.
     * @return true if the goal is reachable, false otherwise.
     */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    /**
     * @brief Drives the Audi R8 to a single goal.
     *
     * Runs a 50 ms control loop. Each iteration reads odometry, queries the
     * Audi library for the required steering angle, and commands throttle/brake
     * based on the velocity-brake profile. Exits when within tolerance_ of the
     * goal, then issues a hard-brake loop until the car is fully stopped.
     *
     * @param goal Target point in world coordinates.
     */
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;

private:
    Audi audi_; //!< Audi bicycle-model kinematic library

    // ── Control constants ────────────────────────────────────────────────────
    static constexpr int    LOOP_PERIOD_MS        = 50;     //!< Control loop period [ms]
    static constexpr double STOP_VELOCITY         = 0.01;   //!< Speed below which the car is considered stopped [m/s]
    static constexpr double MAX_SPEED             = 8.0;    //!< Maximum speed used for timeToGoal estimate [m/s]
    static constexpr double MAX_BRAKE_TORQUE      = 8000.0; //!< Full brake torque [Nm]
    static constexpr double MAX_THROTTLE          = 1.0;    //!< Maximum throttle in normal driving [0–1]
    static constexpr double TURNING_THROTTLE      = 0.3;    //!< Throttle during tight arc recovery manoeuvre [0–1]
    static constexpr double DECELERATION          = 2.0;    //!< Estimated deceleration under full brake [m/s²]
    static constexpr double LOW_PROGRESS_THROTTLE = 0.05;   //!< Progress fraction below which full throttle is withheld [0–1]
};

#endif // ACKERMAN_H
