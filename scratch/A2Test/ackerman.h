#ifndef ACKERMAN_H
#define ACKERMAN_H
#include "controller.h"
#include "audi.h"
/**
 * @brief Ackerman controller for the Audi R8 platform.
 *
 * Inherits threading, odometry, goal management and telemetry from Controller.
 * Uses the Audi kinematic model to compute steering. Uses a progress-based
 * throttle/brake profile — full throttle at start, coasting in the middle,
 * braking in the final phase. When a goal is unreachable via constant steering,
 * brakes to a stop then drives a tight full-lock arc at low speed until reachable.
 * A nudge failsafe inches the car forward if it stalls just short of the goal,
 * with a flag to suppress braking while the nudge is active.
 */
class Ackerman : public Controller
{
public:
    Ackerman();
    pfms::PlatformType getPlatformType() override;
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;
protected:
    void driveToGoal(const pfms::geometry_msgs::Point& goal) override;
private:
    Audi audi_;
    static constexpr int    LOOP_PERIOD_MS     = 50;     ///< Control loop period [ms]
    static constexpr double STOP_VELOCITY      = 0.001;   ///< Considered stopped below this [m/s]
    static constexpr double MAX_SPEED          = 8.0;    ///< Used for timeToGoal estimate [m/s]
    static constexpr double MAX_BRAKE_TORQUE   = 8000.0; ///< Full brake torque [Nm]
    static constexpr double MAX_THROTTLE       = 0.7;    ///< Maximum throttle during normal driving [0-1]
    static constexpr double TURNING_THROTTLE   = 0.3;    ///< Throttle for tight arc manoeuvre [0-1]
    static constexpr double MAX_BRAKE_PROGRESS = 0.75;   ///< Progress at which braking begins [0-1]
    static constexpr double BRAKE_RAMP_WIDTH   = 0.05;   ///< Progress range over which brake ramps to max [0-1]
    static constexpr double NUDGE_THRESHOLD    = 0.1;    ///< How close to minDist before nudging [m]
    //Brake is active while nudging - minor
    static constexpr double NUDGE_THROTTLE     = 0.1;    ///< Throttle for nudging toward goal [0-1]
};
#endif // ACKERMAN_H