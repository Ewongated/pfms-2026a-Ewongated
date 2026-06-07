#ifndef ACKERMAN_H
#define ACKERMAN_H
#include "controller.h"
#include "audi.h"
/**
 * @brief Ackerman controller for the Audi R8 platform.
 *
 * Inherits threading, odometry, goal management and telemetry from Controller.
 * Uses the Audi kinematic model to compute steering. Uses velocity-based braking
 * — full brake is applied when the computed stopping distance exceeds the distance
 * to the goal, ensuring consistent behaviour regardless of journey length or speed.
 * When a goal is unreachable via constant steering, brakes to a stop then drives
 * a tight full-lock arc at low speed until reachable. After re-entering normal
 * control from an arc, throttle is held low until sufficient progress is made.
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
    static constexpr int    LOOP_PERIOD_MS   = 50;     ///< Control loop period [ms]
    static constexpr double STOP_VELOCITY    = 0.01;   ///< Considered stopped below this [m/s]
    static constexpr double MAX_SPEED        = 8.0;    ///< Used for timeToGoal estimate [m/s]
    static constexpr double MAX_BRAKE_TORQUE = 8000.0; ///< Full brake torque [Nm]
    static constexpr double MAX_THROTTLE     = 1.0;    ///< Maximum throttle during normal driving [0-1]
    static constexpr double TURNING_THROTTLE = 0.3;    ///< Throttle for tight arc manoeuvre [0-1]
    static constexpr double DECELERATION     = 2.0;    ///< Estimated deceleration under full brake [m/s²]
    static constexpr double LOW_PROGRESS_THROTTLE = 0.05; ///< Progress threshold before full throttle allowed [0-1]
};
#endif // ACKERMAN_H