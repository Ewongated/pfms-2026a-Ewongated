#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

/**
 * @brief Ackerman controller for the Audi R8 platform.
 *
 * Inherits threading, odometry, goal management and telemetry from Controller.
 * Uses the Audi kinematic model to compute steering. Within BRAKE_DISTANCE,
 * throttle ramps down and brake ramps up together to actively fight momentum.
 * Full brake is applied once within tolerance.
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
    static constexpr double BRAKE_DISTANCE   = 10.0;   ///< Distance at which braking begins [m]
    static constexpr double MAX_SPEED        = 8.0;    ///< Used for timeToGoal estimate [m/s]
    static constexpr double MAX_BRAKE_TORQUE = 8000.0; ///< Full brake [Nm]
};

#endif // ACKERMAN_H