#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include "pfms_types.h"
#include "pfmsconnector.h"
#include <cmath>
#include <memory>

/**
 * @brief Base class for all ranger (range sensor) types.
 *
 * Implements all shared getter methods from RangerInterface and stores
 * the common sensor properties. Derived classes (Laser, Sonar) set these
 * in their constructors and implement getData().
 *
 * The sensor pose is computed relative to the vehicle reference frame
 * using a fixed forward/lateral/vertical offset — this transform is static
 * and does not change while the platform is running.
 */
class Ranger : public RangerInterface
{
public:
    double                   getAngularResolution() override;
    pfms::nav_msgs::Odometry getSensorPose()        override;
    double                   getFieldOfView()       override;
    double                   getMaxRange()          override;
    double                   getMinRange()          override;
    pfms::RangerType         getSensingMethod()     override;

    /// @brief Returns the scan's angle_min relative to sensor heading [rad]
    double getAngleMin() { return scanAngleMin_; }

    virtual std::vector<double> getData() = 0;

protected:
    pfms::PlatformType             platformType_;
    pfms::nav_msgs::Odometry       sensorPose_;
    std::shared_ptr<PfmsConnector> connector_;

    double           angularResolution_;
    double           fieldOfView_;
    double           maxRange_;
    double           minRange_;
    pfms::RangerType sensingMethod_;

    double sensorForwardOffset_;
    double sensorLateralOffset_;
    double sensorVerticalOffset_;
    double scanAngleMin_ = 0.0;

    /**
     * @brief Computes world-frame sensor pose from platform odometry.
     *
     * Called inside getData() after each fresh odometry read.
     * Uses fixed static offsets — NOT called in the constructor.
     *
     * @param platformOdo Current platform odometry
     */
    void computeSensorPose(const pfms::nav_msgs::Odometry& platformOdo);
};

#endif // RANGER_H