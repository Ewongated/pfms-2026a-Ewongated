#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include "pfms_types.h"
#include "pfmsconnector.h"
#include <cmath>
#include <mutex>

/**
 * @brief Abstract base class for all ranger (sensor) types.
 *
 * Holds state common to Laser and Sonar: a PfmsConnector, platform type,
 * sensor geometry parameters and the last known sensor pose in world frame.
 *
 * Derived classes must implement getData() and set all geometry fields
 * (fov_, angularResolution_, maxRange_, minRange_, sensingMethod_) in
 * their constructors.
 */
class Ranger : public RangerInterface
{
public:
    explicit Ranger(pfms::PlatformType type);

    double getAngularResolution(void) override;
    pfms::nav_msgs::Odometry getSensorPose(void) override;
    double getFieldOfView(void) override;
    double getMaxRange(void) override;
    double getMinRange(void) override;
    pfms::RangerType getSensingMethod(void) override;

    // getData() pure virtual — implemented by Laser / Sonar

protected:
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //!< Simulator connector
    pfms::PlatformType             type_;             //!< Platform sensor is mounted on

    // Geometry — set by derived class constructors
    double           fov_;               //!< Field of view [deg]
    double           angularResolution_; //!< Angular resolution [deg/ray]
    double           maxRange_;          //!< Maximum valid range [m]
    double           minRange_;          //!< Minimum valid range [m]
    pfms::RangerType sensingMethod_;     //!< POINT or CONE

    mutable std::mutex       mutex_;      //!< Protects sensorPose_
    pfms::nav_msgs::Odometry sensorPose_; //!< Last known world-frame sensor pose
};

#endif // RANGER_H
