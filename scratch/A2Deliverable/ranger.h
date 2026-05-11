#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include "pfms_types.h"
#include "pfmsconnector.h"
#include <cmath>
#include <mutex>

/**
 * @brief Abstract base class for all ranger (distance sensor) types.
 *
 * Holds state common to Laser and Sonar: a PfmsConnector, the platform type
 * the sensor is mounted on, sensor geometry parameters, and the most recently
 * computed world-frame sensor pose.
 *
 * Derived classes (Laser, Sonar) must implement getData() and initialise all
 * geometry fields (fov_, angularResolution_, maxRange_, minRange_,
 * angleMin_, sensingMethod_) in their constructors.
 *
 * sensorPose_ is updated by getData() on every call so that
 * Mission::isPathObstructed() always uses a current sensor position.
 */
class Ranger : public RangerInterface
{
public:
    /**
     * @brief Constructs a Ranger and opens a PfmsConnector for the given platform.
     *
     * All geometry fields are zero-initialised; derived constructors set them
     * to the correct values for their sensor type.
     *
     * @param type Platform the sensor is physically mounted on.
     */
    explicit Ranger(pfms::PlatformType type);

    /** @brief Returns the angular resolution of the sensor [deg/ray]. */
    double getAngularResolution(void) override;

    /** @brief Returns the most recently computed world-frame sensor pose. */
    pfms::nav_msgs::Odometry getSensorPose(void) override;

    /** @brief Returns the sensor field of view [deg]. */
    double getFieldOfView(void) override;

    /** @brief Returns the maximum valid measurement range [m]. */
    double getMaxRange(void) override;

    /** @brief Returns the minimum valid measurement range [m]. */
    double getMinRange(void) override;

    /** @brief Returns the sensing method (POINT for Laser, CONE for Sonar). */
    pfms::RangerType getSensingMethod(void) override;

    /**
     * @brief Returns the start angle of the scan in the sensor frame [rad].
     *
     * For a symmetric 270° laser the start angle is -135° (-3π/4 rad).
     * Updated from the raw scan message by Laser::getData() on every call.
     */
    double getAngleMin(void);

    // getData() is pure virtual — implemented by Laser and Sonar.

protected:
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //!< Simulator connector
    pfms::PlatformType             type_;             //!< Platform the sensor is mounted on

    // ── Geometry — initialised by derived constructors ───────────────────────
    double           fov_;               //!< Field of view [deg]
    double           angularResolution_; //!< Angular resolution [deg/ray]
    double           maxRange_;          //!< Maximum valid range [m]
    double           minRange_;          //!< Minimum valid range [m]
    double           angleMin_;          //!< Start angle of scan in sensor frame [rad]
    pfms::RangerType sensingMethod_;     //!< POINT (Laser) or CONE (Sonar)

    mutable std::mutex       mutex_;      //!< Protects sensorPose_ and geometry fields updated by getData()
    pfms::nav_msgs::Odometry sensorPose_; //!< Most recently computed world-frame sensor pose
};

#endif // RANGER_H
