#ifndef LASER_H
#define LASER_H

#include "ranger.h"

/**
 * @brief Laser rangefinder sensor.
 *
 * Reads a 270° planar laser scan from the simulator via PfmsConnector.
 * Sensor mounting positions (relative to the platform origin):
 *  - Audi R8:  3.725 m forward of the rear-axle centre along the platform heading.
 *  - Husky:    at the platform centre (no offset).
 *
 * On each call to getData(), the sensor pose is recomputed from the current
 * platform odometry so that Mission::isPathObstructed() always uses an
 * up-to-date world-frame position. Scan geometry fields (angularResolution_,
 * fov_, maxRange_, minRange_, angleMin_) are also updated from the raw scan
 * message so they reflect the actual simulator configuration.
 *
 * Sensing method: POINT (each ray returns a single range value).
 */
class Laser : public Ranger
{
public:
    /**
     * @brief Constructs a Laser for the given platform and sets sensor geometry.
     * @param type Platform the laser is mounted on (ACKERMAN or SKIDSTEER).
     */
    explicit Laser(pfms::PlatformType type);
    ~Laser() = default;

    /**
     * @brief Reads one laser scan from the simulator.
     *
     * Updates sensorPose_ and all geometry fields from the raw scan message.
     * Range values outside [minRange_, maxRange_] are replaced with 0.0 so
     * callers can identify invalid rays by checking for zero.
     *
     * @return Vector of range readings [m]. Invalid rays are 0.0.
     */
    std::vector<double> getData() override;

private:
    // ── Default geometry constants (overwritten from scan message in getData()) ──
    static constexpr double FOV_DEG         = 270.0; //!< Field of view [deg]
    static constexpr double ANG_RES_DEG     = 0.25;  //!< Angular resolution [deg/ray]
    static constexpr double MAX_RANGE_M     = 30.0;  //!< Maximum range [m]
    static constexpr double MIN_RANGE_M     = 0.1;   //!< Minimum range [m]

    // ── Sensor offset ────────────────────────────────────────────────────────
    static constexpr double ACKERMAN_OFFSET = 3.725; //!< Distance forward of rear axle (Audi R8 only) [m]
};

#endif // LASER_H
