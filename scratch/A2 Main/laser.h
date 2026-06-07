#ifndef LASER_H
#define LASER_H

#include "ranger.h"

/**
 * @brief Laser rangefinder sensor.
 *
 * Reads a LaserScan from the simulator. Sensor geometry for both platforms:
 * - Audi R8: co-located with sonar, 3.725 m forward of rear-axle centre.
 * - Husky:   co-located with sonar at platform centre.
 *
 * FOV = 270°, angular resolution = 0.25°/ray, sensing method = POINT.
 * getSensorPose() returns the last computed world-frame sensor pose.
 */
class Laser : public Ranger
{
public:
    explicit Laser(pfms::PlatformType type);
    ~Laser() = default;

    /**
     * @brief Reads one laser scan from the simulator.
     * Updates sensorPose_ from the accompanying platform odometry.
     * Discards readings outside [minRange_, maxRange_].
     * @return Vector of valid range readings [m]
     */
    std::vector<double> getData() override;

private:
    // Laser geometry constants
    static constexpr double FOV_DEG         = 270.0; //!< Field of view [deg]
    static constexpr double ANG_RES_DEG     = 0.25;  //!< Angular resolution [deg/ray]
    static constexpr double MAX_RANGE_M     = 30.0;  //!< Maximum range [m]
    static constexpr double MIN_RANGE_M     = 0.1;   //!< Minimum range [m]

    // Sensor offset from platform origin (Audi R8 only — Husky is at centre)
    static constexpr double ACKERMAN_OFFSET = 3.725; //!< [m] forward of rear axle
};

#endif // LASER_H
