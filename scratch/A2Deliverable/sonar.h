#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

/**
 * @brief Sonar (ultrasonic cone) range sensor.
 *
 * Reads a single sonar range from the simulator via PfmsConnector. The sonar
 * is co-located with the laser on both platforms but mounted 0.2 m higher,
 * enabling detection of objects above the laser's horizontal plane.
 *
 * ### Mounting positions
 *  - Audi R8:  3.725 m forward of the rear-axle centre, 0.2 m above the laser.
 *  - Husky:    at the platform centre, 0.2 m above the laser.
 *
 * Sensing method: CONE (single beam, returns one range per call).
 * getData() returns a single-element vector, or an empty vector if the
 * measurement falls outside [minRange_, maxRange_].
 */
class Sonar : public Ranger
{
public:
    /**
     * @brief Constructs a Sonar for the given platform and sets sensor geometry.
     * @param type Platform the sonar is mounted on (ACKERMAN or SKIDSTEER).
     */
    explicit Sonar(pfms::PlatformType type);
    ~Sonar() = default;

    /**
     * @brief Reads one sonar measurement from the simulator.
     *
     * Updates sensorPose_ with the current world-frame sensor position,
     * including the height offset. Returns an empty vector if the reading
     * is outside [minRange_, maxRange_].
     *
     * @return Single-element vector containing the range [m], or empty if invalid.
     */
    std::vector<double> getData() override;

private:
    static constexpr double FOV_DEG         = 30.0;  //!< Cone full angle [deg]
    static constexpr double MAX_RANGE_M     = 10.0;  //!< Maximum valid range [m]
    static constexpr double MIN_RANGE_M     = 0.1;   //!< Minimum valid range [m]
    static constexpr double ACKERMAN_OFFSET = 3.725; //!< Forward offset from rear axle (Audi R8 only) [m]
    static constexpr double HEIGHT_OFFSET   = 0.2;   //!< Height above laser plane [m]
};

#endif // SONAR_H
