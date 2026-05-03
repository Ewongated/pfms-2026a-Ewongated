#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

/**
 * @brief Sonar (ultrasonic) range sensor.
 *
 * Reads a single Sonar range from the simulator. Sonar is co-located with
 * the laser on both platforms but 0.2 m above it, so it can detect objects
 * that are above the laser plane. Sensing method is CONE.
 *
 * getData() returns a single-element vector containing the sonar range, or
 * an empty vector if the reading is outside [minRange_, maxRange_].
 */
class Sonar : public Ranger
{
public:
    explicit Sonar(pfms::PlatformType type);
    ~Sonar() = default;

    /**
     * @brief Reads one sonar measurement from the simulator.
     * Updates sensorPose_. Returns empty vector if reading is invalid.
     * @return Single-element vector [range_m], or empty if invalid
     */
    std::vector<double> getData() override;

private:
    static constexpr double FOV_DEG         = 30.0;  //!< Cone half-angle [deg]
    static constexpr double MAX_RANGE_M     = 10.0;  //!< Maximum range [m]
    static constexpr double MIN_RANGE_M     = 0.1;   //!< Minimum range [m]
    static constexpr double ACKERMAN_OFFSET = 3.725; //!< [m] forward of rear axle (Audi only)
    static constexpr double HEIGHT_OFFSET   = 0.2;   //!< [m] above laser
};

#endif // SONAR_H
