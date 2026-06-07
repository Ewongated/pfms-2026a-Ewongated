#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

/**
 * @brief Sonar (ultrasonic) range sensor implementation.
 *
 * Provides a single CONE-type range reading. The sonar is mounted 0.2m
 * above the laser on both platforms, and uses the same forward offset
 * (3.725m for ACKERMAN, 0.0m for SKIDSTEER).
 */
class Sonar : public Ranger
{
public:
    /**
     * @brief Constructs a Sonar sensor for the given platform.
     * @param type Platform type (ACKERMAN or SKIDSTEER)
     */
    explicit Sonar(pfms::PlatformType type);

    ~Sonar() = default;

    /**
     * @brief Reads one sonar measurement from the simulator.
     *
     * Flushes a stale odometry reading, reads fresh sonar and odometry data,
     * then updates sensorPose_.
     *
     * @return Vector of size 1 containing the range [m]
     */
    std::vector<double> getData() override;
};

#endif // SONAR_H
