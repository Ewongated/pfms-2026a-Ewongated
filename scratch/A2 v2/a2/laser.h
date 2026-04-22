#ifndef LASER_H
#define LASER_H

#include "ranger.h"

/**
 * @brief Laser range sensor implementation.
 *
 * Provides a 180-degree POINT-type laser scan with 640 rays.
 * Connects to the laser sensor on the specified platform via PfmsConnector.
 *
 * For ACKERMAN platforms, the sensor is located 3.725m forward of the
 * rear-axle centre. For SKIDSTEER platforms, the sensor is at the centre.
 */
class Laser : public Ranger
{
public:
    /**
     * @brief Constructs a Laser sensor for the given platform.
     * @param type Platform type (ACKERMAN or SKIDSTEER)
     */
    explicit Laser(pfms::PlatformType type);

    ~Laser() = default;

    /**
     * @brief Reads a full laser scan from the simulator.
     *
     * Blocks briefly (~200ms) to receive a fresh scan. Updates sensorPose_
     * from current platform odometry and updates angular parameters from
     * actual scan metadata.
     *
     * @return Vector of range values [m], one per ray
     */
    std::vector<double> getData() override;

    /**
     * @brief Sets the angular resolution of the laser.
     * @param angularResolution Desired resolution [deg], must be > 0
     * @return true if set successfully, false if value invalid
     */
    bool setAngularResolution(double angularResolution);
};

#endif // LASER_H
