#ifndef FUSION_H
#define FUSION_H
#include <vector>
#include "fusioninterface.h"
#include "rangerinterface.h"

/*!
 * @brief Fusion class combines range data from multiple sensors to determine
 * cell occupancy states (FREE, OCCUPIED, UNKNOWN).
 *
 * Laser (POINT) rays mark cells FREE if passed through, OCCUPIED if endpoint
 * lands inside. Only rays that hit an object (range < maxRange) are used.
 * Sonar (CONE) sectors mark cells FREE if cone passes through, OCCUPIED if
 * the arc endpoint falls inside. Cone is treated as a true sector, not a triangle.
 * Occupancy always takes precedence over FREE.
 */
class Fusion : public FusionInterface
{
public:
    //! Default constructor - sensors can be set later
    Fusion();

    /*!
     * @brief Constructor with sensor vector
     * @param rangers Vector of pointers to RangerInterface sensors
     */
    Fusion(std::vector<RangerInterface*> rangers);

    /*!
     * @brief Sets the cells to be evaluated during fusion
     * @param cells Vector of pointers to Cell objects
     */
    void setCells(std::vector<pfms::Cell*> cells) override;

    /*!
     * @brief Acquires data from all sensors and fuses with cells.
     * Updates cell states based on sensor readings.
     * Raw data is stored for getRawRangeData().
     */
    void grabAndFuseData() override;

    /*!
     * @brief Returns raw range data from last grabAndFuseData() call.
     * Outer vector indexed by sensor, inner vector contains range readings.
     * Empty if grabAndFuseData() has not been called.
     * @return vector of vectors of range readings per sensor
     */
    std::vector<std::vector<double>> getRawRangeData() override;

    /*!
     * @brief Detects centres of objects using POINT-based sensors.
     * Clusters consecutive laser hits to find object centroids.
     * Updated each grabAndFuseData() call.
     * @return flat vector of {x,y,z} per detected object
     */
    std::vector<double> getObjectDentre() override;

    //! Returns scanning area [m^2] - not part of FusionInterface
    double getScanningArea();

private:
    std::vector<std::vector<double>> data_;    //!< Raw range data per sensor from last fusion
    std::vector<RangerInterface*>    rangers_; //!< Sensors used for fusion
    std::vector<pfms::Cell*>         cells_;   //!< Cells to fuse sensor data into

    /*!
     * @brief Check if point (px,py) lies inside a cell's bounds
     */
    bool pointInCell(pfms::Cell* cell, double px, double py);

    /*!
     * @brief Check if line segment (x1,y1)->(x2,y2) intersects a cell.
     * Uses the slab (AABB) method. Returns true if segment crosses the cell.
     */
    bool segmentIntersectsCell(pfms::Cell* cell,
                                double x1, double y1,
                                double x2, double y2);

    /*!
     * @brief Check if a point lies within a sonar cone sector.
     * Sector defined by origin, pointing direction, half-angle, and radius.
     */
    bool pointInSector(double sx, double sy, double sensorYaw,
                       double halfFov, double range,
                       double px, double py);

    /*!
     * @brief Check if a sonar sector intersects a cell (for FREE marking).
     * Tests cell corners, centre, and the two bounding rays of the cone.
     */
    bool sectorIntersectsCell(pfms::Cell* cell,
                               double sx, double sy, double sensorYaw,
                               double halfFov, double range);

    /*!
     * @brief Check if the sonar arc endpoint (at measured range) overlaps a cell.
     * Samples arc at fine angular increments. Used for OCCUPIED marking.
     */
    bool arcEndpointInCell(pfms::Cell* cell,
                            double sx, double sy, double sensorYaw,
                            double halfFov, double range);

    /*!
     * @brief Process a POINT (laser) sensor against all cells.
     * Rays that hit an object mark intermediate cells FREE and endpoint cell OCCUPIED.
     * Rays at maxRange (no detection) do not affect any cells.
     */
    void fuseLaser(const std::vector<double>& readings, RangerInterface* ranger,
                   double sx, double sy, double syaw);

    /*!
     * @brief Process a CONE (sonar) sensor against all cells.
     * Arc endpoint inside cell -> OCCUPIED. Cone passing through -> FREE.
     */
    void fuseSonar(const std::vector<double>& readings, RangerInterface* ranger,
                   double sx, double sy, double syaw);
};

#endif // FUSION_H