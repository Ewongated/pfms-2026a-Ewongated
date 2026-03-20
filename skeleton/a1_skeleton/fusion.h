#ifndef FUSION_H
#define FUSION_H
#include <vector>
#include "fusioninterface.h"
#include "rangerinterface.h"

/*!
 * @brief Fusion class combines range data from multiple sensors to determine
 * cell occupancy states (FREE, OCCUPIED, UNKNOWN).
 *
 * For each sensor ray/cone that passes through a cell:
 *   - Endpoint inside cell  -> OCCUPIED
 *   - Endpoint past cell    -> FREE
 *   - Endpoint before cell  -> no change (UNKNOWN)
 * Occupancy always takes precedence over FREE.
 */
class Fusion : public FusionInterface
{
public:
    //! Default constructor
    Fusion();

    /*!
     * @brief Constructor with sensor vector
     * @param rangers Vector of pointers to RangerInterface sensors
     */
    Fusion(std::vector<RangerInterface*> rangers);

    void setCells(std::vector<pfms::Cell*> cells) override;
    void grabAndFuseData() override;
    std::vector<std::vector<double>> getRawRangeData() override;
    std::vector<double> getObjectDentre() override;

    //! Returns total scanning area [m^2] across all sensors (not in FusionInterface)
    double getScanningArea();

private:
    std::vector<std::vector<double>> data_;    //!< Raw range data per sensor
    std::vector<RangerInterface*>    rangers_; //!< Sensors used for fusion
    std::vector<pfms::Cell*>         cells_;   //!< Cells to fuse data into

    //! Process POINT (laser) sensor readings against all cells
    void fuseLaser(const std::vector<double>& readings, RangerInterface* ranger,
                   double sx, double sy, double syaw);

    //! Process CONE (sonar) sensor reading against all cells
    void fuseSonar(const std::vector<double>& readings, RangerInterface* ranger,
                   double sx, double sy, double syaw);

    //! Returns true if (px,py) is inside cell bounds
    bool pointInCell(pfms::Cell* cell, double px, double py);

    /*!
     * @brief Slab AABB intersection test for a line segment vs cell.
     * @param tEntry parametric entry point into cell [0,1]
     * @param tExit  parametric exit point from cell [0,1]
     * @return true if segment crosses the cell rectangle
     */
    bool segmentIntersectsCell(pfms::Cell* cell,
                                double x1, double y1,
                                double x2, double y2,
                                double& tEntry, double& tExit);

    //! Returns true if point is within sonar cone sector
    bool pointInSector(double sx, double sy, double sensorYaw,
                       double halfFov, double range,
                       double px, double py);

    //! Returns true if sonar sector intersects cell (for cone pass-through check)
    bool sectorIntersectsCell(pfms::Cell* cell,
                               double sx, double sy, double sensorYaw,
                               double halfFov, double range);

    //! Returns true if sonar arc endpoint at measured range lands inside cell
    bool arcEndpointInCell(pfms::Cell* cell,
                            double sx, double sy, double sensorYaw,
                            double halfFov, double range);

    //! Returns true if sonar arc endpoint is past the cell (FREE condition)
    bool arcEndpointPastCell(pfms::Cell* cell,
                              double sx, double sy, double sensorYaw,
                              double halfFov, double range);
};

#endif // FUSION_H