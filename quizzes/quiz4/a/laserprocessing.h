#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <pfms_types.h>
#include <pfmsconnector.h>
#include <math.h>

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    pfmsConnectorPtr  Pointer to the pfms connector
   */
  LaserProcessing(std::shared_ptr<PfmsConnector> pfmsConnectorPtr);


  /*! @brief Store all the centres of obstacles in the laser scan data (x,y)
    * @note The obstacles are in the local frame of the platform, so they need to be transformed to the global frame using the odometry
   *
   * @return obstacles in the laser scan data
   */
   std::vector<pfms::geometry_msgs::Point> getObstacles(void);

private:

  /*! @brief Aquire a new laserScan, attempts consecutive reads if it fails, up to 3 times
    * @note This is a blocking call, as we need the laser scan data to get
   */
  void newScan();

  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;
  pfms::sensor_msgs::LaserScan laserScan_;
  const double minRange_; //<! Minimum range to consider an obstacle, as the laser scan could get the platform itself as an obstacle if it is too close
  const double maxRange_; //<! Maximum range to consider an obstacle, as the laser scan can be noisy at long range
  unsigned long seq_;
};

#endif // LASERPROCESSING_H
