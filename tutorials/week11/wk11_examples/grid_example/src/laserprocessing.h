#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <math.h>

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

  /*! TASK1
   * @brief Determines the closest point to robot, in it's local reference frame
   *
   * @return location of closest point
   */
   geometry_msgs::msg::Point closestPoint();


  /*! @brief Accepts a new laserScan
   *  @param[in]    laserScan  - laserScan to be processed
   */
  void newScan(sensor_msgs::msg::LaserScan laserScan);

private:
  sensor_msgs::msg::LaserScan laserScan_;
};

#endif // LASERPROCESSING_H
