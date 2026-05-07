#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>
// #include <tf2/utils.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <Eigen/Dense>

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

  /*! TASK1
   * @brief Count number of readings belonging to objects
   *
   * @return the number of laser readings that are NOT at infinity, nan or max range
   */
  unsigned int countObjectReadings();

  /*! TASK 2
   *  @brief Count number of high intensity segments
   *  @note Segments are formed by high intensity readings.
   * A segment is a sequence (consecutive) high intensity readings that are less than 0.3m
   * away from each other, refer image in README.md
   *
   * @return the number of segments in the current laser scan
   */
  unsigned int countSegments();

  /*! TASK 3
   * @brief Return position of closest cone
   *
   * The position should be the location of the cone (which can be computed as the average laser readings belonging to the cone)
   * As the cone is circular, the laser will pick up a few readings at the location of the cone.
   * @return point - point at location of closest cone
   */
  geometry_msgs::msg::Point detectClosestCone();


  /*! TASK 4
   * @brief Return the locatin of the road centre
   *
   * Detect two cones, that are closest together, and on either side of the road
   * the road centre shoudl be the point in the middle of the two cones
   * @return point - the location of the centre of the road
   */
  geometry_msgs::msg::Point detectRoadCentre();


  /*! @brief Accepts a new laserScan
   *  @param[in]    laserScan  - laserScan to be processed
   */
  void newScan(sensor_msgs::msg::LaserScan laserScan);


private:
  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::msg::Point polarToCart(unsigned int index);

   /*! @brief Given two points (only x,y are used), returns the slope slope of the lines connecting them
    *  @param[in] p1 - first point
    *  @param[in] p2 - second point
    *  @return slope of line between the points in radians
    */
  double angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);

private:
  sensor_msgs::msg::LaserScan laserScan_;


  std::vector<std::pair<double,double>> cones_;

};

#endif // DETECTCABINET_H
