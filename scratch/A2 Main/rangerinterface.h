#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>
#include "pfms_types.h"

/*!
 *  \brief     Ranger Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived sensor classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Alen Alempijevic
 *  \version   1.01-1
 *  \date      2022-07-04
 *  \pre       none
 *  \bug       none reported as of 2022-07-04
 *  \warning   students MUST NOT change this class (the header file)
 */

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class

/**
 * @brief Specifies the functionality for the Ranger Class, your Ranger
 * class must inherit from it. <b> You MUST NOT edit this file </b>.
 *
 */
class RangerInterface
{
public:
  RangerInterface(){};

  //Generates raw data for sensor
  virtual std::vector<double> getData() = 0;

  /**
  Getter for Angular resolution
  @return angular resolution [deg]
  */
  virtual double getAngularResolution(void) = 0;

  /**
  Getter for sensor pose
  @return sensor pose (x,y and yaw) in fms::nav_msgs::Odometry
  */
  virtual pfms::nav_msgs::Odometry getSensorPose(void) = 0;

  /**
  Getter for field of view, for POINT based sensors FOV is zero
  @return field of view [deg]
  */
  virtual double getFieldOfView(void) = 0;

  /**
  Getter for maximum range
  @return maximum rage [m]
  */
  virtual double getMaxRange(void) = 0;

  /**
  Getter for mimimum range
  @return minimum rage [m]
  */
  virtual double getMinRange(void) = 0;

  /**
  Getter for ranger type
  @return RangerType (aka Sensging Method)
  */
  virtual pfms::RangerType getSensingMethod(void) = 0;

};

#endif // RANGERFUSIONINTERFACE_H
