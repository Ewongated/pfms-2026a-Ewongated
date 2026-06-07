#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>

#include <functional>
#include <memory>

//Instead of Pipes now we need to use Ros communication machanism and messages
//#include <pipes.h>
#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h> //To use getYaw function from the quaternion of orientation
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "controllerinterface.h"

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    //pfms::geometry_msgs::Point location;
    geometry_msgs::msg::Point location;

    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

/**
 * \brief Shared functionality/base class for platform controllers
 *
 * Platforms need to implement:
 * - Controller::calcNewGoal (and updating GoalStats)
 * - ControllerInterface::reachGoal (and updating PlatformStats)
 * - ControllerInterface::checkOriginToDestination
 * - ControllerInterface::getPlatformType
 * - ControllerInterface::getOdometry (and updating PlatformStats.odo)
 */
class Controller: public ControllerInterface, public rclcpp::Node
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();

  //We would now have to sacrifice having a return value to have a setGoal
  //At week 10 we do not know about services (which allow us to retrun value
  //So to allow to set a goal via topic we forfit having areturn value for now
  //At week 11 you can replace this with a service
  //bool Controller::setGoal(geometry_msgs::Point goal) {
  //in A1/A2 was : bool setGoal(pfms::geometry_msgs::Point goal);
  void setGoal(const geometry_msgs::msg::Point& msg);  

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                        geometry_msgs::msg::Point goal,
                                        double& distance,
                                        double& time,
                                        geometry_msgs::msg::Pose& estimatedGoalPose) = 0;


  //pfms::PlatformType getPlatformType(void);

  bool setTolerance(double tolerance);

  double distanceTravelled(void);

  double timeInMotion(void);

  double distanceToGoal(void);

  double timeToGoal(void);

  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepted
   */
  //pfms::nav_msgs::Odometry getOdometry(void);
  geometry_msgs::msg::Pose getOdometry(void);

  ///////////////////////////////////////////////////////////////
  //! @todo
  //! TASK 3 - Callback , add declaration here
  //!


protected:

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  virtual GoalStats calcNewGoal() = 0;

  /**
   * Checks if the goal has been reached.
   *
   * Update own pose before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

  /** returns copy of goal
   * @return goal
   */
  GoalStats getGoalStats(void);


protected:
  bool goalSet_; //!< Flag indicating if a goal has been set
  double tolerance_; //!< Radius of tolerance


private: 
  geometry_msgs::msg::Pose pose_;//!< The current pose of platform
  std::mutex poseMtx_; //!< Mutex for controlling access to pose

  //stats
  GoalStats goal_;
  std::mutex goalMtx_; //!< Mutex for controlling access to goal

  
  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run

  //Instead of Pipes now we use ROS communication mechanism
  //Pipes* pipesPtr_; //!< The pipe to communicate
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub1_;

  //! @todo Create a subscription to the odometry topic

};

#endif // CONTROLLER_H
