#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>

#include <functional>
#include <memory>

//Instead of Pipes now we need to use Ros communication machanism and messages
//#include <pipes.h>
#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h> //To use getYaw function from the quaternion of orientation
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"


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
class Controller: public rclcpp::Node
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();


  /** 
   * Connected to callback function to set the goal
   * @param msg The goal to set
   */
  void setGoal(const geometry_msgs::msg::Point& msg);  

  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepted
   */
  //pfms::nav_msgs::Odometry getOdometry(void);
  geometry_msgs::msg::Pose getOdometry(void);

  /**
   * Updates the internal pose
   *
   * Calback function for the odometry subscriber
   */
  void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own pose before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

  geometry_msgs::msg::Pose pose_;//!< The current pose of platform
  std::mutex mtx_;//<! Mutex to protect the pose

  geometry_msgs::msg::Point goal_;//!< The current goal of the platform
  bool goalSet_;//!< Whether a goal has been set
  
  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int cmd_pipe_seq_; //!<The sequence number of the command

  //Instead of Pipes now we use ROS communication mechanism
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_; //<! Subscription to odometry
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub2_;//<! Subscription to goal

};

#endif // CONTROLLER_H
