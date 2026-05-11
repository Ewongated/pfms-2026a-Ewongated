#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

//We include messages types for quadcopter
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  Quadcopter();

  ~Quadcopter();

  bool reachGoal(void);


  bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal,
                                 double& distance, double& time,
                                 geometry_msgs::msg::Pose& estimatedGoalPose);

  //! @todo TASK 2 -- add control function to the interface for the service of type std_srvs::srv::SetBool
  /**
   * The callback for the service
   *
   * @param req The request message
   * @param res The response message
   */

private:

  /**
   * Calculates the angle needed for the quadcopter to reach a goal and updates the goal stats
   * @return GoalStats with location, distance and time to reach the goal
   */
  GoalStats calcNewGoal(void);

  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);


  void sendTakeOff(void);
  
  void sendLanding(void);
 
  double target_angle_ = 0; //<-- Angle required for quadcopter to have a straight shot at the goal
  bool liftoff_; //<-- indicate if the quadcopter is in the air
  bool landed_; //<-- indicate if the quadcopter is landed

  const double TARGET_SPEED;
  const double TARGET_HEIGHT_TOLERANCE;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_; 
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubLanding_; 
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif // QUADCOPTER_H
