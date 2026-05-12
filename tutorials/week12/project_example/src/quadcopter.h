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

private:

  /**
   * Sends a command to the quadcopter to move in a certain direction and turn.
   * @param turn_l_r Left/right turn
   * @param move_l_r Left/right movement
   * @param move_u_d Up/down movement
   * @param move_f_b Forward/backward movement
   */
  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  //! Angle required for quadcopter to have a straight shot at the goal
  double target_angle_ = 0;
  bool liftoff_;

  const double TARGET_SPEED;
  const double TARGET_HEIGHT_TOLERANCE;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_; 
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif // QUADCOPTER_H
