#ifndef Skidsteer_H
#define Skidsteer_H

#include "controller.h"
#include "laserprocessing.h"

//! UAV drone platform controller
class Skidsteer: public Controller
{
public:
  //Default constructor - should set all the attributes to a default value for plaform and enable use of the class.
  Skidsteer();

  ~Skidsteer();

  /**
   * @brief Run the Skidsteer
   * @todo Implement the execute function, which should start the control loop for the Skidsteer to reach the goals when start is set to true, and stop the control loop when start is set to false
   * @note As per the controller interface, this is a non-blocking call, will kick of the control loop 
   */
  bool execute(bool start);

  /**
   * @brief Set the goals for the Skidsteer
   * @todo Implement the setGoals function, which should set the goals for the Skidsteer and return true if all goals are reachable, false otherwise
   * You can use the checkOriginToDestination function to check if each goal is reachable and to populate the distance and time to goal
   * @param goals The goals to be reached
   * @return true if all goals are reachable, false otherwise
   */
  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

  /**
   * @brief Check if the Skidsteer can reach the destination from the origin
   * @param origin The origin pose, specified as odometry for the platform  
   * @param destination The destination point for the platform
   * @param distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
   * @param time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
   * @param estimatedGoalPose The estimated goal pose when reaching goal
   * @return bool indicating the platform can reach the destination from origin supplied
   */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time,
                                 pfms::nav_msgs::Odometry& estimatedGoalPose);

  ///////////////////////////////////////////////////////////////
  //! @todo
  //! Consider where these functions should be implemented:
  //! Should they be in the base Controller class or in the
  //! platform-specific derived class (Skidsteer)?
  //! Think about:
  //! - Which class owns the data being accessed?
  //! - Is the implementation platform-specific or generic?
  //! - Does it make sense for all platform types?
  //! Implement the functions below as you see fit, you can implement them in either the base Controller class or in the Skidsteer class.

  /**
   * @brief Getter for distance to be travelled to reach current goal
   * @return distance to be travelled to reach current goal [m]
   */
  double distanceToGoal(void);

  /**
   * @brief Getter for time to reach current goal
   * @return time to travel to current goal [s]
   */
  double timeToGoal(void);

  /**
   * @brief Setter for tolerance
   * @param tolerance The tolerance [m] when reaching the goal
   * @return true if tolerance is valid (non-negative), false otherwise
   */
  bool setTolerance(double tolerance);

  /**
   * @brief Getter for distance travelled to reach current goal
   * @return distance travelled to reach current goal [m]
   */
  double distanceTravelled(void);

  /**
   * @brief Getter for time travelled to reach current goal
   * @return time travelled to current goal [s]
   */
  double timeTravelled(void);

  std::vector<pfms::geometry_msgs::Point> getObstacles(void);

private:

  void reachGoal(void);

  /** 
   * Sends the command to the platform, delegated to be implemented by the platform specific controller
   * @param turn_l_r The turning command for the platform, positive for left turn, negative for right turn
   * @param move_f_b The moving command for the platform, positive for forward, negative for backward
   */
  void sendCmd(double turn_l_r, double move_f_b);

  //! Angle required for Skidsteer to have a straight shot at the goal
  double target_angle_ = 0;
  double dist_;

  std::shared_ptr<LaserProcessing> laserProcessingPtr_;

  const double MAX_LINEAR_SPEED;
  const double MAX_ANGULAR_SPEED;

  ///////////////////////////////////////////////////////////////
  //! @todo
  //! Consider which variables should be in the base class vs derived class.
  //! Variables used only by one platform type should be in that derived class.
  //! Variables used by Controller methods should remain in the base class.

  double distanceTravelled_; //!< Total distance travelled for this program run
  double timeInMotion_; //!< Total time spent travelling for this program run
  long unsigned int seq_; //!< The sequence number of the command
  bool goalSet_; //!< If the goal is set
  std::atomic<bool> running_; //!< Flag to indicate if the thread should be running
  std::thread* thread_; //!< Thread for the control loop
  std::condition_variable cv_; //!< Condition variable for control

};

#endif // Skidsteer_H
