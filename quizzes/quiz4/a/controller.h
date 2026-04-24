#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>

//! This structure can be used to store information about the current goals and the goals to be pursued
//! you can add more attributes if you think they are useful
struct GoalStats {
    //! location of goal
    pfms::geometry_msgs::Point location;
    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

/**
 * \brief Shared functionality/base class for platform controllers
 */
class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();

   //See controllerinterface.h for more information

   /** 
    * Run controller in reaching goals - non blocking call, delegated to be implemented by the platform specific controller
    */
    bool execute(bool start) = 0;
  
    /**
      * Returns platform status (indicating if it is executing a series of goals (RUNNING) or idle (IDLE) - waiting for goals)
      * @return platform status
      */
   virtual pfms::PlatformStatus status(unsigned int& currentGoalIndex);
 
    /**
      * Setter for goals, delegated to be implemented by the platform specific controller
      */
   virtual bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) = 0;
 

  /** 
   * Checks whether the platform can travel between origin and destination
   * Delegated to be implemented by the platform specific controller
   */
   virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                    double& distance, double& time,  pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;
 
    /**
      * Getter for platform type
      * @return PlatformType
      */                                    
   pfms::PlatformType getPlatformType(void);
 
    /**
      * Getter for the current odometry of the platform
      * @return current odometry of the platform
      */
   pfms::nav_msgs::Odometry getOdometry(void);

    /**
      * Getter for the current obstacles
      * @return current obstacles ( obstacles are in the global frame)
      * @note The obstacles are in the global frame
      */
   std::vector<pfms::geometry_msgs::Point> getObstacles(void) =0;


protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own odometry before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

    /**
      * Thread-safe getter for the number of goals
      * @return number of goals
      */
   unsigned int getGoalsSize(void);

    /**
      * Thread-safe getter for a specific goal
      * @param index The index of the goal to retrieve
      * @param goal Reference to store the goal data
      * @return true if goal exists at index, false otherwise
      */
   bool getGoal(unsigned int index, GoalStats& goal);


  ///////////////////////////////////////////////////////////////
  //! @todo
  //! Consider which variables should be in the base class vs derived class.
  //! Variables used only by one platform type should be in that derived class.
  //! Variables used by Controller methods should remain in the base class.

  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //!< Pointer to the pfms connector for communication with the platform and simulator

  std::vector<GoalStats> goals_; //!< The goals to be reached, we store all GoalStats in this vector

  pfms::PlatformType type_; //!< The platform type
  pfms::PlatformStatus status_; //!< The current status of the platform

  double tolerance_; //!< Radius of tolerance

  unsigned int currentGoalIdx_; //Current goal ID

  std::mutex mtxGoals_; //!< Mutex for goals


};

#endif // CONTROLLER_H
