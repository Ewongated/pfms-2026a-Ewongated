#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include "missioninterface.h"
#include "rangerinterface.h"
#include "pfmsconnector.h"

class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @brief The constrcutor must set the MissionObjective to BASIC
    @param controllers to be passed to mission to coordinate
    @sa ControllerInterface and @sa MissionInterface for more information
    */
    Mission(std::vector<ControllerInterface*> controllers);

    /** 
    @brief Constructor for Mission with rangers, used for ADVANCED mission objective
    @param controllers to be passed to mission to coordinate
    @param rangers to be used for collision detection in advanced mode
    @sa ControllerInterface, @sa RangerInterface and @sa MissionInterface for more information
    */
    Mission(std::vector<ControllerInterface*> controllers, std::vector<RangerInterface*> rangers);

    ~Mission();

    

private:
  
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<RangerInterface*> rangers_; //!< A private copy of RangerInterfaces @sa RangerInterface
};

#endif // RANGERFUSION_H
