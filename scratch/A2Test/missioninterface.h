#ifndef MISSIONINTERFACE_H
#define MISSIONINTERFACE_H

#include <vector>
#include "controllerinterface.h"

/**
 * @brief Specifies the required interface for your Mission class
 * must inherit from it. <b> You MUST NOT edit this file </b>.
 * 
 */

/*!
 *  \brief     Mision Interface Class
 *  \details
 *  Specifies the required interface for your Mission
 *  must inherit from it. <b> You MUST NOT edit this file </b>.
 *  \author    Alen Alempijevic
 *  \version   1.02-0
 *  \date      2026-04-02
 *  \pre       none
 *  \bug       none reported as of 2026-04-07
 *  \warning   students MUST NOT change this class 
 */



class MissionInterface
{
public:
    MissionInterface(){};

    /**
     * @brief Accepts the container of goals, any previous goals should be cleared. 
     *
     * @param goals The goals are supplied in the order they should be pursued.
     * @param platform The platform type for which the goals are being set. This is needed as the same mission class can be used for multiple platforms.
     */
    virtual void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform) = 0;

    /**
     * @brief Execute or stop the mission, non blocking call
     * @param start true to start mission execution, false to stop mission execution
     * @return bool indicating mission can be completed (false if mission not possible)
     */
    virtual bool execute(bool start) = 0;

    /**
    Retrurns mission completion status (indicating percentage of completion of task) by each platform @sa setGoals
    @return vector with each element of vector corresponding to a platform. The value is percent of completed distance of entire mission for the corresponding platform value between 0-100.
    */
    virtual std::vector<unsigned int> status(void) = 0;


    /**
     * @brief Set mission objective
     * @param objective The mission objective to set, either BASIC or ADVANCED
      * BASIC: execute goals in order supplied, no obstacle detection
      * ADVANCED: execute goals in order supplied, but if obstacle detected at next goal abandon mission 
      * @return bool indicating if mission objective was set successfully
     */
    virtual bool setMissionObjective(pfms::MissionObjective objective) = 0;

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the total distance travelled by the corresponding platform
     * from the time of starting the program.
     *
     * @return std::vector<double> - each element is distance travelled for each platform [m]
     */
    virtual std::vector<double> getDistanceTravelled() = 0;

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the time the corresponding platfore has been moving
     * from the time the program started. Moving means the platform was not stationary.
     *
     * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
     *
     */
    virtual std::vector<double> getTimeMoving() = 0;

    /**
     * @brief Check if mission was abandoned due to obstacle detection
     * @return true if mission was abandoned because of collision, false otherwise
     */
    virtual bool isAbandoned() const = 0;
    
    /**
     * @brief Get which controllers abandoned the mission due to obstacle detection
     * @return vector of booleans indicating which controllers were stopped (true = stopped due to collision)
     */
    virtual std::vector<bool> getAbandonedControllers() const = 0;

};

#endif // MISSIONINTERFACE_H
