#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include "missioninterface.h"
#include "rangerinterface.h"
#include "pfmsconnector.h"

/**
 * @brief Coordinates multiple platform controllers to execute a shared mission.
 *
 * Mission can operate in two modes (set via setMissionObjective):
 *
 * BASIC: Passes goals to each controller and executes them concurrently.
 *        Status reports % of total distance completed per platform.
 *
 * ADVANCED: As BASIC, but before commanding each platform to proceed to the
 *           next goal, reads the laser on that platform and checks whether the
 *           straight-line path is obstructed. If obstructed, that controller
 *           is stopped and marked abandoned.
 *
 * execute() is non-blocking — all coordination runs in a background thread.
 */
class Mission : public MissionInterface
{
public:
    /**
     * @brief Constructs a Mission for BASIC mode.
     * @param controllers Platforms to coordinate (ownership not transferred)
     */
    explicit Mission(std::vector<ControllerInterface*> controllers);

    /**
     * @brief Constructs a Mission with rangers for ADVANCED mode.
     * @param controllers Platforms to coordinate
     * @param rangers     One ranger per controller, in matching order
     */
    Mission(std::vector<ControllerInterface*> controllers,
            std::vector<RangerInterface*> rangers);

    ~Mission();

    // ---- MissionInterface ----

    void setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                  pfms::PlatformType platform) override;

    bool execute(bool start) override;

    std::vector<unsigned int> status() override;

    bool setMissionObjective(pfms::MissionObjective objective) override;

    std::vector<double> getDistanceTravelled() override;

    std::vector<double> getTimeMoving() override;

    bool isAbandoned() const override;

    std::vector<bool> getAbandonedControllers() const override;

private:
    std::vector<ControllerInterface*> controllers_; //!< Platforms being coordinated
    std::vector<RangerInterface*>     rangers_;      //!< Sensors for obstacle detection (ADVANCED)

    /// Goals per platform type — maps PlatformType → goal list
    std::map<pfms::PlatformType, std::vector<pfms::geometry_msgs::Point>> goalMap_;

    /// Pre-computed total mission distance per controller index
    std::vector<double> totalDistance_;

    pfms::MissionObjective objective_; //!< BASIC or ADVANCED

    std::thread       missionThread_; //!< Background coordination thread
    std::atomic<bool> running_;       //!< Set false to stop mission

    mutable std::mutex mutex_;        //!< Protects abandoned_ and abandonedControllers_
    bool               abandoned_;    //!< True if any controller was stopped due to obstacle
    std::vector<bool>  abandonedControllers_; //!< Per-controller abandonment flag

    /**
     * @brief Background mission loop.
     *
     * Starts all controllers concurrently, then monitors their progress.
     * In ADVANCED mode, also checks for obstacles before each new goal.
     */
    void runMission();

    /**
     * @brief Checks whether the path from the current platform position to
     *        the next goal is obstructed using the associated laser.
     *
     * Approximates the path as a straight line. For each laser ray whose
     * bearing points toward the goal (within a small angular window), checks
     * if the reported range is less than the distance to the goal. If so,
     * the path is considered obstructed.
     *
     * @param controllerIdx Index into controllers_ / rangers_
     * @param nextGoal      The goal to check
     * @return true if an obstacle is detected on the path
     */
    bool isPathObstructed(unsigned int controllerIdx,
                          const pfms::geometry_msgs::Point& nextGoal);

    /**
     * @brief Computes the total straight-line mission distance for a controller.
     *
     * Sums checkOriginToDestination() for each consecutive goal pair,
     * starting from the controller's current odometry.
     *
     * @param controllerIdx Index into controllers_
     * @return Total estimated distance [m]
     */
    double computeTotalDistance(unsigned int controllerIdx);
};

#endif // MISSION_H
