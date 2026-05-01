#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include "missioninterface.h"
#include "rangerinterface.h"
#include "pfmsconnector.h"

/**
 * @brief Coordinates multiple platform controllers to execute a shared mission.
 *
 * ### BASIC mode
 * Passes goals to each controller and executes them concurrently.
 * status() reports percentage of total estimated distance completed.
 *
 * ### ADVANCED mode
 * As BASIC, but after each goal is reached, reads the laser on that platform
 * and checks whether the straight-line path to the next goal is obstructed.
 * If obstructed, that controller is stopped and marked abandoned; others
 * continue unaffected.
 *
 * ### Threading
 * execute(true) is non-blocking — coordination runs in a background thread.
 * Controllers are started inside runMission() (not in execute()), ensuring
 * execute() returns immediately and avoiding double-execute if the test also
 * calls controller->execute(true) directly.
 *
 * ### Multi-platform support
 * Goals are stored per controller index (not per PlatformType), so two
 * controllers of the same type (e.g. two ACKERMAN) are supported correctly.
 */
class Mission : public MissionInterface
{
public:
    /**
     * @brief Constructs a Mission for BASIC mode. Objective defaults to BASIC.
     * @param controllers Platforms to coordinate (ownership not transferred)
     */
    explicit Mission(std::vector<ControllerInterface*> controllers);

    /**
     * @brief Constructs a Mission with rangers for ADVANCED mode.
     * @param controllers Platforms to coordinate
     * @param rangers     One ranger per controller, in matching index order
     */
    Mission(std::vector<ControllerInterface*> controllers,
            std::vector<RangerInterface*> rangers);

    ~Mission();

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
    std::vector<RangerInterface*>     rangers_;     //!< Sensors for obstacle detection

    /// Goals per controller index — parallel to controllers_
    std::vector<std::vector<pfms::geometry_msgs::Point>> goalsByIndex_;

    std::vector<double> totalDistance_; //!< Pre-computed total distance per controller

    pfms::MissionObjective objective_; //!< BASIC or ADVANCED

    std::thread       missionThread_; //!< Background coordination thread
    std::atomic<bool> running_;       //!< False signals thread to stop

    mutable std::mutex mutex_;                 //!< Protects goalsByIndex_ and abandonment state
    bool               abandoned_;             //!< True if any controller stopped by obstacle
    std::vector<bool>  abandonedControllers_;  //!< Per-controller abandonment flag

    /// Background coordination loop (starts all controllers, monitors progress).
    void runMission();

    /**
     * @brief Checks whether the straight-line path to nextGoal is obstructed.
     * Reads laser for controllers_[i], checks rays within ±BEAM_HALF_WIDTH_RAD
     * of the goal bearing. Returns true if any ray hits before the goal.
     */
    bool isPathObstructed(unsigned int controllerIdx,
                          const pfms::geometry_msgs::Point& nextGoal);

    /// Chains checkOriginToDestination() across all goals to get total distance.
    double computeTotalDistance(unsigned int controllerIdx);

    static constexpr double BEAM_HALF_WIDTH_RAD = 5.0 * M_PI / 180.0; //!< ±5° obstacle window
};

#endif // MISSION_H
