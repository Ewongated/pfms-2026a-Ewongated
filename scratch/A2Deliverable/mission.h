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
 * Goals are dispatched to each controller and all controllers execute
 * concurrently. status() reports the percentage of total estimated distance
 * completed per controller.
 *
 * ### ADVANCED mode
 * As BASIC, but after each goal is reached the laser on that platform is read
 * and the straight-line path to the next goal is checked for obstacles. If an
 * obstacle is detected, that controller is stopped via execute(false) and
 * marked as abandoned. Other controllers continue unaffected.
 *
 * ### Threading
 * execute(true) is non-blocking — all coordination runs inside a background
 * thread (runMission()). Controllers are started from within runMission(), not
 * from execute(), so execute() always returns immediately.
 *
 * ### Multi-platform support
 * Goals are stored per controller index (not per PlatformType), so two
 * controllers of the same type (e.g. two ACKERMAN platforms) each receive
 * their own independent goal list when setGoals() is called twice.
 */
class Mission : public MissionInterface
{
public:
    /**
     * @brief Constructs a Mission for BASIC mode.
     *
     * Mission objective defaults to BASIC. Rangers are not required.
     * @param controllers Platforms to coordinate (ownership is not transferred).
     */
    explicit Mission(std::vector<ControllerInterface*> controllers);

    /**
     * @brief Constructs a Mission with rangers for ADVANCED mode.
     * @param controllers Platforms to coordinate.
     * @param rangers     One ranger per controller, in matching index order.
     */
    Mission(std::vector<ControllerInterface*> controllers,
            std::vector<RangerInterface*> rangers);

    /** @brief Destructor — stops all controllers and joins the mission thread. */
    ~Mission();

    /**
     * @brief Associates goals with the first unassigned controller matching platform.
     *
     * Uses index-based storage so that two controllers of the same PlatformType
     * each receive their own goal list when setGoals() is called twice with the
     * same platform type. Previous goals for that slot are cleared.
     *
     * @param goals    Ordered list of waypoints.
     * @param platform Platform type to match against the controller list.
     */
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                  pfms::PlatformType platform) override;

    /**
     * @brief Starts or stops mission execution.
     *
     * start=true: Dispatches goals to controllers, pre-computes total distances,
     * resets abandonment state, and launches runMission() in a background thread.
     * Returns immediately (non-blocking).
     *
     * start=false: Signals the mission thread to stop, stops all controllers
     * via execute(false), and joins the thread. Resets goalsAssigned_ so that
     * setGoals() can be called again for a fresh mission.
     *
     * @param start true to begin execution, false to stop.
     * @return true always.
     */
    bool execute(bool start) override;

    /**
     * @brief Returns per-controller mission completion percentage [0–100].
     *
     * Returns 100 when a controller is IDLE and its goal index has passed the
     * end of its goal list. Returns at most 99 while a controller is still
     * RUNNING to avoid premature reporting of completion.
     *
     * @return Vector with one entry per controller.
     */
    std::vector<unsigned int> status() override;

    /**
     * @brief Sets the mission objective (BASIC or ADVANCED).
     * @param objective BASIC for goal-following only; ADVANCED enables obstacle detection.
     * @return true always.
     */
    bool setMissionObjective(pfms::MissionObjective objective) override;

    /**
     * @brief Returns the total distance travelled by each controller since execution began.
     * @return Vector of distances [m], one per controller.
     */
    std::vector<double> getDistanceTravelled() override;

    /**
     * @brief Returns the total time each controller has spent in motion.
     * @return Vector of times [s], one per controller.
     */
    std::vector<double> getTimeMoving() override;

    /**
     * @brief Returns true if any controller was stopped due to obstacle detection.
     * @return true if at least one controller was abandoned, false otherwise.
     */
    bool isAbandoned() const override;

    /**
     * @brief Returns per-controller abandonment flags.
     * @return Vector of booleans; true indicates the controller was stopped by an obstacle.
     */
    std::vector<bool> getAbandonedControllers() const override;

private:
    std::vector<ControllerInterface*> controllers_; //!< Platforms being coordinated
    std::vector<RangerInterface*>     rangers_;     //!< Sensors used for obstacle detection (ADVANCED mode)

    std::vector<std::vector<pfms::geometry_msgs::Point>> goalsByIndex_; //!< Goal list per controller index
    std::vector<bool>   goalsAssigned_;   //!< Tracks which controller slots have been assigned goals
    std::vector<double> totalDistance_;   //!< Pre-computed total mission distance per controller [m]

    pfms::MissionObjective objective_; //!< BASIC or ADVANCED

    std::thread       missionThread_; //!< Background coordination thread
    std::atomic<bool> running_;       //!< Signals the mission thread to stop when false

    mutable std::mutex mutex_;                //!< Protects goalsByIndex_, goalsAssigned_, and abandonment state
    bool               abandoned_;            //!< True if any controller was stopped by an obstacle
    std::vector<bool>  abandonedControllers_; //!< Per-controller abandonment flag

    /**
     * @brief Background mission coordination loop.
     *
     * Starts all controllers concurrently then polls their status every 100 ms.
     * In BASIC mode, waits until all controllers reach IDLE.
     * In ADVANCED mode, additionally checks for obstacles whenever a controller
     * advances to a new goal, stopping and marking the controller if obstructed.
     */
    void runMission();

    /**
     * @brief Returns true if the straight-line path to nextGoal is obstructed.
     *
     * Reads the laser for controllers_[controllerIdx], computes the bearing from
     * the sensor's current world-frame position to nextGoal, and examines every
     * ray within ±BEAM_HALF_WIDTH_RAD of that bearing. Returns true if any such
     * ray has a range more than 1 m shorter than the goal distance.
     *
     * @param controllerIdx Index of the controller (and its matching ranger).
     * @param nextGoal      The upcoming goal point to check.
     * @return true if an obstacle is detected, false if the path is clear.
     */
    bool isPathObstructed(unsigned int controllerIdx,
                          const pfms::geometry_msgs::Point& nextGoal);

    /**
     * @brief Chains checkOriginToDestination() across all goals to compute total distance.
     *
     * Uses the controller's current odometry as the start, then advances the
     * estimated pose after each leg to correctly account for heading changes.
     *
     * @param controllerIdx Index of the controller whose goal list to use.
     * @return Total estimated distance [m], or 0.0 if the goal list is empty.
     */
    double computeTotalDistance(unsigned int controllerIdx);

    static constexpr double BEAM_HALF_WIDTH_RAD = 5.0 * M_PI / 180.0; //!< Half-width of obstacle detection window [rad] (±5°)
};

#endif // MISSION_H
