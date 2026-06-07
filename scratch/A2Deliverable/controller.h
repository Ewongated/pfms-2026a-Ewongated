#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cmath>

/**
 * @brief Abstract base class for all platform controllers.
 *
 * Implements the threading infrastructure, goal management, status tracking,
 * and odometry access shared between Ackerman and Skidsteer.
 *
 * Calling execute(true) launches a non-blocking background thread that drives
 * the platform through its goal list in order. Calling execute(false) signals
 * the thread to stop and blocks until it has joined. All shared state is
 * protected by mutex_.
 *
 * Derived classes must implement:
 *  - driveToGoal()              — platform-specific motion to one goal
 *  - checkOriginToDestination() — reachability check and distance/time estimate
 *  - getPlatformType()          — returns the pfms::PlatformType enum value
 */
class Controller : public ControllerInterface
{
public:
    Controller();
    virtual ~Controller();

    /**
     * @brief Starts or stops platform execution.
     *
     * start=true: If goals are set and no thread is already running, launches
     * run() in a background thread and returns immediately (non-blocking).
     * Returns false if the goal list is empty or execution is already active.
     *
     * start=false: Sets running_ to false, joins the background thread, and
     * sets platform status to IDLE. Always returns true.
     *
     * @param start true to begin executing goals, false to stop immediately.
     * @return true if the command was accepted, false otherwise.
     */
    bool execute(bool start) override;

    /**
     * @brief Returns the current platform status and the active goal index.
     * @param[out] currentGoalIndex Zero-based index of the goal currently being pursued.
     * @return IDLE, RUNNING, or STOPPING.
     */
    pfms::PlatformStatus status(unsigned int& currentGoalIndex) override;

    /**
     * @brief Replaces the goal list and resets all progress counters.
     *
     * Calls checkOriginToDestination() for each goal in sequence to verify
     * reachability. Resets distanceTravelled_, timeTravelled_, and
     * currentGoalIndex_ to zero regardless of the reachability result.
     *
     * @note Stop any running thread via execute(false) before calling this.
     * @param goals Ordered list of waypoints to pursue.
     * @return true if all goals are reachable, false if any goal cannot be reached.
     */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;

    /**
     * @brief Sets the goal-reached tolerance radius.
     * @param tolerance Acceptance radius [m].
     * @return true always.
     */
    bool setTolerance(double tolerance) override;

    /** @brief Returns the straight-line distance to the current goal [m]. */
    double distanceToGoal(void) override;

    /** @brief Returns the estimated time remaining to reach the current goal [s]. */
    double timeToGoal(void) override;

    /** @brief Returns the total distance accumulated across all completed legs [m]. */
    double distanceTravelled(void) override;

    /** @brief Returns the total time the platform has spent in motion [s]. */
    double timeTravelled(void) override;

    /** @brief Returns a copy of the most recently read platform odometry. */
    pfms::nav_msgs::Odometry getOdometry(void) override;

protected:
    /**
     * @brief Drives the platform to a single goal. Pure virtual.
     *
     * Must block until the platform is within tolerance_ of the goal or
     * running_ becomes false. Must issue a hard-stop command before returning
     * so the platform does not drift between goals.
     *
     * @param goal Target point in world coordinates.
     */
    virtual void driveToGoal(const pfms::geometry_msgs::Point& goal) = 0;

    // ── Shared state — all access must be protected by mutex_ ────────────────

    std::shared_ptr<PfmsConnector>          pfmsConnectorPtr_; //!< Simulator connector
    std::vector<pfms::geometry_msgs::Point> goals_;            //!< Ordered goal list
    unsigned int                            currentGoalIndex_; //!< Index of the goal currently being pursued
    pfms::PlatformStatus                    platformStatus_;   //!< IDLE, RUNNING, or STOPPING
    double                                  tolerance_;        //!< Goal-reached acceptance radius [m]
    double                                  distanceTravelled_;//!< Cumulative distance across completed legs [m]
    double                                  timeTravelled_;    //!< Total time spent in motion [s]
    double                                  distanceToGoal_;   //!< Straight-line distance to the current goal [m]
    double                                  timeToGoal_;       //!< Estimated time to reach the current goal [s]
    pfms::nav_msgs::Odometry                odometry_;         //!< Most recently read platform odometry

    mutable std::mutex mutex_;   //!< Protects all shared state above
    std::thread        thread_;  //!< Background driving thread
    std::atomic<bool>  running_; //!< Signals the background thread to stop when false

    /**
     * @brief Background goal-iteration loop.
     *
     * Called on the background thread by execute(true). Iterates through
     * goals_, calling driveToGoal() for each. Updates distanceTravelled_ and
     * currentGoalIndex_ after each leg, then sets platform status to IDLE.
     */
    void run();

    /**
     * @brief Reads fresh odometry from the simulator connector into odometry_.
     *
     * @warning Must NOT be called while mutex_ is held — PfmsConnector::read
     * may block, which would deadlock if the calling thread already owns mutex_.
     */
    void updateOdometry();

    /**
     * @brief Computes the Euclidean distance between two 2-D points.
     * @param a First point.
     * @param b Second point.
     * @return Distance [m].
     */
    static double euclidean(const pfms::geometry_msgs::Point& a,
                            const pfms::geometry_msgs::Point& b);

    /**
     * @brief Computes the Euclidean distance from an odometry position to a point.
     * @param odo Platform odometry (only position.x and position.y are used).
     * @param pt  Target point.
     * @return Distance [m].
     */
    static double euclidean(const pfms::nav_msgs::Odometry& odo,
                            const pfms::geometry_msgs::Point& pt);

    /**
     * @brief Wraps an angle into the range [-π, π].
     * @param angle Input angle [rad], any magnitude or sign.
     * @return Equivalent angle in [-π, π].
     */
    static double normaliseAngle(double angle);
};

#endif // CONTROLLER_H
