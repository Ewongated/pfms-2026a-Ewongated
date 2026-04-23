#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <cmath>

/**
 * @brief Abstract base class for all platform controllers.
 *
 * Implements the threading infrastructure, goal management, status tracking,
 * and odometry access that is shared between Ackerman and Skidsteer.
 *
 * Threading design:
 *  - execute(true) launches a background thread that drives the platform
 *    through the goal list. The call itself returns immediately (non-blocking).
 *  - execute(false) signals the thread to stop and joins it.
 *  - All shared state (goals, currentGoalIndex, distanceTravelled, status)
 *    is protected by mutex_.
 *
 * Derived classes must implement:
 *  - driveToGoal(): platform-specific control law for reaching a single goal
 *  - checkOriginToDestination(): platform-specific distance/time estimate
 *  - getPlatformType(): return the concrete platform enum value
 */
class Controller : public ControllerInterface
{
public:
    Controller();
    virtual ~Controller();

    // ---- ControllerInterface implementation ----

    bool execute(bool start) override;
    pfms::PlatformStatus status(unsigned int& currentGoalIndex) override;
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
    bool setTolerance(double tolerance) override;
    double distanceToGoal(void) override;
    double timeToGoal(void) override;
    double distanceTravelled(void) override;
    double timeTravelled(void) override;
    pfms::nav_msgs::Odometry getOdometry(void) override;

    // getPlatformType and checkOriginToDestination remain pure virtual

protected:
    // ---- Platform-specific hooks (implemented in derived classes) ----

    /**
     * @brief Drives the platform to a single goal.
     *
     * Called repeatedly by the run() loop for each goal in sequence.
     * Implementations should block until the goal is reached (within
     * tolerance_) or until running_ becomes false.
     *
     * @param goal The target point
     */
    virtual void driveToGoal(const pfms::geometry_msgs::Point& goal) = 0;

    // ---- Shared state (access protected by mutex_) ----

    std::shared_ptr<PfmsConnector>        pfmsConnectorPtr_; //!< Simulator connector
    std::vector<pfms::geometry_msgs::Point> goals_;          //!< Goal list (in order)
    unsigned int                          currentGoalIndex_; //!< Index of goal being pursued
    pfms::PlatformStatus                  platformStatus_;   //!< IDLE / RUNNING / STOPPING
    double                                tolerance_;        //!< Goal-reached threshold [m]
    double                                distanceTravelled_;//!< Cumulative distance [m]
    double                                timeTravelled_;    //!< Cumulative time in motion [s]
    double                                distanceToGoal_;   //!< Distance to current goal [m]
    double                                timeToGoal_;       //!< Time to current goal [s]
    pfms::nav_msgs::Odometry              odometry_;         //!< Latest platform odometry

    mutable std::mutex mutex_; //!< Protects all shared state above

    // ---- Threading ----

    std::thread          thread_;   //!< Background driving thread
    std::atomic<bool>    running_;  //!< Set false to signal thread to stop

    /**
     * @brief Main driving loop, executed in the background thread.
     *
     * Iterates through goals_, calls driveToGoal() for each, updates
     * distanceTravelled_ and currentGoalIndex_, then sets status to IDLE.
     */
    void run();

    /**
     * @brief Reads fresh odometry from the simulator and stores it in odometry_.
     *
     * Thread-safe: acquires mutex_ before writing.
     */
    void updateOdometry();

    /**
     * @brief Computes Euclidean distance between two points (ignoring z).
     */
    static double euclidean(const pfms::geometry_msgs::Point& a,
                            const pfms::geometry_msgs::Point& b);

    /**
     * @brief Computes Euclidean distance from an odometry position to a point.
     */
    static double euclidean(const pfms::nav_msgs::Odometry& odo,
                            const pfms::geometry_msgs::Point& pt);

    /**
     * @brief Normalises an angle to [-pi, pi].
     */
    static double normaliseAngle(double angle);
};

#endif // CONTROLLER_H