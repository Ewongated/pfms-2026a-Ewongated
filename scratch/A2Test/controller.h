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
 * Implements threading infrastructure, goal management, status tracking and
 * odometry access shared between Ackerman and Skidsteer.
 *
 * execute(true) launches a non-blocking background thread that drives the
 * platform through the goal list. execute(false) signals the thread to stop
 * and joins it. All shared state is protected by mutex_.
 *
 * Derived classes must implement driveToGoal(), checkOriginToDestination()
 * and getPlatformType().
 */
class Controller : public ControllerInterface
{
public:
    Controller();
    virtual ~Controller();

    bool execute(bool start) override;
    pfms::PlatformStatus status(unsigned int& currentGoalIndex) override;
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
    bool setTolerance(double tolerance) override;
    double distanceToGoal(void) override;
    double timeToGoal(void) override;
    double distanceTravelled(void) override;
    double timeTravelled(void) override;
    pfms::nav_msgs::Odometry getOdometry(void) override;
    void waitUntilStopped();  // blocks until |v| < STOP_VELOCITY

protected:
    /**
     * @brief Drive platform to a single goal (platform-specific, pure virtual).
     *
     * Must block until within tolerance_ of goal or running_ becomes false,
     * then issue a hard-stop command before returning.
     */
    virtual void driveToGoal(const pfms::geometry_msgs::Point& goal) = 0;

    // Shared state — all access protected by mutex_
    std::shared_ptr<PfmsConnector>          pfmsConnectorPtr_; //!< Simulator connector
    std::vector<pfms::geometry_msgs::Point> goals_;            //!< Ordered goal list
    unsigned int                            currentGoalIndex_; //!< Index currently being pursued
    pfms::PlatformStatus                    platformStatus_;   //!< IDLE / RUNNING / STOPPING
    double                                  tolerance_;        //!< Goal-reached radius [m]
    double                                  distanceTravelled_;//!< Cumulative distance [m]
    double                                  timeTravelled_;    //!< Cumulative in-motion time [s]
    double                                  distanceToGoal_;   //!< Distance to current goal [m]
    double                                  timeToGoal_;       //!< Estimated time to current goal [s]
    pfms::nav_msgs::Odometry                odometry_;         //!< Latest platform odometry

    mutable std::mutex mutex_; //!< Protects all shared state above
    std::thread        thread_; //!< Background driving thread
    std::atomic<bool>  running_; //!< False signals thread to stop

    void run();          //!< Background loop — iterates goals, calls driveToGoal
    void updateOdometry(); //!< Read fresh odometry (must not hold mutex_ when called)

    static double euclidean(const pfms::geometry_msgs::Point& a,
                            const pfms::geometry_msgs::Point& b);
    static double euclidean(const pfms::nav_msgs::Odometry& odo,
                            const pfms::geometry_msgs::Point& pt);
    static double normaliseAngle(double angle);
};

#endif // CONTROLLER_H
