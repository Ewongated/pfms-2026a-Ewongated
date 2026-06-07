#include "controller.h"
#include <chrono>
#include <iostream>

/**
 * @brief Constructs the Controller base with safe default state.
 *
 * All numeric fields are zero-initialised. Status starts as IDLE.
 * The background thread is not started until execute(true) is called.
 */
Controller::Controller()
    : currentGoalIndex_(0)
    , platformStatus_(pfms::PlatformStatus::IDLE)
    , tolerance_(0.5)
    , distanceTravelled_(0.0)
    , timeTravelled_(0.0)
    , distanceToGoal_(0.0)
    , timeToGoal_(0.0)
    , running_(false)
{
    odometry_ = pfms::nav_msgs::Odometry();
}

/**
 * @brief Destructor — stops the driving thread cleanly before destruction.
 */
Controller::~Controller()
{
    execute(false);
}

/**
 * @brief Starts or stops platform execution.
 *
 * execute(true): If goals have been set and no thread is running, launches
 * the background run() thread and returns immediately (non-blocking).
 *
 * execute(false): Signals the running thread to stop, waits for it to join,
 * then sets status back to IDLE.
 *
 * @param start true to start execution, false to stop
 * @return true if the command was accepted
 */
bool Controller::execute(bool start)
{
    if (start) {
        // Check under lock, but release before launching thread —
        // the thread calls updateOdometry() which also acquires mutex_,
        // so the lock must not be held when std::thread is constructed.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_.load() || goals_.empty()) {
                return false;
            }
            running_ = true;
            platformStatus_ = pfms::PlatformStatus::RUNNING;
        }
        thread_ = std::thread(&Controller::run, this);
        return true;
    } else {
        running_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            platformStatus_ = pfms::PlatformStatus::STOPPING;
        }
        if (thread_.joinable()) {
            thread_.join();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        platformStatus_ = pfms::PlatformStatus::IDLE;
        return true;
    }
}

/**
 * @brief Returns the current platform status and active goal index.
 *
 * @param[out] currentGoalIndex Index of the goal currently being pursued (0-based)
 * @return IDLE, RUNNING, or STOPPING
 */
pfms::PlatformStatus Controller::status(unsigned int& currentGoalIndex)
{
    std::lock_guard<std::mutex> lock(mutex_);
    currentGoalIndex = currentGoalIndex_;
    return platformStatus_;
}

/**
 * @brief Sets the list of goals to pursue.
 *
 * Clears any previous goals and resets progress counters. Any running
 * thread should be stopped (execute(false)) before calling this.
 *
 * @param goals Ordered list of waypoints to visit
 * @return true if all goals are reachable (always true for basic check)
 */
bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
    std::lock_guard<std::mutex> lock(mutex_);
    goals_             = goals;
    currentGoalIndex_  = 0;
    distanceTravelled_ = 0.0;
    timeTravelled_     = 0.0;
    distanceToGoal_    = 0.0;
    timeToGoal_        = 0.0;
    return true;
}

/**
 * @brief Sets the goal-reached tolerance.
 * @param tolerance Acceptable distance from goal centre [m]
 * @return true always
 */
bool Controller::setTolerance(double tolerance)
{
    std::lock_guard<std::mutex> lock(mutex_);
    tolerance_ = tolerance;
    return true;
}

/**
 * @brief Returns the estimated distance remaining to the current goal.
 * @return Distance [m]
 */
double Controller::distanceToGoal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return distanceToGoal_;
}

/**
 * @brief Returns the estimated time remaining to the current goal.
 * @return Time [s]
 */
double Controller::timeToGoal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return timeToGoal_;
}

/**
 * @brief Returns the total distance the platform has travelled since execution began.
 * @return Distance [m]
 */
double Controller::distanceTravelled()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return distanceTravelled_;
}

/**
 * @brief Returns the total time the platform has been in motion.
 * @return Time [s]
 */
double Controller::timeTravelled()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return timeTravelled_;
}

/**
 * @brief Returns the latest platform odometry.
 * @return Current pose and velocity
 */
pfms::nav_msgs::Odometry Controller::getOdometry()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return odometry_;
}

/**
 * @brief Background driving loop.
 *
 * Iterates through the goal list, calling driveToGoal() for each.
 * Tracks distance travelled by sampling odometry before and after each
 * goal. Sets status to IDLE when the list is exhausted or running_ is
 * cleared by execute(false).
 */
void Controller::run()
{
    updateOdometry();
    pfms::nav_msgs::Odometry prevOdo;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        prevOdo = odometry_;
    }

    while (running_.load()) {
        unsigned int goalIdx;
        size_t totalGoals;
        pfms::geometry_msgs::Point goal;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goalIdx    = currentGoalIndex_;
            totalGoals = goals_.size();
            if (goalIdx >= totalGoals) break;
            goal = goals_.at(goalIdx);
        }

        // Drive to this goal (blocks until reached or running_ cleared)
        driveToGoal(goal);

        if (!running_.load()) break;

        // Update distance and advance goal index
        updateOdometry();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Recalibrate distance estimate using actual position at goal
            double stepDist = euclidean(prevOdo, goal);
            distanceTravelled_ += stepDist;
            prevOdo = odometry_;
            currentGoalIndex_++;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        platformStatus_ = pfms::PlatformStatus::IDLE;
    }
    running_ = false;
}

/**
 * @brief Reads a fresh odometry sample from the simulator.
 *
 * Updates odometry_ under the mutex. Sleeps briefly to avoid busy-waiting
 * against the simulator's publish rate (~10Hz).
 */
void Controller::updateOdometry()
{
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);
    std::lock_guard<std::mutex> lock(mutex_);
    odometry_ = odo;
}

/**
 * @brief Computes Euclidean distance between two points (ignoring z).
 * @param a First point
 * @param b Second point
 * @return Distance [m]
 */
double Controller::euclidean(const pfms::geometry_msgs::Point& a,
                              const pfms::geometry_msgs::Point& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * @brief Computes Euclidean distance from an odometry position to a point.
 * @param odo Platform odometry
 * @param pt Target point
 * @return Distance [m]
 */
double Controller::euclidean(const pfms::nav_msgs::Odometry& odo,
                              const pfms::geometry_msgs::Point& pt)
{
    double dx = odo.position.x - pt.x;
    double dy = odo.position.y - pt.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * @brief Normalises an angle to the range [-pi, pi].
 * @param angle Input angle [rad]
 * @return Normalised angle [rad]
 */
double Controller::normaliseAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}