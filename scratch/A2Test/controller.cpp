#include "controller.h"
#include <chrono>
#include <iostream>

/**
 * @brief Constructs the Controller base with safe default state.
 * Status starts IDLE. Background thread not started until execute(true).
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
    odometry_ = pfms::nav_msgs::Odometry{};
}

/**
 * @brief Destructor — joins background thread cleanly.
 */
Controller::~Controller()
{
    execute(false);
}

/**
 * @brief Starts or stops platform execution.
 *
 * execute(true): If goals set and no thread running, launches run() in a
 * background thread and returns immediately (non-blocking).
 * execute(false): Clears running_, joins thread, sets IDLE.
 */
bool Controller::execute(bool start)
{
    if (start) {
        // Check state under lock but release before constructing thread —
        // the thread calls updateOdometry() which also acquires mutex_.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_.load() || goals_.empty()) {
                return false;
            }
            running_        = true;
            platformStatus_ = pfms::PlatformStatus::RUNNING;
        }
        thread_ = std::thread(&Controller::run, this);
        return true;
    }

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

/**
 * @brief Returns current platform status and active goal index.
 */
pfms::PlatformStatus Controller::status(unsigned int& currentGoalIndex)
{
    std::lock_guard<std::mutex> lock(mutex_);
    currentGoalIndex = currentGoalIndex_;
    return platformStatus_;
}

/**
 * @brief Replaces the goal list and resets all progress counters.
 * Stop any running thread via execute(false) before calling this.
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
    bool reachable = true;
    pfms::nav_msgs::Odometry origin = odometry_;
    for (auto& goal : goals) {
        double distance, time;
        pfms::nav_msgs::Odometry estimatedGoalPose;
        if (!checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose)) {
            // std::cout << "[setGoals] Goal (" << goal.x << ", " << goal.y << ") unreachable" << std::endl;
            reachable = false;
            break;
        }
        // std::cout << "[setGoals] Goal (" << goal.x << ", " << goal.y << ") reachable, distance=" << distance << " time=" << time << std::endl;
        origin = estimatedGoalPose;
    }
    // std::cout << "[setGoals] All goals reachable=" << reachable << std::endl;
    return reachable;
}

/**
 * @brief Sets the goal-reached tolerance radius [m].
 */
bool Controller::setTolerance(double tolerance)
{
    std::lock_guard<std::mutex> lock(mutex_);
    tolerance_ = tolerance;
    return true;
}

double Controller::distanceToGoal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return distanceToGoal_;
}

double Controller::timeToGoal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return timeToGoal_;
}

double Controller::distanceTravelled()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return distanceTravelled_;
}

double Controller::timeTravelled()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return timeTravelled_;
}

pfms::nav_msgs::Odometry Controller::getOdometry()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return odometry_;
}

/**
 * @brief Background driving loop.
 *
 * Iterates through goals_, calling driveToGoal() for each. Updates
 * distanceTravelled_ and currentGoalIndex_ after each goal, then sets IDLE.
 */
void Controller::run()
{
    updateOdometry();

    while (running_.load()) {
        pfms::geometry_msgs::Point goal;
        pfms::nav_msgs::Odometry   prevOdo;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (currentGoalIndex_ >= goals_.size()) break;
            goal    = goals_[currentGoalIndex_];
            prevOdo = odometry_;
        }
        
        std::cout << "[run] Pursuing goal " << currentGoalIndex_ 
        << " -> (" << goal.x << ", " << goal.y << ")" << std::endl;


        driveToGoal(goal);
        std::cout << "[driveToGoal] Returning to run()" << std::endl;
        
        if (!running_.load()) break;

        std::cout << "[run] Goal " << currentGoalIndex_ << " complete, incrementing index." << std::endl;

        updateOdometry();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Accumulate straight-line distance from pre-leg position to goal
            distanceTravelled_ += euclidean(prevOdo.position, goal);
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
 * @brief Reads fresh odometry from the simulator connector.
 * Must not be called while mutex_ is held (PfmsConnector::read blocks).
 */
void Controller::updateOdometry()
{
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);
    std::lock_guard<std::mutex> lock(mutex_);
    odometry_ = odo;
}

double Controller::euclidean(const pfms::geometry_msgs::Point& a,
                              const pfms::geometry_msgs::Point& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double Controller::euclidean(const pfms::nav_msgs::Odometry& odo,
                              const pfms::geometry_msgs::Point& pt)
{
    const double dx = odo.position.x - pt.x;
    const double dy = odo.position.y - pt.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Wraps angle to [-pi, pi] using fmod (safe for large/NaN inputs).
 */
double Controller::normaliseAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
}
