#include "controller.h"
#include <chrono>
#include <iostream>

/**
 * @brief Constructs the Controller base with safe default state.
 *
 * Platform status starts IDLE. The background driving thread is not launched
 * until execute(true) is called with a non-empty goal list.
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
 * @brief Destructor — stops the background thread cleanly before destruction.
 *
 * Delegates to execute(false) so the thread is always joined and the platform
 * is commanded to stop before the object is released.
 */
Controller::~Controller()
{
    execute(false);
}

bool Controller::execute(bool start)
{
    if (start) {
        // Acquire lock to check preconditions and update state atomically,
        // then release before constructing the thread — the thread calls
        // updateOdometry() which also acquires mutex_.
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

    // Signal the thread to stop, wait for it to finish, then set IDLE.
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

pfms::PlatformStatus Controller::status(unsigned int& currentGoalIndex)
{
    std::lock_guard<std::mutex> lock(mutex_);
    currentGoalIndex = currentGoalIndex_;
    return platformStatus_;
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
    std::lock_guard<std::mutex> lock(mutex_);
    goals_             = goals;
    currentGoalIndex_  = 0;
    distanceTravelled_ = 0.0;
    timeTravelled_     = 0.0;
    distanceToGoal_    = 0.0;
    timeToGoal_        = 0.0;

    // Walk the goal chain using the current odometry as the start pose,
    // advancing the estimated pose after each leg.
    bool reachable = true;
    pfms::nav_msgs::Odometry origin = odometry_;
    for (auto& goal : goals) {
        double distance, time;
        pfms::nav_msgs::Odometry estimatedGoalPose;
        if (!checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose)) {
            reachable = false;
            break;
        }
        origin = estimatedGoalPose;
    }
    return reachable;
}

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

        driveToGoal(goal);

        if (!running_.load()) break;

        updateOdometry();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Accumulate straight-line distance from the pre-leg position to the goal.
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

double Controller::normaliseAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
}
