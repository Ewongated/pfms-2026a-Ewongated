#include "skidsteer.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

//#define DEBUG 1

using std::cout;
using std::endl;

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor
//! We have some constants defined for the Skidsteer, and have initialised some others for you.

Skidsteer::Skidsteer() :
    MAX_LINEAR_SPEED(2.0),
    MAX_ANGULAR_SPEED(1.0),
    seq_(0),
    distanceTravelled_(0.0),   // TASK 1
    timeInMotion_(0.0),        // TASK 1
    goalSet_(false),           // TASK 1
    running_(false)
{
    type_ = pfms::PlatformType::SKIDSTEER;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(type_);
    laserProcessingPtr_ = std::make_unique<LaserProcessing>(pfmsConnectorPtr_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    running_ = true;
    thread_ = new std::thread(&Skidsteer::reachGoal, this);
}



Skidsteer::~Skidsteer(){

    //Let's stop main execution thread
    running_=false;

    //Gracefully join the thread
    if(thread_->joinable()){
        thread_->join();
    }

}

bool Skidsteer::execute(bool start) {
    if (start) {
        std::unique_lock<std::mutex> lck(mtxGoals_);
        if (goals_.empty()) return false;
        status_ = pfms::PlatformStatus::RUNNING;
        lck.unlock();
        cv_.notify_all();   // wake reachGoal thread
    } else {
        status_ = pfms::PlatformStatus::IDLE;
        sendCmd(0, 0);      // stop the platform
    }
    return true;
}

bool Skidsteer::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    std::unique_lock<std::mutex> lck(mtxGoals_);
    goals_.clear();
    currentGoalIdx_ = 0;

    pfms::nav_msgs::Odometry odo = getOdometry();
    pfms::nav_msgs::Odometry estimatedPose;
    pfms::nav_msgs::Odometry origin = odo;

    bool allReachable = true;
    for (auto& g : goals) {
        GoalStats gs;
        gs.location = g;
        double dist, time;
        bool ok = checkOriginToDestination(origin, g, dist, time, estimatedPose);
        gs.distance = ok ? dist : -1;
        gs.time     = ok ? time : -1;
        if (!ok) allReachable = false;
        goals_.push_back(gs);
        origin = estimatedPose;   // chain goals sequentially
    }
    return allReachable;
}

void Skidsteer::reachGoal(void) {
    while (running_.load()) {

        // Wait until execute(true) is called and goals exist
        {
            std::unique_lock<std::mutex> lck(mtxGoals_);
            cv_.wait(lck, [this] {
                return !running_.load() ||
                       (status_ == pfms::PlatformStatus::RUNNING && !goals_.empty());
            });
        }

        if (!running_.load()) break;

        // Work through goals one at a time
        while (running_.load() && status_ == pfms::PlatformStatus::RUNNING) {

            GoalStats goal;
            {
                std::unique_lock<std::mutex> lck(mtxGoals_);
                if (currentGoalIdx_ >= goals_.size()) {
                    status_ = pfms::PlatformStatus::IDLE;
                    sendCmd(0, 0);
                    break;
                }
                goal = goals_.at(currentGoalIdx_);
            }

            // --- Control loop: rotate then drive ---
            auto lastTime = std::chrono::steady_clock::now();
            pfms::nav_msgs::Odometry lastOdo = getOdometry();

            while (running_.load() && status_ == pfms::PlatformStatus::RUNNING) {

                pfms::nav_msgs::Odometry odo = getOdometry();

                // Track distance and time
                auto now = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(now - lastTime).count();
                lastTime = now;

                double dStep = std::hypot(odo.position.x - lastOdo.position.x,
                                          odo.position.y - lastOdo.position.y);
                distanceTravelled_ += dStep;
                timeInMotion_      += dt;
                lastOdo = odo;

                // Update goal distance/time estimates
                {
                    std::unique_lock<std::mutex> lck(mtxGoals_);
                    double dx = goals_.at(currentGoalIdx_).location.x - odo.position.x;
                    double dy = goals_.at(currentGoalIdx_).location.y - odo.position.y;
                    goals_.at(currentGoalIdx_).distance = std::hypot(dx, dy);
                    goals_.at(currentGoalIdx_).time =
                        goals_.at(currentGoalIdx_).distance / MAX_LINEAR_SPEED;
                }

                // Check if goal reached
                if (goalReached()) {
                    sendCmd(0, 0);
                    std::unique_lock<std::mutex> lck(mtxGoals_);
                    currentGoalIdx_++;
                    break;
                }

                // Compute desired heading to goal
                double dx = goal.location.x - odo.position.x;
                double dy = goal.location.y - odo.position.y;
                double desiredYaw = std::atan2(dy, dx);
                double yawErr = desiredYaw - odo.yaw;
                // Normalise to [-pi, pi]
                yawErr = std::atan2(std::sin(yawErr), std::cos(yawErr));

                // Proportional controller
                double angular = std::max(-MAX_ANGULAR_SPEED,
                                 std::min(MAX_ANGULAR_SPEED, 1.5 * yawErr));

                // Only drive forward once roughly aligned
                double linear = 0.0;
                if (std::fabs(yawErr) < 0.3) {
                    double dist = std::hypot(dx, dy);
                    linear = std::min(MAX_LINEAR_SPEED, dist);
                }

                sendCmd(angular, linear);
            }
        }
    }
    sendCmd(0, 0);
}



// We have fully implemented the checkOriginToDestination function for you for Husky
bool Skidsteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
    double& distance, double& time,
    pfms::nav_msgs::Odometry& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;

    distance = std::hypot(dx, dy);
    time = distance / MAX_LINEAR_SPEED; // Assuming max speed, as Skidsteer can adjust its speed
    double angle_to_goal = std::atan2(dy, dx);
    double angle_diff = angle_to_goal - origin.yaw;
    // Normalize angle_diff to [-pi, pi]
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
    double turn_time = std::fabs(angle_diff) / MAX_ANGULAR_SPEED;
    time += turn_time; // Total time is turn time + drive time  

    // The estimated goal pose would be the goal at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;
    estimatedGoalPose.linear.x = 0;
    estimatedGoalPose.linear.y = 0;

    return true;
}


double Skidsteer::distanceToGoal(void) {
    GoalStats goal;
    if (!getGoal(currentGoalIdx_, goal)) {
        return 0;
    }
    return goal.distance;
}

double Skidsteer::timeToGoal(void) {
    GoalStats goal;
    if (!getGoal(currentGoalIdx_, goal)) {
        return 0;
    }
    return goal.time;
}

bool Skidsteer::setTolerance(double tolerance) {
    if(tolerance < 0){
        // std::cerr << "Tolerance must be positive" << std::endl;
        return false;
    }
    tolerance_ = tolerance;
    return true;
}

double Skidsteer::distanceTravelled(void) {
    return distanceTravelled_;
}

double Skidsteer::timeTravelled(void) {
    return timeInMotion_;
}

std::vector<pfms::geometry_msgs::Point> Skidsteer::getObstacles(void) {

    // std::cout << "Calling getObstacles" << std::endl;
    std::vector<pfms::geometry_msgs::Point> obstacles;

    //Transform the data using the odometry
    pfms::nav_msgs::Odometry odo = getOdometry();

    // This will transform the obstacles from the local frame of the platform to the global frame
    // Using the odometry information
    for (auto& obstacle : laserProcessingPtr_->getObstacles()) {
        // Transform the obstacle point using the odometry
        pfms::geometry_msgs::Point transformedObstacle;
        transformedObstacle.x = obstacle.x * cos(odo.yaw) - obstacle.y * sin(odo.yaw) + odo.position.x;
        transformedObstacle.y = obstacle.x * sin(odo.yaw) + obstacle.y * cos(odo.yaw) + odo.position.y;
        transformedObstacle.z = obstacle.z + odo.position.z;

        obstacles.push_back(transformedObstacle);

    }

    return obstacles;
}


void Skidsteer::sendCmd(double turn_l_r, double move_f_b) {
    pfms::commands::SkidSteer cmd = {
        seq_++,
        turn_l_r,
        move_f_b,
    };
    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));//Small delay to ensure message sent
}
