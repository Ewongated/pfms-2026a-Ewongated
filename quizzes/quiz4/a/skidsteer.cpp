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
    running_(false)
{
    // Is this the best polace to set the platform type? Why? 
    type_ = pfms::PlatformType::SKIDSTEER; //Type is skidsteer

    // We create a pointer to the PfmConnector here in the constructor, so we can OPEN connection ONLY once
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(type_);

    // We create a pointer to the LaserProcessing object here in the constructor, so we can OPEN connection ONLY once
    laserProcessingPtr_ = std::make_unique<LaserProcessing>(pfmsConnectorPtr_);

    std::this_thread::sleep_for (std::chrono::milliseconds(50));

    running_=true;

    // We start the thread in the constructor, as we want it to be running while the platform is on
    thread_ = new std::thread(&Skidsteer::reachGoal,this);
}



Skidsteer::~Skidsteer(){

    //Let's stop main execution thread
    running_=false;

    //Gracefully join the thread
    if(thread_->joinable()){
        thread_->join();
    }

}

bool Skidsteer::execute(bool start){

    // We should only start the execution if we have goals to pursue and start is set to true
    // How would you activate the thread to pursue the goals when start is set to true?

    return true;
}


bool Skidsteer::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {

    std::unique_lock lck(mtxGoals_);
    goals_.clear();//We empty the goals
    bool OK=true;
    bool start=true;
    pfms::nav_msgs::Odometry estimatedGoalPose;
    GoalStats goal;
    for(auto g:goals){
        goal.location = g;

        //! @todo
        //! We can use the checkOriginToDestination function to populate the distance and time to goal
        // and also to check if the goal is reachable or not
        goal.distance = -1;
        goal.time = -1;

        goals_.push_back(goal);
    }
    
    lck.unlock();

    return true;
}

void Skidsteer::reachGoal(void) {

    GoalStats goal;
    while(running_.load()){

        //! @todo  This thread is active now (as we start it in the constructor)
        // But we should make it wait until we have goals to pursue and the execute function is called
        // We can use a condition variable for this
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }

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
