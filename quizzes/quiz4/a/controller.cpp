#include "controller.h"
#include <cmath>

#define DEBUG 1

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?

Controller::Controller() :
    currentGoalIdx_(0),
    status_(pfms::PlatformStatus::IDLE),
    tolerance_(0.5)
{
};

pfms::PlatformType Controller::getPlatformType(void){
    return type_;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    pfms::nav_msgs::Odometry odo;
    // We need to read the data
    bool success = false;
    for (int attempts = 0; attempts < 3 && !success; attempts++) {
        success = pfmsConnectorPtr_->read(odo);
    }    
    return odo;
}

unsigned int Controller::getGoalsSize(void) {
    std::unique_lock<std::mutex> lck(mtxGoals_);
    return goals_.size();
}

bool Controller::getGoal(unsigned int index, GoalStats& goal) {
    std::unique_lock<std::mutex> lck(mtxGoals_);
    if (index >= goals_.size()) {
        return false;
    }
    goal = goals_.at(index);
    return true;
}


bool Controller::goalReached() {
    pfms::nav_msgs::Odometry odo = getOdometry();
    GoalStats& goal = goals_.at(currentGoalIdx_);
    double dx = goal.location.x - odo.position.x;
    double dy = goal.location.y - odo.position.y;

    return (pow(pow(dx,2)+pow(dy,2),0.5) < tolerance_);
}


pfms::PlatformStatus Controller::status(unsigned int& currentGoalIndex){
    currentGoalIndex = currentGoalIdx_;
    return status_;
}

std::vector<pfms::geometry_msgs::Point> Controller::getObstacles(void){
    std::vector<pfms::geometry_msgs::Point> obstacles;
    return obstacles;
}