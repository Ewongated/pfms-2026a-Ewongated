#include "controller.h"
#include <cmath>

using std::placeholders::_1;

/**
 * \brief Shared functionality/base class for platform controllers
 *
 */
Controller::Controller() :
    Node("controller"),
    goalSet_(false),    
    distance_travelled_(0),
    time_travelled_(0)
{
    // We create a node handle in derived class (as they have custom messages/topics)  
    // We still have one message we could potentialy use (odo)
    sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/gt_odom", 1000, std::bind(&Controller::odoCallback,this,_1));
    sub2_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 1000, std::bind(&Controller::setGoal,this,_1));

    sub3_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 1000, std::bind(&Controller::setGoalClicked,this,_1));

    //! @todo TASK 2 -- add a subscription to the service of type std_srvs::srv::SetBool, on the topic "/reach_goal" and bind the callback to the control function

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};

//We would now have to sacrifice having a return value to have a setGoal
//At week 10 we do not know about services (which allow us to retrun value
//So to allow to set a goal via topic we forfit having areturn value for now
//At week 11 you can replace this with a service
//bool Controller::setGoal(geometry_msgs::Point goal) {
void Controller::setGoal(const geometry_msgs::msg::Point& msg){    
    std::lock_guard<std::mutex> lock(goalMtx_);
    goal_.location = msg;
    goalSet_=true;
}

void Controller::setGoalClicked(const geometry_msgs::msg::PointStamped& msg){    
    std::lock_guard<std::mutex> lock(goalMtx_);
    goal_.location.x = msg.point.x;
    goal_.location.y = msg.point.y;
    goal_.location.z = 2.0; //We set the z to 2.0 as we are not using this value
    goalSet_=true;
}


bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeInMotion(void) {
    return time_travelled_;
}

GoalStats Controller::getGoalStats(void) {
    std::lock_guard<std::mutex> lock(goalMtx_);
    return goal_;
}

bool Controller::goalReached() {
    if(!goalSet_){return false;};

    GoalStats goalStats = getGoalStats();
    geometry_msgs::msg::Pose pose = getOdometry();

    double dx = goalStats.location.x - pose.position.x;
    double dy = goalStats.location.y - pose.position.y;
    double dz = goalStats.location.z - pose.position.z;

    double distance = pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5);
    RCLCPP_INFO(this->get_logger(), "distance: %f", distance);

    return (distance < tolerance_);
}


void Controller::odoCallback(const nav_msgs::msg::Odometry& msg){
    std::lock_guard<std::mutex> lock(poseMtx_);
    pose_ = msg.pose.pose;
}

geometry_msgs::msg::Pose Controller::getOdometry(void){
    std::lock_guard<std::mutex> lock(poseMtx_);
    return pose_;
}

std::string Controller::getInfoString()
{
    std::stringstream ss;
    switch(status_)
    {
        case pfms::PlatformStatus::IDLE   : ss << "IDLE ";    break;
        case pfms::PlatformStatus::RUNNING : ss << "RUNNING ";  break;
        case pfms::PlatformStatus::TAKEOFF : ss << "TAKEOFF ";  break;
        case pfms::PlatformStatus::LANDING : ss << "LANDING ";  break;
    }
    return ss.str(); // This command convertes trsingstream to string
}
