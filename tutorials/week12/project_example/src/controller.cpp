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
    time_travelled_(0),
    cmd_pipe_seq_(0)
{
    // We open up the pipes here in the constructor, so we can OPEN them once ONLY
    //pipesPtr_ = new Pipes();
    // Now we create a node handle in derived class (as they have custom messages/topics)  
    // We still have one message we could potentialy use (odo)
    sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/gt_odom", 1000, std::bind(&Controller::odoCallback,this,_1));
    sub2_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 1000, std::bind(&Controller::setGoal,this,_1));

};

//We would now have to sacrifice having a return value to have a setGoal
//At week 10 we do not know about services (which allow us to retrun value
//So to allow to set a goal via topic we forfit having areturn value for now
//At week 11 you can replace this with a service
//bool Controller::setGoal(geometry_msgs::Point goal) {
void Controller::setGoal(const geometry_msgs::msg::Point& msg){    
  goal_ = msg;
  goalSet_=true;
}

bool Controller::goalReached() {
    geometry_msgs::msg::Pose pose = getOdometry();//This will update internal copy of odometry, as well as return value if needed.
    // Calculate absolute travel angle required to reach goal
    double dx = goal_.x - pose.position.x;
    double dy = goal_.y - pose.position.y;
    double dz = goal_.z - pose.position.z;

    return (pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5) < tolerance_);
}


void Controller::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    std::unique_lock<std::mutex> lck(mtx_);
    pose_ = msg->pose.pose;
    // RCLCPP_INFO_STREAM_THROTTLE(get_logger(),*get_clock(),5.0,"callback x,y" 
    //     << pose_.position.x << "," << pose_.position.y );    
}

//Do we need below ... maybe we can use it and impose a mutex
//To secure data?
geometry_msgs::msg::Pose Controller::getOdometry(void){
    std::unique_lock<std::mutex> lck(mtx_);
    geometry_msgs::msg::Pose pose=pose_;
    return pose;
}

