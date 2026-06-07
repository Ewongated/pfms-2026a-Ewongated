#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#define DEBUG 1
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG

using std::cout;
using std::endl;
using namespace std::chrono_literals;

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 - Initialisation
//!
//! What do we need to subscribe to and publish?

Quadcopter::Quadcopter() :
    liftoff_(false),TARGET_SPEED(0.4),
    TARGET_HEIGHT_TOLERANCE(0.2)
{
    tolerance_=0.5;//We set tolerance to be default of 0.5

    pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel",3);  
    pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("drone/takeoff",3);  

    // Rather than a thread of execution we use a timer to call the reachGoal function at a set interval
    timer_ = this->create_wall_timer(   
        100ms, std::bind(&Quadcopter::reachGoal, this));

};

Quadcopter::~Quadcopter(){
   
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {


    if(!liftoff_){
        std_msgs::msg::Empty msg = std_msgs::msg::Empty();
        pubTakeOff_->publish(msg);
    }

    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
    //OR we can do auto msg = geometry_msgs::msg::Twist();
    msg.linear.x= move_f_b;
    msg.linear.y= move_l_r;
    msg.linear.z= move_u_d;
    msg.angular.z = turn_l_r;
    pubCmdVel_->publish(msg);
}

bool Quadcopter::reachGoal(void) {
    if(!goalSet_){return false;};

    geometry_msgs::msg::Pose pose = getOdometry();//This will update internal copy of odometry, as well as return value if needed.
    double dx = goal_.x - pose.position.x;
    double dy = goal_.y - pose.position.y;
    double target_angle = std::atan2(dy, dx);

    // Get relative target angle
    //double theta = pose_.yaw - target_angle_;
    double theta = tf2::getYaw(pose.orientation) - target_angle;

    // Move at `speed` in target direction
    dx = TARGET_SPEED * std::cos(theta);
    dy = TARGET_SPEED * std::sin(theta);

    //What about the height?
    double dz=0;

    if(pose.position.z>(goal_.z+TARGET_HEIGHT_TOLERANCE)){
        dz=-0.05;
    }
    if(pose.position.z<(goal_.z+TARGET_HEIGHT_TOLERANCE)){
        dz=+0.05;
    }

    bool reached = goalReached();  

    if(reached){
        goalSet_=false;
        // Stop thq quadcopter immediately
        sendCmd(0, 0, 0, 0);
        ROS_INFO("Goal reached");
    }
    else{
        //Let's send command with these parameters
        sendCmd(0, -dy, dz, dx);
        // RCLCPP_INFO_STREAM_THROTTLE(get_logger(),*get_clock(),5.0,"x,y,theta:" 
        //     << pose.position.x <<"," << pose.position.y << "," << theta << " send: " << dx << " " << -dy << " " << dz);
    }

    return reached;
}
