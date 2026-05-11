#include "sample.h"
/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser)
 */

using std::cout;
using std::endl;

using std::placeholders::_1;


PfmsSample::PfmsSample()
    : Node("laser_example")
{
  //Subscribing to odometry
  sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/orange/odom", 10, std::bind(&PfmsSample::odom_callback, this, _1));

  //! @todo Subscribe to the laser as TUTORIAl.md and look at the odometry example

  //thread_(std::thread([this]() -> void {process();}))
  thread_ = new std::thread(&PfmsSample::process,this);
}

PfmsSample::~PfmsSample()
{
  if (thread_->joinable()) {
    thread_-> join();
  }
}



// A callback for odometry
//void PfmsSample::odom_callback(const nav_msgs::msg::Odometry& msg)
void PfmsSample::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{



    geometry_msgs::msg::Pose pose = msg->pose.pose;

    std::unique_lock<std::mutex> lck1 (dataMtx_);
    robotPose_ = pose; // We store a copy of the pose in robotPose_
    robotPoseReceived_ = true;
    dataReceivedCv_.notify_all();
}

//! @todo - Ex 1: Add a callback for the laser scan, look at odo callback example



void PfmsSample::process()
{

    geometry_msgs::msg::Pose robotPose;
    sensor_msgs::msg::LaserScan laserData;

    rclcpp::Rate loop_rate(10); // 10Hz    

    while (rclcpp::ok()) {

      // Wait until both laser and odometry data are received
      {
          std::unique_lock<std::mutex> lock(dataMtx_);
          dataReceivedCv_.wait(lock, [this]() {
            return (!rclcpp::ok() || (laserDataReceived_.load() && robotPoseReceived_.load()));
        });
      }

        // Exit the loop if ROS is shutting down
      if (!rclcpp::ok()) {
          break;
      }

      {
        std::unique_lock<std::mutex> lck (dataMtx_);
        robotPose = robotPose_;
        laserData = laserData_;
        laserDataReceived_ = false;
        robotPoseReceived_ = false;
 
        lck.unlock();
      }


        //If we have not created the laserProcessingPtr object, create it
        if(laserProcessingPtr_ == nullptr){
            laserProcessingPtr_ = std::make_unique<LaserProcessing>(laserData);
        }
        else{
            laserProcessingPtr_->newScan(laserData);
        }

        geometry_msgs::msg::Point pt = laserProcessingPtr_->closestPoint();

        // NOTE: we can use 2 different verbosty levels, and swicth then ON/OFF dynamically
        // RCLCPP_INFO_STREAM();
        // RCLCPP_DEBUG_STREAM();

        RCLCPP_INFO_STREAM(get_logger(),"Closest Point: x: " << pt.x << ", y: " << pt.y);


        /**
         * @todo - TASK 2: Obtain a pose (x,y yaw) from nav_msgs/Odometry
         *

        * - On command line type 'ros2 interface show nav_msgs/msg/Odometry'
        * - The position and orientation are in two seperate parts of the message
        * - The orinetation is provided as a quaternion
        * - Which angle to we need?
        * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
        * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
        * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
        * - Where is time of this message stored
        */

        //Let's slow this down to the required rate      
        loop_rate.sleep();

    }
}