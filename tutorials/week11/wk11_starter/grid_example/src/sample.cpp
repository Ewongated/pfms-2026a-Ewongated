#include "sample.h"
/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser)
 */

using std::cout;
using std::endl;

using std::placeholders::_1;


PfmsSample::PfmsSample()
    : Node("sonar_gridmap"), map({"elevation"})
{
  //Subscribing to odometry
  sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/drone/gt_odom", 10, std::bind(&PfmsSample::odom_callback, this, _1));

  //! @todo Subscribe to the laser as TUTORIAl.md and look at the odometry example
  sub2_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/drone/sonar", 10, std::bind(&PfmsSample::sonar_callback, this, _1));

  pub1_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
  
  // We set the frame id to "world" 
  map.setFrameId("world");

  // Create grid map geometry. and we can supply these parameters on command line
  // or in the launch file
  double map_length_x = this->declare_parameter<double>("map_length_x", 10.0);
  double map_length_y = this->declare_parameter<double>("map_length_y", 10.0);
  double map_resolution = this->declare_parameter<double>("map_resolution", 0.5);
  
  map.setGeometry(grid_map::Length(map_length_x, map_length_y), map_resolution);

  RCLCPP_INFO(
    this->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));


  thread_ = new std::thread(&PfmsSample::process,this);
}

PfmsSample::~PfmsSample()
{
  RCLCPP_INFO_STREAM(get_logger(),"Destructor called");
  if (thread_->joinable()) {
    thread_-> join();
  }
}

void PfmsSample::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    geometry_msgs::msg::Pose pose = msg->pose.pose;

    std::unique_lock<std::mutex> lck1 (dataMtx_);
    robotPose_ = pose; // We store a copy of the pose in robotPose_
    robotPoseReceived_ = true;
    dataReceivedCv_.notify_all();
}

void PfmsSample::sonar_callback(const std::shared_ptr<sensor_msgs::msg::Range> msg)
{
    std::unique_lock<std::mutex> lck2 (dataMtx_);
    range_ = *msg; // We store a copy of the laser data in laserData_
    rangeDataReceived_ = true;
    dataReceivedCv_.notify_all();
}


void PfmsSample::process()
{

    geometry_msgs::msg::Pose robotPose;
    sensor_msgs::msg::Range range;

    rclcpp::Rate loop_rate(1); // 10Hz    
    rclcpp::Clock clock;

    while (rclcpp::ok()) {

      // Wait until both laser and odometry data are received
      {
          std::unique_lock<std::mutex> lock(dataMtx_);
          dataReceivedCv_.wait(lock, [this]() {
            return (!rclcpp::ok() || (rangeDataReceived_.load() && robotPoseReceived_.load()));
        });
      }

        // Exit the loop if ROS is shutting down
      if (!rclcpp::ok()) {
          break;
      }

      {
        std::unique_lock<std::mutex> lck (dataMtx_);
        robotPose = robotPose_;
        range = range_;
        rangeDataReceived_ = false;
        robotPoseReceived_ = false;
 
        lck.unlock();
      }


      // Log robot pose and sonar range
      RCLCPP_INFO_STREAM(get_logger(), "Robot Pose: x: " << robotPose.position.x
                                                          << ", y: " << robotPose.position.y
                                                          << ", z: " << robotPose.position.z);
      RCLCPP_INFO_STREAM(get_logger(), "Sonar Range: " << range.range);

      //! @todo - TASK 3 - Add the sonar data to the grid map
      // To do that, use the quad position and the sonar range to compute the height from the ground
      // and add it to the grid map. Use the function map.atPosition("elevation", position) = height;
      // Thereafter publish the grid map (look at the simple demo node.cpp for an example)


      loop_rate.sleep();

	      //Now to compute the point in global reference frame, use the code you developed in Quiz 3 part A1
    }

    // Notify all waiting threads to ensure graceful shutdown
    dataReceivedCv_.notify_all();
}
