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
  
  // Create grid map.
  // grid_map::GridMap map({"elevation"});
  map.setFrameId("world");

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



// A callback for odometry
//void PfmsSample::odom_callback(const nav_msgs::msg::Odometry& msg)
void PfmsSample::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    geometry_msgs::msg::Pose pose = msg->pose.pose;

    std::unique_lock<std::mutex> lck (dataMtx_);
    robotPose_ = pose; // We store a copy of the pose in robotPose_
    robotPoseReceived_ = true;
}

//! @todo - Ex 1: Add a callback for the laser scan, look at odo callback example

void PfmsSample::sonar_callback(const std::shared_ptr<sensor_msgs::msg::Range> msg)
{
    std::unique_lock<std::mutex> lck (dataMtx_);
    range_ = *msg; // We store a copy of the laser data in laserData_
    rangeDataReceived_ = true;
}


void PfmsSample::process()
{

  geometry_msgs::msg::Pose robotPose;
  sensor_msgs::msg::Range range;

  rclcpp::Rate loop_rate(10); // 10Hz    
  rclcpp::Clock clock;

  // Publish grid map every 5 seconds
  rclcpp::Time last_publish_time = this->now();

  while(!(rangeDataReceived_.load() && robotPoseReceived_.load())) {
    RCLCPP_INFO(get_logger(), "Waiting for odometry and sonar data...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(!rclcpp::ok()) {
      return; // This will exit the loop, allowing to join if ROS is shutting down
    }
  }

  while (rclcpp::ok()) {
    // Wait until both sonar and odometry data are received
    {
      std::unique_lock<std::mutex> lck(dataMtx_);
      robotPose = robotPose_;
      range = range_;
    }

    // As the assessment contains a project which looks at using sonar data to augument the map
    // we will not provide a full implementation here, rather we will provide some pointers.

    // Firstly, with any sensor data we need to check if it is valid, and this is done
    // by checking the min and max range of the sensor. If the data is not valid, we will 
    // skip processing it.

    // Then we consider sonar data is in the robot frame, so we need to transform it to the world frame    
    // We can thereafter use the position of the robot and the range data to create a point in 3D space
    // and then transform it to world coordinates. 

    // So, we will use come ficticous values for the sonar data, and we will assume that the sonar is
    range.range = 2.0; // Example range value

    // Transform sonar range to world coordinates and update the grid map
    tf2::Quaternion q(
        robotPose.orientation.x,
        robotPose.orientation.y,
        robotPose.orientation.z,
        robotPose.orientation.w);
    tf2::Transform robotToWorld(q, tf2::Vector3(
        robotPose.position.x,
        robotPose.position.y,
        robotPose.position.z));

    tf2::Vector3 sonarPoint(0.0, 0.0, -range.range); // Assuming sonar is along the x-axis
    tf2::Vector3 worldPoint = robotToWorld * sonarPoint;

    // Check if the point is inside the map
    grid_map::Position position(worldPoint.x(), worldPoint.y());

    if (map.isInside(position)) {

      // You can get current value
      double value = map.atPosition("elevation", position);

      // Let's set map to the value of the sonar in global coordinates

      // You can set the value of the elevation layer at the position , supplying the height as the value.
      map.atPosition("elevation", position) = worldPoint.z();

    }

    // Publishing the map is a large amount of data, so we will not do it every time
    // Check if 5 seconds have passed since the last publish
    // You would not need to publish it frequently especially if your node uses the map directly.
    rclcpp::Time current_time = this->now();
    if ((current_time - last_publish_time).seconds() >= 5.0) {
      // Publish grid map
      map.setTimestamp(current_time.nanoseconds());
      std::unique_ptr<grid_map_msgs::msg::GridMap> message;
      message = grid_map::GridMapRosConverter::toMessage(map);
      pub1_->publish(std::move(message));
      RCLCPP_INFO(get_logger(), "Grid map published.");
      last_publish_time = current_time; // Update the last publish time
    }

    loop_rate.sleep();
  }

}
