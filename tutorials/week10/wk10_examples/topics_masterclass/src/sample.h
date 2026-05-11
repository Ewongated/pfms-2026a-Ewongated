#include <thread>
#include <chrono>
#include <mutex>

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "laserprocessing.h"

class PfmsSample : public rclcpp::Node
{
public:
  PfmsSample();

  ~PfmsSample();

/*! @brief Odometry Callback
  *
  *  @param nav_msgs::OdometryConstPtr - The odometry message
  *  @note This function and the declaration are ROS specific
  */
//void odom_callback(const nav_msgs::msg::Odometry& msg);
void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

 /*! @todo - TASK 1: LaserScan Callback, add a callback for the laser scan
  *
   * @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
void laser_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

 /*! @brief process Main processing function
   *
   *  @note This function is connected to a thread of execution, running every 1s
   *  This is an example, for running a process in parallel to callbacks, using a mutex lock
   *  to protect the data. If needing to run at specific rate, consider using a timer
   *  or a timer callback. If needing to run on new data, consider using convars
   */

  void process(void);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    //! @todo Add a subscriber as TUTORIAl.md and look at the odometry example

    geometry_msgs::msg::Pose robotPose_;   
    std::atomic<bool> robotPoseReceived_ ;  // Flag to indicate odometry data received

    sensor_msgs::msg::LaserScan laserData_;
    std::atomic<bool> laserDataReceived_ ; // Flag to indicate laser data received
    
    std::mutex dataMtx_; // Mutex for shared data access

    std::condition_variable dataReceivedCv_;

    std::unique_ptr<LaserProcessing> laserProcessingPtr_;

    std::thread* thread_;
};

