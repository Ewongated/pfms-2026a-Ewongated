#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

//!@todo: TASK 5: Include the Trigger message
// Syntax: #include "package_name/srv/service_name.hpp"

#include "laserprocessing.h"


/**
 * This node shows some connections and publishing images
 */


class Detection : public rclcpp::Node{

public:
  /*! @brief Detection constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    Detection();

  /*! @brief Detection destructor.
   *
   *  Will tear down the object
   */
    ~Detection();


    /*! @todo - TASK 5: Change the detect function so it can be a callback for the service
     *  Syntax: void function_name(const std::shared_ptr<service_type::Request>  req,
      *                             std::shared_ptr<service_type::Response> res);
     */
      void detect();

private:

      /*! @brief LaserScan Callback
        *
        *  @param std::shared_ptr<sensor_msgs::msg::LaserScan - The laserscan message as a const pointer
        *  @note This function and the declaration are ROS specific to handle callbacks
        */
         void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

private:

    //! @todo: TASK 5: Create a service object
    //! Syntax: rclcpp::Service<service_type>::SharedPtr variable_name_;


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;//!< Pointer to the laser scan subscriber
    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object

    struct LaserData
    {
        sensor_msgs::msg::LaserScan scan;
        std::mutex mtx;
    } laserData_; //!< Laser data structure containing the laser scan and a mutex to protect it

};

