
#include "rclcpp/rclcpp.hpp"
#include "sample.h"


int main(int argc, char **argv)
{


    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   *
   * The third argument below "week10" is the rosnode name
   * The name must be unique, only one node of the same name can ever register with the roscore
   * If a rosnode with same name exists, it will be terminated
   */
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PfmsSample>());

  /**
   * Let's cleanup everything, shutdown ros
   */
  rclcpp::shutdown();
 
  return 0;
}

