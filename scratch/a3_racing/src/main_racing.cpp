#include "rclcpp/rclcpp.hpp"
#include "racingnode.h"

/**
 * @brief Entry point for the racing_node executable.
 *
 * Usage:
 * @code
 * ros2 run a3_racing racing_node
 * ros2 run a3_racing racing_node --ros-args -p goal_tolerance:=2.0
 * ros2 run a3_racing racing_node --ros-args -p advanced:=true
 * @endcode
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacingNode>());
    rclcpp::shutdown();
    return 0;
}