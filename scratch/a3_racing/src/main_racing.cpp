/**
 * @file main_racing.cpp
 * @brief Entry point for the racing_node executable.
 *
 * @code
 * ros2 run a3_racing racing_node
 * ros2 run a3_racing racing_node --ros-args -p goal_tolerance:=1.5
 * ros2 run a3_racing racing_node --ros-args -p advanced:=true
 * @endcode
 */
#include "rclcpp/rclcpp.hpp"
#include "racingnode.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacingNode>());
    rclcpp::shutdown();
    return 0;
}
