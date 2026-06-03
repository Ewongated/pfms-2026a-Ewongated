#include <rclcpp/rclcpp.hpp>
#include "racing_track_pkg/racing_track_node.h"

/**
 * @brief Entry point for the racing_track_node executable.
 *
 * Initialises rclcpp, spins the RacingTrackNode (handles subscriber callbacks
 * and service callbacks on the main thread), and shuts down cleanly.
 *
 * Launch via:
 * @code
 * ros2 run racing_track_pkg racing_track_node
 * @endcode
 * or with parameters:
 * @code
 * ros2 run racing_track_pkg racing_track_node --ros-args -p advanced:=true -p track_width:=8.5
 * @endcode
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacingTrackNode>());
    rclcpp::shutdown();
    return 0;
}
