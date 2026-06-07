#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <cmath>
#include <memory>
#include <utility>

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("grid_map_simple_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), 0.1);
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  rclcpp::Rate rate(30.0);
  rclcpp::Clock clock;
  while (rclcpp::ok()) {
    // Add data to grid map.
    rclcpp::Time time = node->now();
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      map.at(
        "elevation",
        *it) = -0.04 + 0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()) * position.x();
    }

    // Let's check if 1.0,1.0  is inside the map.
    grid_map::Position position(0.5, 1.0); // This will be at edge of the map
    // Check if the position is inside the map
    bool isInside = map.isInside(position);

    if (isInside) {
        std::cout << "The position is inside the map." << std::endl;
        // Set the elevation at the position to 2.0
        map.atPosition("elevation", position) = 1.0; // Set elevation to 1 meter
    }
   
    // Publish grid map.
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 1000, "Grid map published.");

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}