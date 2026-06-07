cmake_minimum_required(VERSION 3.5)
project(a3_support)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(TinyXML2 REQUIRED) # To parse the XML files
find_package(ament_index_cpp REQUIRED) # to gte the path to the other packages
find_package(tf2_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)

add_executable(goals_logger src/goals_logger.cpp )
ament_target_dependencies(goals_logger rclcpp geometry_msgs visualization_msgs)

add_executable(goals_publisher src/goals_publisher.cpp )
ament_target_dependencies(goals_publisher rclcpp geometry_msgs visualization_msgs)

add_executable(single_message_recorder src/simple_bag_recorder.cpp )
ament_target_dependencies(single_message_recorder rclcpp rosbag2_cpp sensor_msgs nav_msgs std_msgs geometry_msgs tf2_msgs grid_map_msgs)

add_executable(stl_publisher src/stl_publisher.cpp )
ament_target_dependencies(stl_publisher rclcpp visualization_msgs)
target_link_libraries(stl_publisher tinyxml2::tinyxml2 ament_index_cpp::ament_index_cpp)

add_executable(tf_static_republisher src/tf_static_republisher.cpp )
ament_target_dependencies(tf_static_republisher rclcpp tf2_msgs)

install(TARGETS
  goals_logger
  goals_publisher
  single_message_recorder
  stl_publisher
  tf_static_republisher
  DESTINATION lib/${PROJECT_NAME})

# We are also going to install the data directory that has sample goals.
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
