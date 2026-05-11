cmake_minimum_required(VERSION 3.5)
project(grid_example)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(sample src/main.cpp src/sample.cpp)

ament_target_dependencies(
  sample rclcpp std_msgs sensor_msgs nav_msgs 
         tf2 tf2_ros tf2_geometry_msgs
         grid_map_core grid_map_msgs grid_map_ros
)


add_executable(simple_demo src/simple_demo_node.cpp)

ament_target_dependencies(
  simple_demo rclcpp grid_map_core grid_map_msgs grid_map_ros
)

install(TARGETS
  sample
  simple_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()