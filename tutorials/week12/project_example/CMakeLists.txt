cmake_minimum_required(VERSION 3.5)
project(project_example)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#################################################################
add_executable(quadcopter src/main_platform.cpp src/controller.cpp src/quadcopter.cpp)
ament_target_dependencies(quadcopter rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

#################################################################
add_executable(mission src/main_mission.cpp src/mission.cpp)
#ament_target_dependencies(platform rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(mission rclcpp std_msgs sensor_msgs nav_msgs tf2 visualization_msgs)


install(TARGETS
  mission
  quadcopter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
