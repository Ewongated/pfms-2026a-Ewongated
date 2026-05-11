cmake_minimum_required(VERSION 3.5)
project(week11_quad)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pfms2ros 3.0.7 REQUIRED) # This is the package we need to use pfms_types.h, we don't need to link against it
# <!-- Add the std_srvs package -->
find_package(std_srvs REQUIRED)

#################################################################
# The library we are developing will be called controllers
add_library(controllers src/controller.cpp src/quadcopter.cpp)
# <!-- Add the std_srvs -->
ament_target_dependencies(controllers rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)

add_executable(sample src/main.cpp)
ament_target_dependencies(sample rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(sample controllers)

install(TARGETS
  sample
  controllers
  DESTINATION lib/${PROJECT_NAME})

ament_package()
