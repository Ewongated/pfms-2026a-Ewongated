cmake_minimum_required(VERSION 3.5)
project(a3_racing)

# ── C++ standard ──────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

# ── Find packages ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# ── laserprocessing static library ───────────────────────────────────────────
# Built as STATIC so it is linked directly into the executable and tests —
# no separate .so file is needed at runtime.
add_library(laserprocessing STATIC
  src/laserprocessing.cpp
)
ament_target_dependencies(laserprocessing
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# ── racing_node executable ────────────────────────────────────────────────────
add_executable(racing_node
  src/main_racing.cpp
  src/racingnode.cpp
)
ament_target_dependencies(racing_node
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(racing_node laserprocessing)

# ── Install targets ───────────────────────────────────────────────────────────
install(TARGETS racing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_racing test/utest.cpp)
  target_link_libraries(test_racing laserprocessing)
  ament_target_dependencies(test_racing
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    rosbag2_cpp
  )
endif()

ament_package()