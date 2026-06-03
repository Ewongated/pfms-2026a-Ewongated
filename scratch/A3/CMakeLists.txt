cmake_minimum_required(VERSION 3.8)
project(racing_track_pkg)

# ── Compile options ────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── Find dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Pure library (no ROS — unit-testable standalone) ──────────────────────────
add_library(track_analyser_lib
    src/track_analyser.cpp
)
target_include_directories(track_analyser_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── ROS 2 node executable ─────────────────────────────────────────────────────
add_executable(racing_track_node
    src/main.cpp
    src/racing_track_node.cpp
)
target_include_directories(racing_track_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(racing_track_node track_analyser_lib)
ament_target_dependencies(racing_track_node
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    visualization_msgs
    tf2
    tf2_ros
)

# ── Unit tests ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(rosbag2_storage REQUIRED)

    # Helper macro — avoids repeating boilerplate for each test executable
    macro(add_ros_gtest test_name test_src)
        ament_add_gtest(${test_name} ${test_src})
        target_include_directories(${test_name} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        target_link_libraries(${test_name} track_analyser_lib)
        ament_target_dependencies(${test_name}
            rclcpp
            sensor_msgs
            nav_msgs
            geometry_msgs
            rosbag2_cpp
            rosbag2_storage
            tf2
            tf2_ros
        )
    endmacro()

    add_ros_gtest(test_obstacle_detection test/test_obstacle_detection.cpp)
    add_ros_gtest(test_goal_in_corridor   test/test_goal_in_corridor.cpp)
    add_ros_gtest(test_waypoint_detection test/test_waypoint_detection.cpp)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS racing_track_node track_analyser_lib
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
