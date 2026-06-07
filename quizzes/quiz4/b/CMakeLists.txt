cmake_minimum_required(VERSION 3.5)
project(quiz4_b)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(analysis src/analysis.cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener analysis)
ament_target_dependencies(listener rclcpp std_msgs)

##################################################################################################
add_subdirectory(test)

install(TARGETS
  listener 
  utest
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
