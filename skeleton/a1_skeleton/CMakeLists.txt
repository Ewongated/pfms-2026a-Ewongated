# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Change project name
project (a1_sup_example)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(BUILD_TESTS ON)

find_package(pfms2ros 3.1.0 REQUIRED) # This is the package that contains the pfmsconnector and cell library. 
find_package(ignition-msgs8 REQUIRED) # required to interact with the gazebo simulator
find_package(ignition-transport11 REQUIRED) # required to interact with the gazebo simulator

# CMake will create your ranger library
add_library(ranger SHARED ranger.cpp laser.cpp sonar.cpp)
target_link_libraries(ranger pfmsconnector)

# CMake will create your fusion library
add_library(fusion SHARED fusion.cpp)
target_link_libraries(fusion ranger)

# CMake will create executable add the cpp files needed
add_executable(a1 main.cpp)

# Cmake will link against the libraries in final step (The below is a demo)
target_link_libraries(a1 cell fusion ranger pfmsconnector)

if(BUILD_TESTS)
   add_subdirectory(test)
endif()