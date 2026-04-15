# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Add project name
project (week08)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(MYDEBUG CACHE STRING "Additional definition" ON) 


# To enable testsing we instaed set BUILD_TEST to ON by default
set(MARKING_TESTS ON)

## Include the location of pfms headers
find_package(pfms2ros 3.1.1 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

#########################################################################
# We have shared an example implementation and the below incorporates this
set(A1_EXTERNAL_CMAKE "$ENV{HOME}/.ros/pfms/a1/CMakeLists.txt")
set(A1_LOCAL_CMAKE "${CMAKE_CURRENT_SOURCE_DIR}/dep/CMakeLists.txt")

if(NOT EXISTS "$ENV{HOME}/.ros/pfms/a1")
    message(STATUS "Folder $ENV{HOME}/.ros/pfms/a1 not found, running ros2 run pfms download_rosbags.py")
    execute_process(COMMAND ros2 run pfms download_rosbags.py)
endif()

if(EXISTS "${A1_EXTERNAL_CMAKE}")
    include(${A1_EXTERNAL_CMAKE})
elseif(EXISTS "${A1_LOCAL_CMAKE}")
    message(WARNING "${A1_EXTERNAL_CMAKE} not found, falling back to local dep/CMakeLists.txt")
    include(${A1_LOCAL_CMAKE})
else()
    message(FATAL_ERROR "No A1 dependency CMakeLists found at ${A1_EXTERNAL_CMAKE} or ${A1_LOCAL_CMAKE}")
endif()
#########################################################################

add_executable(ex01 main.cpp)
target_link_libraries(ex01 pfmsconnector cell ranger fusion ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_dependencies(ex01 pfms2ros::pfmsconnector)

##################################################################################################
# The below enables tests
# We have only provided 2 tests
if(MARKING_TESTS)
    add_subdirectory(marking)
endif()
