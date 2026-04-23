# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.1)

project (sol4_a)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# Export compile commands for linters and IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BUILD_TESTS ON) # This switch here sets the tests ON automatically
set(MARKING_TESTS OFF) # This switch here sets the tests ON automatically

find_package(pfms2ros 3.1.1 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# Add a laserprocessing library
add_library(laserprocessing laserprocessing.cpp)
# We need to link it against pfmsconnector, because we use it
target_link_libraries(laserprocessing pfmsconnector)

#################################################################
# The library we are developing will be called controllers
add_library(controllers controller.cpp skidsteer.cpp)
# We need to link it against pfmsconnector and the laserprocessing
target_link_libraries(controllers pfmsconnector laserprocessing)

#######################################
# The below is our an example of our automated tets for marking
# tests can be run from build directory (for this example ./test/utest)
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

if(MARKING_TESTS)
    add_subdirectory(marking)
endif()
