# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz2_a)

set(BUILD_TESTS ON) # Enables building tests by default
set(MARKING_TESTS OFF) # This is off by default, it can be turned ON after the deadline whne marking files are provided

# Find OpenCV to draw on screen 
find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# Include the CMake file for finding Car library
include(${CMAKE_SOURCE_DIR}/dep/FindCarLib.cmake)

# Library to visualise race
add_library(displayrace display_race.cpp)
target_link_libraries( displayrace ${OpenCV_LIBS})

# The library students are developing, the one that handles racing
add_library(analysis analysis.cpp)
target_link_libraries(analysis car displayrace)

# A sample executable
add_executable(racing_ex main.cpp)
target_link_libraries( racing_ex ${OpenCV_LIBS} car analysis displayrace)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()

# Marking tests are deposited after deadline, this directory does not exists yet
if(MARKING_TESTS)
    add_subdirectory(marking)
endif()