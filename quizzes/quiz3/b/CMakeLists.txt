# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz3_b)
set(BUILD_TESTS ON)

include(${CMAKE_SOURCE_DIR}/dep/FindSimulator.cmake)

add_library(analysis analysis.cpp)
target_link_libraries(analysis radar)

# Add executable
add_executable(${PROJECT_NAME}_ex main.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME}_ex analysis)

# tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
endif()