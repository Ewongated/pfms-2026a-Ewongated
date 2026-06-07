# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz3_a)

set(BUILD_TESTS ON) # This switch here sets the tests on automatically
set(MARKING_TESTS ON) # You can enable this switch to run the marking tests after the deadline and when they are desposited

include(${CMAKE_SOURCE_DIR}/dep/FindSimulator.cmake)

# The tf2 library students are developing
add_library(tf2 tf2.cpp)
target_link_libraries(tf2 simulator )
# The analysis library students are developing
add_library(analysis analysis.cpp)
target_link_libraries(analysis simulator )

# A sample executable
add_executable(${PROJECT_NAME}_ex main.cpp)
target_link_libraries( ${PROJECT_NAME}_ex analysis tf2 simulator ${OpenCV_LIBS})

##################################################################################################
# UNIT TESTS
##################################################################################################
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

# Marking tests are deposited after deadline (you will be notified via teams), this directory does not exists yet
if(MARKING_TESTS)
    add_subdirectory(marking)
endif()