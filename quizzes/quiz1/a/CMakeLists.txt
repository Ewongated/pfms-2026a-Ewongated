# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz1_a)

set(BUILD_TESTS ON) #Enable all unit tests to be compiled by default

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
add_executable(quiz1_a main.cpp charger.cpp processing.cpp)

##################################################################################################
# The below are our automated tets supplied
#
# tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
##################################################################################################
