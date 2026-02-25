# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (quiz1_b)

set(BUILD_TESTS ON) #Enable all unit tests to be compiled by default

# CMake will now create the Makefiles that are needed to make an executable "hello" that requires "hello.cpp" 
add_executable(quiz1_b main.cpp shape.cpp rectangle.cpp)

##################################################################################################
# The below are our automated tets supplied
#
# tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
##################################################################################################