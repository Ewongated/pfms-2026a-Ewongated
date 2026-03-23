# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (simple_threading)

find_package (Threads)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

# CMake will now create the Makefiles that are needed to make an executable "hello" that requires "hello.cpp" 
add_executable(ex01 main.cpp analysis.cpp)

