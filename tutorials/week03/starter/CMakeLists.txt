# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (shapes)

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable

# This executable demonstrates the declaration, implementation, and use of the circle and rectangle class from a shape 
add_executable(shapes_eg main.cpp circle.cpp shape.cpp)
