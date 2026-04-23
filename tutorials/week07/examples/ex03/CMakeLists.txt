# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.5)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (week07_ex03)

# We make a single executable, but could have made at least one library (or used a radar library)
add_executable(radar_processing main.cpp radar.cpp dataprocessing.cpp)
