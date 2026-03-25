# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.5)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (ex01_starter)

# Add the executable which showcases convars
add_executable(ex01_convar main_convar.cpp)

# Add the executable which showcases futures/promises
add_executable(ex01_futures main_futures.cpp)
