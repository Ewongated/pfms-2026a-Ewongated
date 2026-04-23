# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.5)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (convar_starter)

# CMake will now create the Makefiles that are needed to make an executable "test" that requires "main.cpp"
add_executable(example main.cpp sensor.cpp analysis.cpp)
