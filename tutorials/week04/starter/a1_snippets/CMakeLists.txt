# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (a1_snippets LANGUAGES CXX VERSION 3.0.1)

# This is added to c++171 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Include the location of pfms headers
find_package(pfms2ros 3.1.0 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# Add executable and then link it with teh requireed libraries
add_executable(command_ackerman command_ackerman.cpp)
# Link the library to the executable
target_link_libraries(command_ackerman pfmsconnector)

add_executable(command_skidsteer command_skidsteer.cpp)
target_link_libraries(command_skidsteer pfmsconnector)

add_executable(receive_laser receive_laser.cpp)
target_link_libraries(receive_laser pfmsconnector)

add_executable(receive_sonar receive_sonar.cpp)
target_link_libraries(receive_sonar pfmsconnector)

add_executable(visualise_cells visualise_cells.cpp)
target_link_libraries(visualise_cells pfmsconnector cell)

# This executable needs pfmshog which then needs to be linked with few gazebo libraries 
add_executable(move_items move_items.cpp)
target_link_libraries(move_items pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
