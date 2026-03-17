# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (shapes_ex)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Let's create a library
# To do so we need add_library command 
# Replace <library_name> with the name of library (when the library is created it will be lib<library_name>.so)
# Replace <cpp_files> with the files that form part of library
# add_library(<library_name> SHARED <cpp_files>)
add_library(shapes SHARED circle.cpp shape.cpp rectangle.cpp triangle.cpp)

# We have another library
add_library(shapeprocessing SHARED shapeprocessing.cpp)
# This library depends on shapes library 
target_link_libraries(shapeprocessing shapes)

# We can now also create an executable, though this is not shared with others (just the library)
add_executable(shapes_ex main.cpp)

# We now have a library and an executable, we need to link each other
target_link_libraries(shapes_ex shapeprocessing shapes)

#################################################################
# UNIT_TESTING SECTION
# We add the test directory, which has it's own CMakeLists.txt
add_subdirectory(test)

