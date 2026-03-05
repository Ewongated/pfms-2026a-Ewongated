cmake_minimum_required (VERSION 3.1)

project(quiz2_b)
set(BUILD_TESTS ON)

add_executable(${PROJECT_NAME}_ex main.cpp container_ops.cpp)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()
