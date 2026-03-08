#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

add_executable(sensorTests test_sensors.cpp)
target_link_libraries(sensorTests ${GTEST_LIBRARIES} pthread)
target_link_libraries(sensorTests ranger cell pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)


add_executable(rawTests test_rawdata.cpp)
target_link_libraries(rawTests ${GTEST_LIBRARIES} pthread)
target_link_libraries(rawTests ranger cell pfmshog fusion ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(fusionTests test_fusion.cpp)
target_link_libraries(fusionTests ${GTEST_LIBRARIES} pthread)
target_link_libraries(fusionTests ranger cell pfmshog fusion ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(fusionTestsMulti test_fusion_multi.cpp)
target_link_libraries(fusionTestsMulti ${GTEST_LIBRARIES} pthread)
target_link_libraries(fusionTestsMulti ranger cell pfmshog fusion ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
