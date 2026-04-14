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

add_executable(testConstructors test_constructors.cpp)
target_link_libraries(testConstructors ${GTEST_LIBRARIES} pfmsconnector ranger pthread)

add_executable(testRangerInterface test_ranger_interface.cpp)
target_link_libraries(testRangerInterface ${GTEST_LIBRARIES} pfmshog ranger pthread ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(testFusionInterface test_fusion_interface.cpp)
target_link_libraries(testFusionInterface ${GTEST_LIBRARIES} ranger pthread fusion cell)