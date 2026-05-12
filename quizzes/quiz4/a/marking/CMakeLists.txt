#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
#message(STATUS "The include for simulator is ${SIMULATOR_INCLUDE}")
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

include_directories( ${CMAKE_SOURCE_DIR})  #This enabled us to include files that are part of project

# Add the unit test executable against the source code of the test
add_executable(selfTest utest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(selfTest GTest::GTest GTest::Main pfmshog controllers pfmsconnector ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
# We can finally add the test to a list of tests to be run with make test
add_test(Q4A1 selfTest)

# Add the unit test executable against the source code of the test
add_executable(markingTest utest2.cpp)
# Linking to gtest and to your libraries
target_link_libraries(markingTest GTest::GTest GTest::Main pfmshog controllers pfmsconnector ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
# We can finally add the test to a list of tests to be run with make test
add_test(Q4A1 markingTest)

