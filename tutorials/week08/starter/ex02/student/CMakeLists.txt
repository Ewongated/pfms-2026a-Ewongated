#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

include_directories(
 ${CMAKE_SOURCE_DIR}  #This enabled us to include files that are part of project
)

# Add the unit test executable against the source code of the test
add_executable(testConstructors test_constructors.cpp)
# Linking to gtest and to your libraries
target_link_libraries(testConstructors GTest::GTest GTest::Main pfmsconnector controller)
# We can finally add the test to a list of tests to be run with make test
add_test(EX1 testConstructors)

# There should be equivalence between
# GTest::GTest GTest::Main
# AND
# ${GTEST_LIBRARIES}

# Add the unit test executable against the source code of the test
add_executable(testAudi test_audi.cpp)
# Linking to gtest and to your libraries
target_link_libraries(testAudi GTest::GTest GTest::Main pthread pfmsconnector pfmshog controller ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
# We can finally add the test to a list of tests to be run with make test
add_test(EX2 testAudi)

# Add the unit test executable against the source code of the test
add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
# Linking to gtest and to your libraries
target_link_libraries(testAudiReachGoals ${GTEST_LIBRARIES} pthread pfmsconnector pfmshog controller ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
# We can finally add the test to a list of tests to be run with make test
add_test(EX3 testAudiReachGoals)

# Add the unit test executable against the source code of the test
add_executable(testControllerInterface test_controller_interface.cpp)
# Linking to gtest and to your libraries
target_link_libraries(testControllerInterface ${GTEST_LIBRARIES} pthread pfmsconnector pfmshog controller ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
# We can finally add the test to a list of tests to be run with make test
add_test(EX4 testControllerInterface)


