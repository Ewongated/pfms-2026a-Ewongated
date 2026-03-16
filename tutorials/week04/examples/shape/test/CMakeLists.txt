#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(..) # This enables us to use the header files from the folder below directly
enable_testing()
###################################

add_executable(utest utest.cpp)
add_executable(utest2 utest2.cpp)
# Linking to gtest and to your libraries
target_link_libraries(utest GTest::GTest GTest::Main shapes)
target_link_libraries(utest2 GTest::GTest GTest::Main shapes shapeprocessing)
# We can finally add the test to a list of tests to be run with make test
add_test(shapes_test utest)
add_test(shapeprocessing_test utest2)
