#######################################################################
# GTest setup
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

include_directories(../include)
include_directories(..)

# --- Provided tests ---

add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
target_link_libraries(testAudiReachGoals ${GTEST_LIBRARIES} pfmshog controller ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(testSkidSteerReachGoals test_skid_reach_goals.cpp)
target_link_libraries(testSkidSteerReachGoals ${GTEST_LIBRARIES} controller pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(testMissionReachGoals test_mission_reach_goals.cpp)
target_link_libraries(testMissionReachGoals ${GTEST_LIBRARIES} controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(testMissionObstruction test_mission_obstruction.cpp)
target_link_libraries(testMissionObstruction ${GTEST_LIBRARIES} ranger controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# --- Student-written tests ---

add_executable(testConstructors test_constructor.cpp)
target_link_libraries(testConstructors ${GTEST_LIBRARIES} controller pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(testInterruptResume test_interrupt_resume.cpp)
target_link_libraries(testInterruptResume ${GTEST_LIBRARIES} controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

add_executable(testObstacleDetection test_obstacle_detection.cpp)
target_link_libraries(testObstacleDetection ${GTEST_LIBRARIES} ranger controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
