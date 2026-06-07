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

# ACKERMAN TEST
add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
target_link_libraries(testAudiReachGoals ${GTEST_LIBRARIES} pfmshog controller ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# QUADCOPTER TEST
add_executable(testSkidSteerReachGoals test_skid_reach_goals.cpp)
target_link_libraries(testSkidSteerReachGoals ${GTEST_LIBRARIES} controller pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# MISSION TEST
add_executable(testMissionReachGoals test_mission_reach_goals.cpp)
target_link_libraries(testMissionReachGoals ${GTEST_LIBRARIES} controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# # TSP Ackerman
add_executable(testMissionObstruction test_mission_obstruction.cpp)
target_link_libraries(testMissionObstruction ${GTEST_LIBRARIES} ranger controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# Constructor Test
add_executable(testConstructors test_constructors.cpp)
target_link_libraries(testConstructors ${GTEST_LIBRARIES} controller pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# Ackerman Mission Interrupt/Resume Test
add_executable(testAckermanMission test_ackerman_mission.cpp)
target_link_libraries(testAckermanMission ${GTEST_LIBRARIES} controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# Skidsteer Mission Interrupt/Resume Test
add_executable(testSkidsteerMission test_skidsteer_mission.cpp)
target_link_libraries(testSkidsteerMission ${GTEST_LIBRARIES} controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# Student Obstruction Test — Skidsteer obstructed
add_executable(testSkidObstruction test_skidsteer_obstruction.cpp)
target_link_libraries(testSkidObstruction ${GTEST_LIBRARIES} ranger controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

# # TSP Quadcopter
# add_executable(testTspReachGoalsQuad tsp_reach_goals_quad.cpp)
# target_link_libraries(testTspReachGoalsQuad ${GTEST_LIBRARIES} controller mission pfmshog ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)

