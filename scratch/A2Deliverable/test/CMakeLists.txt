#######################################################################
# Enable gtests — this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

# Allow headers in a separate include directory if present
include_directories(../include)
include_directories(..)

# ── Ackerman reach goals (provided) ─────────────────────────────────────────
add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
target_link_libraries(testAudiReachGoals
    ${GTEST_LIBRARIES} pfmshog controller
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testAudiReachGoals COMMAND testAudiReachGoals)

# ── Skidsteer reach goals (provided) ────────────────────────────────────────
add_executable(testSkidSteerReachGoals test_skid_reach_goals.cpp)
target_link_libraries(testSkidSteerReachGoals
    ${GTEST_LIBRARIES} controller pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testSkidSteerReachGoals COMMAND testSkidSteerReachGoals)

# ── Mission reach goals — BASIC mode (provided) ──────────────────────────────
add_executable(testMissionReachGoals test_mission_reach_goals.cpp)
target_link_libraries(testMissionReachGoals
    ${GTEST_LIBRARIES} controller mission pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testMissionReachGoals COMMAND testMissionReachGoals)

# ── Mission obstruction — Ackerman ADVANCED mode (provided) ─────────────────
add_executable(testMissionObstruction test_mission_obstruction.cpp)
target_link_libraries(testMissionObstruction
    ${GTEST_LIBRARIES} ranger controller mission pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testMissionObstruction COMMAND testMissionObstruction)

# ── Constructor tests — Ackerman and Skidsteer (student) ────────────────────
add_executable(testConstructors test_constructors.cpp)
target_link_libraries(testConstructors
    ${GTEST_LIBRARIES} controller pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testConstructors COMMAND testConstructors)

# ── Ackerman mission interrupt/resume (student) ──────────────────────────────
add_executable(testAckermanMission test_ackerman_mission.cpp)
target_link_libraries(testAckermanMission
    ${GTEST_LIBRARIES} controller mission pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testAckermanMission COMMAND testAckermanMission)

# ── Skidsteer mission interrupt/resume (student) ─────────────────────────────
add_executable(testSkidsteerMission test_skidsteer_mission.cpp)
target_link_libraries(testSkidsteerMission
    ${GTEST_LIBRARIES} controller mission pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testSkidsteerMission COMMAND testSkidsteerMission)

# ── Skidsteer obstruction — ADVANCED mode (student) ──────────────────────────
add_executable(testSkidObstruction test_skidsteer_obstruction.cpp)
target_link_libraries(testSkidObstruction
    ${GTEST_LIBRARIES} ranger controller mission pfmshog
    ignition-msgs8::ignition-msgs8 ignition-transport11::ignition-transport11)
add_test(NAME testSkidObstruction COMMAND testSkidObstruction)
