find_package( OpenCV REQUIRED )
if(OpenCV_VERSION_MAJOR LESS 4)
    message ( STATUS "OpenCV version 3 " )
    set (RELEASE_CODENAME "opencv3")
else()
    message ( STATUS "OpenCV version 4 " )
    set (RELEASE_CODENAME "opencv4")
endif()

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
    set (RELEASE_CODENAME "opencv4_arm64")
endif()

# Finding and linking against the simulator library
set(SIMULATOR_LIB ${CMAKE_SOURCE_DIR}/dep/lib/${RELEASE_CODENAME})
set(SIMULATOR_INCLUDE ${CMAKE_SOURCE_DIR}/dep/include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SIMULATOR_INCLUDE}
)

message ( STATUS "Looking for libray in ${SIMULATOR_LIB}" )
link_directories(${SIMULATOR_LIB})