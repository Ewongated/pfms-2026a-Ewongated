# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(CAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${CAR_LIB_HOME}/include)

# In order to support arm64 we now need to have a seperate library for this platform
# Find the library and files belonging to Car 
message ( STATUS "Looking for libray in ${CAR_LIB_HOME}" )
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if( ${ARCHITECTURE} STREQUAL "aarch64" )
  link_directories(${CAR_LIB_HOME}/libaarch64)
else()
  link_directories(${CAR_LIB_HOME}/lib)
endif()