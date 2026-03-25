#################################################################
# The below finds version of library based on your system
execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if("${RELEASE_CODENAME}" STREQUAL "jammy")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
    set (RELEASE_CODENAME "focal") #Did this to use the focal version of the library on jammy
else("${RELEASE_CODENAME}" STREQUAL "jammy")
    message ( FATAL_ERROR "Your vesrion on Ubuntu ${RELEASE_CODENAME} is not supported" )
endif("${RELEASE_CODENAME}" STREQUAL "jammy")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
    set (RELEASE_CODENAME "arm64")
endif()

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(RADAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${RADAR_LIB_HOME}/include)
link_directories(${RADAR_LIB_HOME}/lib/${RELEASE_CODENAME})
###################################################################