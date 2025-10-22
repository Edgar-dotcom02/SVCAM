cmake_minimum_required( VERSION 2.8.0 FATAL_ERROR )

PROJECT(libtconfig)

SET( V_MAJOR 1 )
SET( V_MINOR 0 )
SET( V_BUGFIX 0 )
SET( VERSION ${V_MAJOR}.${V_MINOR}.${V_BUGFIX} )
SET( APIVERSION ${V_MAJOR}.${V_MINOR} )

set(PROJECT_LIB_DIR "lib")
set(PROJECT_OUTPUT_LIB "tconfig")
set(INSTALL_CMAKE_DIR ${PROJECT_LIB_DIR}/cmake/${PROJECT_NAME})
set(LIBTCONFIG_SOURCE_DIR ${PROJECT_SOURCE_DIR})
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(include/tconfig)
include_directories("include/tconfig")
add_subdirectory(src)

option(BUILD_LIBTCONFIG_EXAMPLES "Build examples for libtconfig" OFF)

if(BUILD_LIBTCONFIG_EXAMPLES)
	add_subdirectory(examples)
endif(BUILD_LIBTCONFIG_EXAMPLES)

set(PROJECT_OUTPUT_LIB_FULLNAME "$<TARGET_FILE_NAME:tconfig>")
message(STATUS "Will build library=${PROJECT_OUTPUT_LIB_FULLNAME}")

# Generating cmake config, to be used in other libraries
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/libtconfig-config.cmake.in 
	${CMAKE_CURRENT_BINARY_DIR}/libtconfig-config.cmake @ONLY)
	
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/libtconfig-config.cmake.in 
	${CMAKE_CURRENT_BINARY_DIR}/libtconfig-config.local @ONLY)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/libtconfig-config-version.cmake.in 
	${CMAKE_CURRENT_BINARY_DIR}/libtconfig-config-version.cmake @ONLY)

# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_OUTPUT_LIB} FILE "${PROJECT_BINARY_DIR}/libtconfig-targets.cmake")
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE LIBTCONFIG)

install(FILES 
	${CMAKE_CURRENT_BINARY_DIR}/libtconfig-config.cmake 
	${CMAKE_CURRENT_BINARY_DIR}/libtconfig-config-version.cmake 
	DESTINATION ${INSTALL_CMAKE_DIR})

install(EXPORT libtconfig-targets DESTINATION "${INSTALL_CMAKE_DIR}")
install(DIRECTORY include/tconfig DESTINATION include)
