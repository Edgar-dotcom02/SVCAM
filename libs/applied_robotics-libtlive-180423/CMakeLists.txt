cmake_minimum_required( VERSION 2.8.0 FATAL_ERROR )

PROJECT(libtlive)

SET( V_MAJOR 1 )
SET( V_MINOR 0 )
SET( V_BUGFIX 0 )
SET( VERSION ${V_MAJOR}.${V_MINOR}.${V_BUGFIX} )
SET( APIVERSION ${V_MAJOR}.${V_MINOR} )

set(PROJECT_LIB_DIR "lib")
set(PROJECT_OUTPUT_LIB "tlive")
set(INSTALL_CMAKE_DIR ${PROJECT_LIB_DIR}/cmake/${PROJECT_NAME})
set(LIBTLIVE_SOURCE_DIR ${PROJECT_SOURCE_DIR})
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${PROJECT_SOURCE_DIR}/3rdparty/lz4)

find_package(TBB REQUIRED)
find_package(OpenCV 3.1 REQUIRED)

#set(CMAKE_CXX_FLAGS "-fPIC")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/FindZeroMQ.cmake)
include_directories(${ZeroMQ_INCLUDE_DIR})
include_directories(${TBB_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(3rdparty/lz4 3rdparty/zstd 3rdparty/3dproc)
include_directories(include)
include_directories(include/tlive)
include_directories("include/tlive")
add_subdirectory(src)
add_subdirectory(3rdparty/3dproc)
add_subdirectory(3rdparty/lz4)
add_subdirectory(3rdparty/zstd)

option(BUILD_LIBTLIVE_EXAMPLES "Build examples for libtlive" OFF)

option (WITH_NEON "Enables neon optimization" OFF)
if(WITH_NEON)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfpu=neon")
endif(WITH_NEON)

if(BUILD_LIBTLIVE_EXAMPLES)
	add_subdirectory(examples)
endif(BUILD_LIBTLIVE_EXAMPLES)

set(PROJECT_OUTPUT_LIB_FULLNAME "$<TARGET_FILE_NAME:tlive>")
message(STATUS "Will build library=${PROJECT_OUTPUT_LIB_FULLNAME}")

# Generating cmake config, to be used in other libraries
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/libtlive-config.cmake.in 
	${CMAKE_CURRENT_BINARY_DIR}/libtlive-config.cmake @ONLY)
	
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/libtlive-config.cmake.in 
	${CMAKE_CURRENT_BINARY_DIR}/libtlive-config.local @ONLY)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/libtlive-config-version.cmake.in 
	${CMAKE_CURRENT_BINARY_DIR}/libtlive-config-version.cmake @ONLY)

# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_OUTPUT_LIB} tlive-lz4 tlive-zstd tlive-3dproc FILE "${PROJECT_BINARY_DIR}/libtlive-targets.cmake")
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE LIBTLIVE)

install(FILES 
	${CMAKE_CURRENT_BINARY_DIR}/libtlive-config.cmake 
	${CMAKE_CURRENT_BINARY_DIR}/libtlive-config-version.cmake 
	DESTINATION ${INSTALL_CMAKE_DIR})

install(EXPORT libtlive-targets DESTINATION "${INSTALL_CMAKE_DIR}")
install(DIRECTORY include/tlive DESTINATION include)
