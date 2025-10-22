cmake_minimum_required(VERSION 3.1)
project(trackingcam3d_reader VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(OpenCV 3 REQUIRED)
find_package(libtlive REQUIRED)
find_package(libtconfig REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIBTLIVE_INCLUDE_DIRS}
    ${LIBTCONFIG_INCLUDE_DIRS}
)

# Library target
add_library(trackingcam3d_reader SHARED
    src/TrackingCam3d_Reader.cpp
    src/TrackingCam3d_Client.cpp
    src/MyDepthIllustrator.cpp
    src/MyFpsMeter.cpp
)

target_link_libraries(trackingcam3d_reader
    ${OpenCV_LIBS}
    ${LIBTLIVE_LIBRARIES}
    ${LIBTCONFIG_LIBRARIES}
)

target_include_directories(trackingcam3d_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Build example executable (main.cpp -> trackingcam3d_reader_demo)
add_executable(trackingcam3d_reader_demo examples/main.cpp)
target_link_libraries(trackingcam3d_reader_demo trackingcam3d_reader)

# Install rules
include(GNUInstallDirs)

install(TARGETS trackingcam3d_reader
    EXPORT trackingcam3d_readerTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/trackingcam3d_reader
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export target info for find_package
install(EXPORT trackingcam3d_readerTargets
    FILE trackingcam3d_readerTargets.cmake
    NAMESPACE trackingcam3d_reader::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/trackingcam3d_reader
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/trackingcam3d_readerConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/trackingcam3d_readerConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/trackingcam3d_readerConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/trackingcam3d_reader
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/trackingcam3d_readerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/trackingcam3d_readerConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/trackingcam3d_reader
)