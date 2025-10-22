cmake_minimum_required(VERSION 3.1)
project(detector_blob)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(trackingcam3d_reader REQUIRED)
find_package(DxlSlave REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${trackingcam3d_reader_INCLUDE_DIRS}
    ${dxlSlave_INCLUDE_DIRS}
)

add_executable(detector_blob
    ${CMAKE_SOURCE_DIR}/src/main.cpp
)

target_link_libraries(detector_blob
    trackingcam3d_reader
    ${OpenCV_LIBS}
    DxlSlave::dxlSlave
    yaml-cpp
    pthread
)
