cmake_minimum_required(VERSION 3.1)
project(DxlSlave VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build static library
add_library(dxlSlave STATIC src/dxlSlave.cpp)
target_include_directories(dxlSlave PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# Build example executable (dxlSlaveTest.cpp -> dxlSlaveTest)
add_executable(dxlSlaveTest examples/dxlSlaveTest.cpp)
target_link_libraries(dxlSlaveTest dxlSlave)

# Install library and header
install(TARGETS dxlSlave
        EXPORT DxlSlaveTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(FILES src/dxlSlave.h DESTINATION include)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/DxlSlaveConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/DxlSlaveConfig.cmake
    INSTALL_DESTINATION lib/cmake/DxlSlave
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/DxlSlaveConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(EXPORT DxlSlaveTargets
        FILE DxlSlaveTargets.cmake
        NAMESPACE DxlSlave::
        DESTINATION lib/cmake/DxlSlave)
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/DxlSlaveConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/DxlSlaveConfigVersion.cmake
        DESTINATION lib/cmake/DxlSlave)
