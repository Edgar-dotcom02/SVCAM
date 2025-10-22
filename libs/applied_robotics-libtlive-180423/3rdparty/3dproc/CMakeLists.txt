find_package(OpenCV 3 REQUIRED)

add_library(tlive-3dproc MyPerformanceMeter.cpp )

install(TARGETS tlive-3dproc EXPORT libtlive-targets DESTINATION ${LIB_DESTINATION} ARCHIVE DESTINATION lib)
