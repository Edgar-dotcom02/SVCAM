add_library(${PROJECT_OUTPUT_LIB} SHARED 	
	tconfig_param.cpp)

install(TARGETS ${PROJECT_OUTPUT_LIB} EXPORT libtconfig-targets RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
