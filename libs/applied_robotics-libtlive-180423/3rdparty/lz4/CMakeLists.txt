add_library(tlive-lz4 STATIC lz4.c lz4frame.c lz4hc.c xxhash.c)

install(TARGETS tlive-lz4 EXPORT libtlive-targets DESTINATION ${LIB_DESTINATION} ARCHIVE DESTINATION lib)
