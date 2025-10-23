add_library(tlive-zstd cover.c
divsufsort.c
entropy_common.c
error_private.c
fse_compress.c
fse_decompress.c
huf_compress.c
huf_decompress.c
pool.c
threading.c
xxhash.c
zbuff_common.c
zbuff_compress.c
zbuff_decompress.c
zdict.c
zstd_common.c
zstd_compress.c
zstd_decompress.c
zstd_double_fast.c
zstd_fast.c
zstd_lazy.c
zstd_ldm.c
zstdmt_compress.c
zstd_opt.c
zstd_v01.c
zstd_v02.c
zstd_v03.c
zstd_v04.c
zstd_v05.c
zstd_v06.c
zstd_v07.c)

install(TARGETS tlive-zstd EXPORT libtlive-targets DESTINATION ${LIB_DESTINATION} ARCHIVE DESTINATION lib)
