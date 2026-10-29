px4_add_library(arch_dshot
	dshot.c
)

# The receive API is declared in the HPM platform header.
target_include_directories(arch_dshot PRIVATE
	${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/hpmicro/hpm_common/include
)

target_compile_options(arch_dshot PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_compile_options(arch_dshot PRIVATE -DHPMSOC_HAS_HPMSDK_DMAV2)
