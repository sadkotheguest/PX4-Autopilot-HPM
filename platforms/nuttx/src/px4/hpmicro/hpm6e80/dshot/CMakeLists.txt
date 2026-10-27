px4_add_library(arch_dshot
	dshot.c
)

target_compile_options(arch_dshot PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_compile_options(arch_dshot PRIVATE -DHPMSOC_HAS_HPMSDK_DMAV2)
