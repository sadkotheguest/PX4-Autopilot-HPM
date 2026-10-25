# HPM6E80 platform layer (minimal bring-up: UART/GPIO/GPTMR)
# - io_pins (PWMv2) and spi are not compatible with the 6E00 IP yet
# - hpm_csr_regs.h from the SDK redefines NuttX arch/csr.h macros, so the
#   platform layer is built without -Werror
add_compile_options(-Wno-error)

add_subdirectory(../hpm_common/board_reset board_reset)
add_subdirectory(../hpm_common/hrt hrt)
add_subdirectory(../hpm_common/board_critmon board_critmon)
add_subdirectory(../hpm_common/romapi romapi)
add_subdirectory(../hpm_common/version version)
