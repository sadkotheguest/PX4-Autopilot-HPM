/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>
#include <nuttx/mm/mm.h>
#include <chip.h>

#include <arch/board/board.h>
#include "riscv_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_dma_alloc.h>

#include "hpm_common.h"
#include "hpm_sdram.h"
#include <nuttx/cache.h>
#ifdef CONFIG_HPM_USBDEV
#  include "hpm_usbdev.h"
#endif
#include "hpm_iomux.h"
#include "hpm_gpio_drv.h"
#include "hpm_trgm_drv.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);

extern uint8_t __vector_ram_start__[], __vector_ram_end__[], __vector_load_addr__[];
extern uint8_t __bss_start__[], __bss_end__[];
extern uint8_t __noncacheable_bss_start__[], __noncacheable_bss_end__[];
extern uint8_t __fast_ram_bss_start__[], __fast_ram_bss_end__[];
extern uint8_t __tdata_start__[], __tdata_end__[], __tdata_load_addr__[];
extern uint8_t __data_start__[], __data_end__[], __data_load_addr__[];
extern uint8_t __ramfunc_start__[], __ramfunc_end__[], __fast_load_addr__[];
extern uint8_t __axi_ramfunc_start__[], __axi_ramfunc_end__[], __axi_ramfunc_load_addr__[];
extern uint8_t __noncacheable_init_start__[], __noncacheable_init_end__[], __noncacheable_init_load_addr__[];
extern uint8_t __fast_ram_init_start__[], __fast_ram_init_end__[], __fast_ram_init_load_addr__[];
__END_DECLS

__EXPORT void c_startup(void)
{
    uint32_t i, size;

    size = __vector_ram_end__ - __vector_ram_start__;
    for (i = 0; i < size; i++) {
        *(__vector_ram_start__ + i) = *(__vector_load_addr__ + i);
    }

    /* ramfunc section */
    size = __ramfunc_end__ - __ramfunc_start__;
    for (i = 0; i < size; i++) {
        *(__ramfunc_start__ + i) = *(__fast_load_addr__ + i);
    }

     /* axi-ramfunc section */
    size = __axi_ramfunc_end__ - __axi_ramfunc_start__;
    for (i = 0; i < size; i++) {
        *(__axi_ramfunc_start__ + i) = *(__axi_ramfunc_load_addr__ + i);
    }

    /* bss section */
    size = __bss_end__ - __bss_start__;
    for (i = 0; i < size; i++) {
        *(__bss_start__ + i) = 0;
    }

    /* noncacheable bss section */
    size = __noncacheable_bss_end__ - __noncacheable_bss_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_bss_start__ + i) = 0;
    }

    /* fast_ram bss section */
    size = __fast_ram_bss_end__ - __fast_ram_bss_start__;
    for (i = 0; i < size; i++) {
        *(__fast_ram_bss_start__ + i) = 0;
    }

    /* data section */
    size = __data_end__ - __data_start__;
    for (i = 0; i < size; i++) {
        *(__data_start__ + i) = *(__data_load_addr__ + i);
    }

    /* tdata section */
    size = __tdata_end__ - __tdata_start__;
    for (i = 0; i < size; i++) {
        *(__tdata_start__ + i) = *(__tdata_load_addr__ + i);
    }

    /* noncacheable init section */
    size = __noncacheable_init_end__ - __noncacheable_init_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_init_start__ + i) = *(__noncacheable_init_load_addr__ + i);
    }

    /* fast_ram init section */
    size = __fast_ram_init_end__ - __fast_ram_init_start__;
    for (i = 0; i < size; i++) {
        *(__fast_ram_init_start__ + i) = *(__fast_ram_init_load_addr__ + i);
    }
}

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{

}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{

}

/************************************************************************************
 * Name: hpm_boardinitialize
 *
 * Description:
 *   All hpm architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
hpm_boardinitialize(void)
{

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 ****************************************************************************/

/* I2C: the bus initialisers come from the NuttX HPM I2C master driver, and
 * the board pads are configured by hpm_i2cbus_pins_init() in i2c.cpp.
 * SPI is not ported to HPM6E00 yet - see bringup_stubs.cpp.
 */

/* External SDRAM: the FEMC is programmed from __hpm_start() (see
 * hpm_sdram_preinit()), before anything can touch the window.  Here the memory
 * is verified and handed to the heap.
 */

/* Start of the non-cacheable DMA window inside the external SDRAM (linker script). */
extern char __sdram_nc_start__[];

static int board_sdram_selftest(uintptr_t base, size_t size)
{
	/* Write the pattern, push it out of the cache and drop it, so the read
	 * below has to come from the SDRAM itself.  Turning the data cache off
	 * for the duration is not an option: up_disable_dcache() only gates the
	 * cache, it does not flush, so the RTOS state (IRQ table included) would
	 * be read stale while the test runs.
	 */

	volatile uint32_t *mem = (volatile uint32_t *)base;
	const uint32_t count = size / 4u;
	int ret = OK;

	for (uint32_t i = 0; i < count; i++) {
		mem[i] = (i << 24) | ((i + 1) << 16) | ((i + 2) << 8) | (i + 3);
	}

	up_clean_dcache(base, base + size);
	up_invalidate_dcache(base, base + size);

	for (uint32_t i = 0; i < count; i++) {
		const uint32_t expect = (i << 24) | ((i + 1) << 16) | ((i + 2) << 8) | (i + 3);

		if (mem[i] != expect) {
			ret = -EIO;
			break;
		}
	}

	return ret;
}

 __EXPORT int board_app_initialize(uintptr_t arg)
{
	printf("\n\nHPMicro PX4\n");

	/* External SDRAM: the FEMC was programmed from __hpm_start() before the
	 * RTOS came up, so the window is live now.  Verify it and hand it to the
	 * heap.  The self test covers the first megabyte only - a full 32 MB pass
	 * would add seconds to every boot, and a part that fails early fails
	 * everywhere.
	 */

	if (hpm_sdram_size() < 0) {
		syslog(LOG_ERR, "[boot] SDRAM init failed\n");

	} else if (board_sdram_selftest(HPM_SDRAM_BASE, 1024 * 1024) < 0) {
		syslog(LOG_ERR, "[boot] SDRAM self test failed (expansion board not fitted?)\n");

	} else {
		/* The tail of the SDRAM is the non-cacheable DMA window (region
		 * SDRAM_NC in the linker script), so only the part in front of it
		 * becomes heap.
		 */

		const uintptr_t nc_start = (uintptr_t)__sdram_nc_start__;
		const size_t heap_size = (nc_start > HPM_SDRAM_BASE) ?
					 (nc_start - HPM_SDRAM_BASE) : HPM_SDRAM_SIZE;

		syslog(LOG_INFO, "[boot] SDRAM: %u MB at 0x%08x added to the heap\n",
		       (unsigned)(heap_size >> 20), (unsigned)HPM_SDRAM_BASE);
		umm_addregion((void *)HPM_SDRAM_BASE, heap_size);
	}

	px4_arch_configgpio(GPIO_LED_SAFETY);
	px4_arch_configgpio(GPIO_BTN_SAFETY);

	px4_platform_init();

#ifdef CONFIG_HPM_USBDEV
	hpm_usbdev_initialize(CONFIG_HPM_USBDEV_INSTANCE);
#endif

	/* SPI/I2C buses */
	hpm_spiinitialize();

	px4_platform_configure();

	/* Configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* initial LED state */
	// drv_led_start();

	// led_off(LED_RED);
	// led_off(LED_GREEN);
	// led_off(LED_BLUE);
#if defined(CONFIG_HPM_SDXC0) || defined(CONFIG_HPM_SDXC1)
	hpm_sdioinitialize();
#endif
	return 0;
}

void rc_input_invert(bool invert)
{
    // trgm_output_t config = { 0 };

    // HPM_IOC->PAD[IOC_PAD_PD24].FUNC_CTL = IOC_PD24_FUNC_CTL_TRGM2_P_10;
    // HPM_IOC->PAD[IOC_PAD_PD25].FUNC_CTL = IOC_PD25_FUNC_CTL_TRGM2_P_08;

    // config.invert = true;
    // config.input = HPM_TRGM2_INPUT_SRC_TRGM2_P10;
    // trgm_output_config(HPM_TRGM2, HPM_TRGM2_OUTPUT_SRC_TRGM2_P8, &config);
    // trgm_enable_io_output(HPM_TRGM2, 1 << 8);
}

