/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * SPI bus configuration for the HPM6E00EVK (HPM6E80).
 *
 * The HPM6E00EVK routes its SPI header to SPI7:
 *   PF26 = SCLK, PF27 = CS0, PF28 = MISO, PF29 = MOSI (alternate function 5).
 *
 * The pads are described by the GPIO_SPI7_* macros in board_config.h and are
 * configured from hpm_spibus_pins_init(), which the NuttX HPM SPI master driver
 * calls from hpm_spibus_initialize() - the same way the HPM6750 board does it.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI)

/* hpm_spibus_pins_init
 * Called from hpm_spi_master.c
 */
void hpm_spibus_pins_init(int bus)
{
	switch (bus) {
	case 0:
#if defined(CONFIG_HPM_SPI0)
		px4_arch_configgpio(GPIO_SPI0_SCK);
		px4_arch_configgpio(GPIO_SPI0_MISO);
		px4_arch_configgpio(GPIO_SPI0_MOSI);
#endif
		break;

	case 7:
#if defined(CONFIG_HPM_SPI7)
		px4_arch_configgpio(GPIO_SPI7_SCK);
		px4_arch_configgpio(GPIO_SPI7_MISO);
		px4_arch_configgpio(GPIO_SPI7_MOSI);
#endif
		break;

	default:
		break;
	}
}

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	/* SPI0 is used as the slave side of the SPI7 <-> SPI0 loopback test; it is
	 * listed here so that hpm_spiinitialize() finds the bus, no devices attached.
	 */
	initSPIBus(SPI::Bus::SPI0, {
	}),

	initSPIBus(SPI::Bus::SPI7, {
	}),
};

// static constexpr bool unused = validateSPIConfig(px4_spi_buses);

#endif /* CONFIG_SPI */
