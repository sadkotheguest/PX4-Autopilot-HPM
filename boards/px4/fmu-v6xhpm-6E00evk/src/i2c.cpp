/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file i2c.cpp
 *
 * I2C bus configuration for the HPM6E00EVK (HPM6E80).
 *
 * The NuttX HPM I2C master driver (hpm_i2c_master.c) calls the board hook
 * hpm_i2cbus_pins_init() and board_init_i2c_clock() from
 * hpm_i2cbus_initialize(), so only the pads have to be described here.
 *
 * HPM6E00EVK: the I2C device on the board is a CAT24C128 EEPROM (16 KiB,
 * address 0x50) wired to I2C1 on PA12 (SDA) / PA13 (SCL).  The HPM SDK ships
 * board_init_i2c_eeprom_pin() for exactly that pad pair; it is not declared
 * in the SDK board.h, so it is declared here.
 */

#include <px4_arch/i2c_hw_description.h>

#if defined(CONFIG_I2C)

extern "C" void board_init_i2c_eeprom_pin(void);

/****************************************************************************
 * Name: hpm_i2cbus_pins_init
 *
 * Description:
 *   Configure the pads of the selected I2C port. Called by the NuttX HPM
 *   I2C master driver when the bus is opened the first time.
 *
 * Input Parameters:
 *   port - I2C port number
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

extern "C" int hpm_i2cbus_pins_init(int port)
{
	switch (port) {
	case 1:
		/* I2C1: PA12 = SDA, PA13 = SCL (CAT24C128 EEPROM) */
		board_init_i2c_eeprom_pin();
		return 0;

	default:
		return -EINVAL;
	}
}

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusInternal(1),
};

#endif /* CONFIG_I2C */
