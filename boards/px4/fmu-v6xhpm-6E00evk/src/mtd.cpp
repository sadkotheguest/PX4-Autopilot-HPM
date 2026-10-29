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
 * @file mtd.cpp
 *
 * MTD manifest for the HPM6E00EVK.
 *
 * The board has no SD card controller and no SPI FRAM, so the only persistent
 * storage is the CAT24C128 I2C EEPROM on I2C1 (16 KiB, address 0x50).  It is
 * handed to PX4 as a single MTD device: the first part holds the parameters
 * (/fs/mtd_params), the rest is reserved for calibration data
 * (/fs/mtd_caldata).  With CONFIG_AT24XX_SIZE=128 the driver works in
 * 64-byte pages, i.e. 256 MTD blocks of 64 bytes.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <board_config.h>

#include <nuttx/spi/spi.h>
#include <px4_platform_common/px4_manifest.h>

#if defined(CONFIG_MTD)

/* CAT24C128 on I2C1: 16 KiB, 256 blocks of 64 bytes. */

static const px4_mft_device_t i2c1_eeprom = {
	.bus_type = px4_mft_device_t::I2C,
	.devid    = PX4_MK_I2C_DEVID(1, 0x50)
};

static const px4_mtd_entry_t eeprom = {
	.device = &i2c1_eeprom,
	.npart = 2,
	.partd = {
		{
			/* 144 blocks = 9 KiB: enough for the full parameter set
			 * (about 1300 four-byte records) with room to grow.
			 */
			.type = MTD_PARAMETERS,
			.path = "/fs/mtd_params",
			.nblocks = 144
		},
		{
			/* The remaining 112 blocks (7 KiB) for calibration data. */
			.type = MTD_CALDATA,
			.path = "/fs/mtd_caldata",
			.nblocks = 112
		}
	},
};

static const px4_mtd_manifest_t board_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&eeprom
	}
};

static const px4_mft_entry_s mtd_mft = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config,
};

static const px4_mft_entry_s mft_mft = {
	.type = MFT,
#ifdef BOARD_HAS_HW_SPLIT_VERSIONING
	.pmft = (void *)system_query_manifest,
#else
	.pmft = (void *)NULL,
#endif
};

static const px4_mft_s mft = {
	.nmft = 2,
	.mfts = {
		&mtd_mft,
		&mft_mft,
	}
};

const px4_mft_s *board_get_manifest(void)
{
	return &mft;
}
#endif
