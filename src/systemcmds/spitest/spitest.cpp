/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file spitest.cpp
 *
 * SPI master/slave loopback test for the HPM6E00EVK.
 *
 * SPI7 is the master and SPI0 the slave; the pads are wired on the board:
 *
 *   SCLK  PF26 -> PE10
 *   MOSI  PF29 -> PE13
 *   MISO  PF28 -> PE12
 *   CS    PF25 -> PE11
 *
 * The slave side is exposed by the NuttX SPI slave character driver as
 * /dev/spislv0 (registered from board_app_initialize()), the master side is
 * the regular NuttX SPI interface of the HPM SPI master driver.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <nuttx/spi/spi.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

extern "C" struct spi_dev_s *hpm_spibus_initialize(int bus);

#define TEST_LEN 16

static void dump(const char *name, const uint8_t *data, int len)
{
	char line[3 * TEST_LEN + 1] = {};
	int pos = 0;

	for (int i = 0; i < len && pos < (int)sizeof(line) - 3; i++) {
		static const char hex[] = "0123456789abcdef";
		line[pos++] = hex[data[i] >> 4];
		line[pos++] = hex[data[i] & 0x0f];
		line[pos++] = ' ';
	}

	PX4_INFO("%s: %s", name, line);
}

extern "C" int spitest_main(int argc, char *argv[])
{
	uint8_t slave_tx[TEST_LEN];
	uint8_t slave_rx[TEST_LEN];
	uint8_t master_tx[TEST_LEN];
	uint8_t master_rx[TEST_LEN];

	for (int i = 0; i < TEST_LEN; i++) {
		slave_tx[i] = 0xA0 + i;
		master_tx[i] = 0x50 + i;
	}

	memset(slave_rx, 0, sizeof(slave_rx));
	memset(master_rx, 0, sizeof(master_rx));

	int fd = open("/dev/spislv0", O_RDWR);

	if (fd < 0) {
		PX4_ERR("open /dev/spislv0 failed: %d", errno);
		return 1;
	}

	/* Slave pads (SPI0, AF5) and the master chip select (plain GPIO) */

	px4_arch_configgpio(GPIO_SPI0_SCK);
	px4_arch_configgpio(GPIO_SPI0_CS);
	px4_arch_configgpio(GPIO_SPI0_MISO);
	px4_arch_configgpio(GPIO_SPI0_MOSI);
	px4_arch_configgpio(GPIO_SPI7_CS);
	px4_arch_gpiowrite(GPIO_SPI7_CS, 1);

	/* Master side: SPI7 */

	struct spi_dev_s *dev = hpm_spibus_initialize(7);

	if (dev == nullptr) {
		PX4_ERR("SPI7 master initialisation failed");
		close(fd);
		return 1;
	}

	SPI_SETMODE(dev, SPIDEV_MODE0);
	SPI_SETBITS(dev, 8);
	SPI_SETFREQUENCY(dev, 1000000);

	/* Arm the slave TX queue, then run one transfer */

	ssize_t enqueued = write(fd, slave_tx, TEST_LEN);
	PX4_INFO("slave enqueued %d of %d bytes", (int)enqueued, TEST_LEN);

	px4_arch_gpiowrite(GPIO_SPI7_CS, 0);
	SPI_EXCHANGE(dev, master_tx, master_rx, TEST_LEN);
	px4_arch_gpiowrite(GPIO_SPI7_CS, 1);

	ssize_t received = read(fd, slave_rx, TEST_LEN);
	PX4_INFO("slave received %d bytes", (int)received);
	close(fd);

	dump("master rx", master_rx, TEST_LEN);
	dump("slave tx ", slave_tx, TEST_LEN);
	dump("slave rx ", slave_rx, received > 0 ? (int)received : 0);
	dump("master tx", master_tx, TEST_LEN);

	int errors = 0;

	for (int i = 0; i < TEST_LEN; i++) {
		if (master_rx[i] != slave_tx[i]) {
			PX4_ERR("master rx[%d] = 0x%02x, expected 0x%02x", i, master_rx[i], slave_tx[i]);
			errors++;
		}

		if (i < received && slave_rx[i] != master_tx[i]) {
			PX4_ERR("slave rx[%d] = 0x%02x, expected 0x%02x", i, slave_rx[i], master_tx[i]);
			errors++;
		}
	}

	if (errors == 0 && received == TEST_LEN) {
		PX4_INFO("SPI loopback SPI7 <-> SPI0: PASS");
		return 0;
	}

	PX4_ERR("SPI loopback SPI7 <-> SPI0: FAIL (%d errors, %d bytes received)", errors, (int)received);
	return 1;
}
