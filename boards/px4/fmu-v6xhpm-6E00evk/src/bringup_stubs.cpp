/****************************************************************************
 * HPM6E80 bring-up stubs
 *
 * The I2C and SPI bus drivers are not ported to HPM6E00 yet, but the generic
 * PX4 platform layer references them:
 *
 *   - px4_mtd.cpp  (at24xxx_attach / ramtron_attach) calls the bus
 *     initialisers,
 *   - platforms/common/spi.cpp reads the board SPI bus table.
 *
 * Provide the minimum needed to link and start NSH/PX4.  Once the real bus
 * drivers are enabled (uncomment i2c.cpp / spi.cpp in this directory's
 * CMakeLists.txt), drop the corresponding definitions from here - otherwise
 * the symbols are defined twice.
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/spi.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>

/* Board SPI bus table: no SPI buses are configured on this board yet. */

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {};

/* I2C bus initialisers (see px4_arch/micro_hal.h: px4_i2cbus_initialize()
 * maps to hpm_i2cbus_initialize() on the HPM platform).
 */

struct i2c_master_s *px4_i2cbus_initialize(int bus)
{
	(void)bus;
	return nullptr;
}

int px4_i2cbus_uninitialize(struct i2c_master_s *dev)
{
	(void)dev;
	return 0;
}

/* SPI bus initialiser used by the MTD layer (ramtron_attach). */

struct spi_dev_s *hpm_spibus_initialize(int bus)
{
	(void)bus;
	return nullptr;
}

/* USB VBUS sensing: there is no USB device port driver on this board yet,
 * but systemcmds/usb_connected links against this symbol.
 */

extern "C" int board_read_VBUS_state(void)
{
	return 0;
}
