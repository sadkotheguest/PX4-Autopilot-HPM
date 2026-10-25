/****************************************************************************
 * HPM6E80 bring-up stubs
 *
 * The SPI bus driver is not ported to HPM6E00 yet, but the generic PX4
 * platform layer references it (platforms/common/spi.cpp reads the board SPI
 * bus table).  Provide the minimum needed to link; drop this file once spi.cpp
 * is enabled in this directory's CMakeLists.txt.
 *
 * The I2C bus initialisers are provided by the NuttX HPM I2C master driver
 * (hpm_i2cbus_initialize / hpm_i2cbus_uninitialize) and the board pads are set
 * up by hpm_i2cbus_pins_init() in i2c.cpp.
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/spi.h>

#include <nuttx/spi/spi.h>

/* Board SPI bus table: no SPI buses are configured on this board yet. */

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {};

/* SPI bus initialiser used by the MTD layer (ramtron_attach).  px4_mtd.cpp
 * references it with C++ linkage, hence the wrapper here.
 */

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
