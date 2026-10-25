/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * PX4 fmu-v6xrt internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <hpm_gpio.h>

#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1 // HPM GPTMR channel 1

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_I2C_BUS_MTD	1

/* Minimal bring-up stubs: SPI/I2C drivers are not enabled yet, but the
 * generic PX4 MTD code references these bus initializers.
 */
struct spi_dev_s;
struct i2c_master_s;
struct spi_dev_s *hpm_spibus_initialize(int bus);
struct i2c_master_s *hpm_i2cbus_initialize(int bus);
struct i2c_master_s *px4_i2cbus_initialize(int bus);
int hpm_i2cbus_uninitialize(struct i2c_master_s *dev);
int px4_i2cbus_uninitialize(struct i2c_master_s *dev);


/* I2C busses */

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define BOARD_MTD_NUM_EEPROM        2 /* MTD: base_eeprom, imu_eeprom*/


#define ADC_BATTERY_VOLTAGE_CHANNEL  0
#define ADC_BATTERY_CURRENT_CHANNEL  1

/* RC Serial port */
#define RC_SERIAL_PORT                "/dev/ttyS6"
#define RC_SERIAL_SINGLEWIRE
#define RC_INVERT_INPUT(invert)       rc_input_invert(invert)

/* I2C Pins
 *
 * HPM6E00EVK: the only I2C device is the CAT24C128 EEPROM (16 KiB, address
 * 0x50) on I2C1, wired to PA13 (SCL) / PA12 (SDA).  I2C0/2/3 are not routed
 * on this board, so they have no pin definitions.
 */
#define GPIO_I2C1_SCL_GPIO (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_DS_3V3_53 | GPIO_PORTA | GPIO_PIN13 | GPIO_LOOPBACK)
#define GPIO_I2C1_SDA_GPIO (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_DS_3V3_53 | GPIO_PORTA | GPIO_PIN12 | GPIO_LOOPBACK)
#define GPIO_I2C1_SCL      (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_DS_3V3_53 | GPIO_PORTA | GPIO_PIN13 | GPIO_LOOPBACK)
#define GPIO_I2C1_SDA      (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_DS_3V3_53 | GPIO_PORTA | GPIO_PIN12 | GPIO_LOOPBACK)

/* SPI Pins */
#define GPIO_SPI2_SCK  (GPIO_ALT | GPIO_AF5  | GPIO_PORTC | GPIO_PIN5 | GPIO_LOOPBACK)
#define GPIO_SPI2_MISO (GPIO_ALT | GPIO_AF5  | GPIO_PORTC | GPIO_PIN6)
#define GPIO_SPI2_MOSI (GPIO_ALT | GPIO_AF5  | GPIO_PORTC | GPIO_PIN7)

#define GPIO_SPI3_SCK  (GPIO_ALT | GPIO_AF5  | GPIO_PORTC | GPIO_PIN20 | GPIO_LOOPBACK)
#define GPIO_SPI3_MISO (GPIO_ALT | GPIO_AF5  | GPIO_PORTC | GPIO_PIN19)
#define GPIO_SPI3_MOSI (GPIO_ALT | GPIO_AF5  | GPIO_PORTC | GPIO_PIN21)

#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PE15 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PE15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT

#define GPIO_SAFETY_SWITCH_IN              /* PB24 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_SMT|GPIO_PORTB|GPIO_PIN24)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* PWM */
#define BOARD_NUM_IO_TIMERS         2
#define DIRECT_PWM_OUTPUT_CHANNELS  4

/* Input Capture not supported */

#define BOARD_HAS_NO_CAPTURE

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

extern int hpm_sdioinitialize(void);

extern void hpm_spiinitialize(void);

extern int hpm_usbinitialize(void);

extern void board_peripheral_reset(int ms);

extern int board_read_VBUS_state(void);

extern void rc_input_invert(bool invert);

extern int hpm_init_can_pins(int port);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
