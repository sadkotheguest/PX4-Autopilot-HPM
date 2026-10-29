/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file led.c
 *
 * Board LED backend for the HPM6E00EVK, used by the generic PX4 LED driver
 * (/dev/led0) once drv_led_start() is called from board_app_initialize().
 *
 * The evaluation board has three LED pins (see the SDK board header):
 *   red   = PE14
 *   green = PE15
 *   blue  = PE4
 * PE15 is kept as the safety-switch indicator on this board (it is configured
 * as GPIO_LED_SAFETY in board_config.h and set high at start-up), so the PX4
 * status LED uses the red and blue channels only:
 *   LED_RED / LED_AMBER -> red
 *   LED_BLUE            -> blue
 *   LED_GREEN           -> blue as well (armed is a steady blue, disarmed
 *                          blinks, so the two states stay distinguishable
 *                          without touching the safety pin)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <drivers/drv_board_led.h>

#include "board.h"
#include "hpm_gpio_drv.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool led_pin(int led, GPIO_Type **ctrl, uint32_t *index, uint8_t *pin)
{
	switch (led) {
	case LED_RED: /* LED_AMBER has the same value on this board */
		*ctrl = BOARD_R_GPIO_CTRL;
		*index = BOARD_R_GPIO_INDEX;
		*pin = BOARD_R_GPIO_PIN;
		return true;

	case LED_BLUE:
	case LED_GREEN:
		*ctrl = BOARD_B_GPIO_CTRL;
		*index = BOARD_B_GPIO_INDEX;
		*pin = BOARD_B_GPIO_PIN;
		return true;

	default:
		return false;
	}
}

static void led_write(int led, uint8_t state)
{
	GPIO_Type *ctrl;
	uint32_t index;
	uint8_t pin;

	if (led_pin(led, &ctrl, &index, &pin)) {
		gpio_write_pin(ctrl, index, pin, state);
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void led_init(void)
{
	/* Only the two status channels: PE15 belongs to the safety indicator. */

	gpio_set_pin_output_with_initial(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX,
					 BOARD_R_GPIO_PIN, BOARD_LED_OFF_LEVEL);
	gpio_set_pin_output_with_initial(BOARD_B_GPIO_CTRL, BOARD_B_GPIO_INDEX,
					 BOARD_B_GPIO_PIN, BOARD_LED_OFF_LEVEL);
}

void led_on(int led)
{
	led_write(led, BOARD_LED_ON_LEVEL);
}

void led_off(int led)
{
	led_write(led, BOARD_LED_OFF_LEVEL);
}

void led_toggle(int led)
{
	GPIO_Type *ctrl;
	uint32_t index;
	uint8_t pin;

	if (led_pin(led, &ctrl, &index, &pin)) {
		gpio_toggle_pin(ctrl, index, pin);
	}
}