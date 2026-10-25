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
 * @file pwmv2test.cpp
 *
 * PWMv2 bring-up test for the HPM6E00EVK.
 *
 * By default this generates a 1 kHz, 50% waveform on PWM1 channel 7, which is
 * PE15 (the green RGB LED on the board).  The channel is programmed through
 * the HPM SDK PWMv2 driver, so it validates the same code path the io_timer
 * layer uses without depending on the mixer.
 *
 * Usage: pwmv2test [frequency_hz] [duty_percent]
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include "hpm_pwmv2_drv.h"
#include "hpm_clock_drv.h"

static PWMV2_Type *const PWM_BASE = HPM_PWM1;

/* PWM1 channel 7 -> counter 3, comparator pair (14, 15). */
static constexpr pwm_channel_t PWM_CHANNEL = pwm_channel_7;
static constexpr pwm_counter_t PWM_COUNTER = pwm_counter_3;
static constexpr uint8_t PWM_CMP_START = 14;

static constexpr uint32_t PWM_PERIOD_SHADOW = PWMV2_SHADOW_INDEX(0);
static constexpr uint8_t PWM_CMP0_SHADOW = 1;
static constexpr uint8_t PWM_CMP1_SHADOW = 2;

static constexpr uint32_t GPIO_PWM1_CH7 = GPIO_ALT | GPIO_AF16 | GPIO_PORTE | GPIO_PIN15;

extern "C" int pwmv2test_main(int argc, char *argv[])
{
	uint32_t frequency = 1000;
	unsigned duty_percent = 50;

	if (argc > 1) {
		frequency = strtoul(argv[1], nullptr, 0);
	}

	if (argc > 2) {
		duty_percent = strtoul(argv[2], nullptr, 0);
	}

	if (frequency == 0 || duty_percent > 100) {
		PX4_ERR("usage: pwmv2test [frequency_hz] [duty_percent]");
		return 1;
	}

	/* Pin and clock */

	px4_arch_configgpio(GPIO_PWM1_CH7);
	clock_add_to_group(clock_pwm1, 0);

	const uint32_t clock = clock_get_frequency(clock_pwm1);

	if (clock == 0) {
		PX4_ERR("PWM1 clock is not running");
		return 1;
	}

	const uint32_t reload = clock / frequency - 1;
	const uint32_t cmp_duty = (uint64_t)(reload + 1) * duty_percent / 100;

	/* Period in shadow 0, compare values in the comparator pair */

	pwmv2_disable_counter(PWM_BASE, PWM_COUNTER);
	pwmv2_reset_counter(PWM_BASE, PWM_COUNTER);

	pwmv2_shadow_register_unlock(PWM_BASE);
	pwmv2_set_shadow_val(PWM_BASE, PWM_PERIOD_SHADOW, reload, 0, false);
	pwmv2_shadow_register_lock(PWM_BASE);

	pwmv2_counter_select_data_offset_from_shadow_value(PWM_BASE, PWM_COUNTER, PWM_PERIOD_SHADOW);
	pwmv2_set_reload_update_time(PWM_BASE, PWM_COUNTER, pwm_reload_update_on_reload);

	pwmv2_cmp_config_t cmp_config[2] = {};

	for (int i = 0; i < 2; i++) {
		cmp_config[i].cmp_source = cmp_value_from_shadow_val;
		cmp_config[i].update_trigger = pwm_shadow_register_update_on_reload;
	}

	cmp_config[0].cmp = 0;
	cmp_config[0].cmp_source_index = PWM_CMP0_SHADOW;
	cmp_config[1].cmp = cmp_duty;
	cmp_config[1].cmp_source_index = PWM_CMP1_SHADOW;

	pwmv2_config_t pwm_config;
	pwmv2_get_default_config(&pwm_config);
	pwm_config.enable_output = true;
	pwm_config.invert_output = false;

	const hpm_stat_t stat = pwmv2_setup_waveform(PWM_BASE, PWM_CHANNEL, &pwm_config,
				  PWM_CMP_START, cmp_config, 2);

	if (stat != status_success) {
		PX4_ERR("pwmv2_setup_waveform failed: %d", (int)stat);
		return 1;
	}

	pwmv2_issue_shadow_register_lock_event(PWM_BASE);
	pwmv2_enable_counter(PWM_BASE, PWM_COUNTER);
	pwmv2_start_pwm_output(PWM_BASE, PWM_COUNTER);

	PX4_INFO("PWM1 ch7 (PE15): clock %u Hz, %u Hz, duty %u%%, reload %u, cmp %u",
		 (unsigned)clock, (unsigned)frequency, duty_percent,
		 (unsigned)reload, (unsigned)cmp_duty);

	/* Read back the working registers so the result is visible from the console. */

	PX4_INFO("work: reload=%u cnt=%u cmp14=%u cmp15=%u",
		 (unsigned)PWM_BASE->CNT_RELOAD_WORK[PWM_COUNTER],
		 (unsigned)PWM_BASE->CNT_VAL[PWM_COUNTER],
		 (unsigned)PWM_BASE->CMP_VAL_WORK[PWM_CMP_START],
		 (unsigned)PWM_BASE->CMP_VAL_WORK[PWM_CMP_START + 1]);

	return 0;
}
