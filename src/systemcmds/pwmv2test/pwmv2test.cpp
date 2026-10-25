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
 * PE15 (the green RGB LED).  The sequence follows the HPM SDK pwmv2 sample
 * (samples/drivers/pwmv2/pwm_output):
 *
 *   pwmv2_deinit(), shadow unlock, period/cmp values in the shadow registers,
 *   counter data offset, burst disable, reload update time, comparator
 *   sources, shadow lock, channel output enable, counter enable and start.
 *
 * Duty cycle: comparator 0 is programmed with (reload - duty), so a larger
 * duty argument gives a wider high level on the pin.
 *
 * Usage:
 *   pwmv2test [frequency_hz] [duty_percent]
 *   pwmv2test loop [pairs]        (1 s of 1 kHz 20 %, then 1 s of 2 kHz 80 %)
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <string.h>
#include <unistd.h>

#include "hpm_pwmv2_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"

static PWMV2_Type *const PWM_BASE = HPM_PWM1;

/* PWM1 channel 7 -> counter 3, comparators 14 and 15. */
static constexpr pwm_channel_t PWM_CHANNEL = pwm_channel_7;
static constexpr pwm_counter_t PWM_COUNTER = pwm_counter_3;
static constexpr uint8_t PWM_CMP0 = 14;
static constexpr uint8_t PWM_CMP1 = 15;

/* Shadow slots: 0 = period, 1 = duty comparator, 2 = period comparator. */
static constexpr uint8_t PWM_PERIOD_SHADOW = PWMV2_SHADOW_INDEX(0);
static constexpr uint8_t PWM_DUTY_SHADOW = PWMV2_SHADOW_INDEX(1);
static constexpr uint8_t PWM_PERIOD_CMP_SHADOW = PWMV2_SHADOW_INDEX(2);

static void set_duty(uint32_t reload, unsigned duty_percent)
{
	/* Comparator 0 counts down from the period: 0 % -> reload, 100 % -> 0. */
	const uint32_t cmp = reload - (uint64_t)reload * duty_percent / 100;

	pwmv2_shadow_register_unlock(PWM_BASE);
	pwmv2_set_shadow_val(PWM_BASE, PWM_DUTY_SHADOW, cmp, 0, false);
	pwmv2_shadow_register_lock(PWM_BASE);
}

static void pwm_apply(uint32_t clock, uint32_t frequency, unsigned duty_percent)
{
	const uint32_t reload = clock / frequency - 1;

	/* Reset the module and program the shadow registers. */

	pwmv2_deinit(PWM_BASE);
	pwmv2_shadow_register_unlock(PWM_BASE);

	pwmv2_set_shadow_val(PWM_BASE, PWM_PERIOD_SHADOW, reload, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, PWM_DUTY_SHADOW, reload, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, PWM_PERIOD_CMP_SHADOW, reload + 1, 0, false);

	pwmv2_counter_select_data_offset_from_shadow_value(PWM_BASE, PWM_COUNTER, PWM_PERIOD_SHADOW);
	pwmv2_counter_burst_disable(PWM_BASE, PWM_COUNTER);
	pwmv2_set_reload_update_time(PWM_BASE, PWM_COUNTER, pwm_reload_update_on_reload);

	pwmv2_select_cmp_source(PWM_BASE, PWM_CMP0, cmp_value_from_shadow_val, PWM_DUTY_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, PWM_CMP1, cmp_value_from_shadow_val, PWM_PERIOD_CMP_SHADOW);

	pwmv2_shadow_register_lock(PWM_BASE);

	/* Two comparators per channel and output enable. */

	pwmv2_disable_four_cmp(PWM_BASE, (pwm_channel_t)(PWM_CHANNEL & ~1));
	pwmv2_channel_enable_output(PWM_BASE, PWM_CHANNEL);

	pwmv2_enable_counter(PWM_BASE, PWM_COUNTER);
	pwmv2_start_pwm_output(PWM_BASE, PWM_COUNTER);

	set_duty(reload, duty_percent);
}

extern "C" int pwmv2test_main(int argc, char *argv[])
{
	uint32_t frequency = 1000;
	unsigned duty_percent = 50;
	unsigned pairs = 30;
	bool loop = false;

	if (argc > 1 && strcmp(argv[1], "loop") == 0) {
		loop = true;

		if (argc > 2) {
			pairs = strtoul(argv[2], nullptr, 0);
		}

	} else {
		if (argc > 1) {
			frequency = strtoul(argv[1], nullptr, 0);
		}

		if (argc > 2) {
			duty_percent = strtoul(argv[2], nullptr, 0);
		}

		if (frequency == 0 || duty_percent > 100) {
			PX4_ERR("usage: pwmv2test [frequency_hz] [duty_percent] | loop [pairs]");
			return 1;
		}
	}

	/* Pin and clock: select the PWM alternate function explicitly, like the
	 * SDK board_init_rgb_pwm_pins() does.
	 */

	HPM_IOC->PAD[IOC_PAD_PE15].FUNC_CTL = IOC_PE15_FUNC_CTL_PWM1_P_7;
	HPM_IOC->PAD[IOC_PAD_PE15].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);

	clock_add_to_group(clock_pwm1, 0);

	const uint32_t clock = clock_get_frequency(clock_pwm1);

	if (clock == 0) {
		PX4_ERR("PWM1 clock is not running");
		return 1;
	}

	if (loop) {
		PX4_INFO("loop %u x (1 kHz 20%% / 2 kHz 80%%)", pairs);

		for (unsigned i = 0; i < pairs; i++) {
			pwm_apply(clock, 1000, 20);
			usleep(1000000);

			pwm_apply(clock, 2000, 80);
			usleep(1000000);
		}

		PX4_INFO("loop done");
		return 0;
	}

	const uint32_t reload = clock / frequency - 1;

	pwm_apply(clock, frequency, duty_percent);

	PX4_INFO("PWM1 ch7 (PE15): clock %u Hz, %u Hz, duty %u%%, reload %u",
		 (unsigned)clock, (unsigned)frequency, duty_percent, (unsigned)reload);

	PX4_INFO("pad: FUNC_CTL=0x%08x PAD_CTL=0x%08x, ch7: CFG0=0x%08x CFG1=0x%08x",
		 (unsigned)HPM_IOC->PAD[IOC_PAD_PE15].FUNC_CTL,
		 (unsigned)HPM_IOC->PAD[IOC_PAD_PE15].PAD_CTL,
		 (unsigned)PWM_BASE->PWM[PWM_CHANNEL].CFG0,
		 (unsigned)PWM_BASE->PWM[PWM_CHANNEL].CFG1);

	PX4_INFO("work: reload=%u cnt=%u cmp14=%u cmp15=%u shadow1=%u",
		 (unsigned)PWM_BASE->CNT_RELOAD_WORK[PWM_COUNTER],
		 (unsigned)PWM_BASE->CNT_VAL[PWM_COUNTER],
		 (unsigned)PWM_BASE->CMP_VAL_WORK[PWM_CMP0],
		 (unsigned)PWM_BASE->CMP_VAL_WORK[PWM_CMP1],
		 (unsigned)PWM_BASE->SHADOW_VAL[PWM_DUTY_SHADOW]);

	return 0;
}
