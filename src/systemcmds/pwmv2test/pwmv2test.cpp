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
 *   pwmv2test dshot [150|300|600] [throttle_pct] [duration_ms]
 *                                 (DShot bit stream through PWMv2 + DMA)
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <string.h>
#include <unistd.h>

#include "hpm_pwmv2_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"
#include "hpm_dma_mgr.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_src.h"
#include "hpm_dmamux_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"

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

/* ------------------------------------------------------------------------ */
/* DShot bring-up: PWMv2 generates the bit stream, DMA feeds the duty shadow */
/*                                                                          */
/* Output: PE08 / PE09 = PWM1 channels 0 / 1 (motor header).  Both channels   */
/* share one duty shadow and the period of counter 0, so they output the same */
/* frame.                                                                    */
/* ------------------------------------------------------------------------ */

static constexpr uint8_t DSHOT_GAP_BITS = 60;

static constexpr pwm_counter_t DSHOT_COUNTER = pwm_counter_0;
static constexpr uint8_t DSHOT_CMP_DUTY0 = 0;   /* channel 0: duty */
static constexpr uint8_t DSHOT_CMP_EDGE0 = 1;   /* channel 0: period edge */
static constexpr uint8_t DSHOT_CMP_DUTY1 = 2;   /* channel 1: duty */
static constexpr uint8_t DSHOT_CMP_EDGE1 = 3;   /* channel 1: period edge */
static constexpr uint8_t DSHOT_PERIOD_SHADOW = PWMV2_SHADOW_INDEX(0);
static constexpr uint8_t DSHOT_DUTY_SHADOW = PWMV2_SHADOW_INDEX(1);
static constexpr uint8_t DSHOT_EDGE_SHADOW = PWMV2_SHADOW_INDEX(2);

static uint32_t dshot_buf[16 + DSHOT_GAP_BITS];

/* Comparator 0 counts down from the period: 75 % for a "1", 37.5 % for a "0". */
static uint32_t dshot_cmp(uint32_t reload, bool bit)
{
	const uint32_t duty = bit ? (reload * 3u / 4u) : (reload * 3u / 8u);

	return reload - duty;
}

/* 11 bit throttle + 1 telemetry request bit + 4 bit CRC. */
static uint16_t dshot_frame_bits(unsigned throttle_percent)
{
	uint32_t throttle = 48u + (uint32_t)throttle_percent * 2000u / 100u;

	if (throttle > 2047u) {
		throttle = 2047u;
	}

	const uint16_t value = (uint16_t)(throttle << 1);      /* telemetry request = 0 */
	const uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

	return (uint16_t)((value << 4) | crc);
}

/* Build the cyclic DMA ring for one frame (same layout as dshot_run()). */
static void dshot_fill_buf(uint32_t reload, unsigned throttle_percent)
{
	const uint16_t frame = dshot_frame_bits(throttle_percent);

	for (int i = 1; i < 16; i++) {
		dshot_buf[i - 1] = dshot_cmp(reload, (frame >> (15 - i)) & 1) << 8;
	}

	for (int i = 0; i < DSHOT_GAP_BITS; i++) {
		dshot_buf[15 + i] = reload << 8;
	}

	dshot_buf[15 + DSHOT_GAP_BITS] = dshot_cmp(reload, (frame >> 15) & 1) << 8;
}

/* Offline check: frame bits and the DMA ring, no hardware access. */
static int dshot_dump(uint32_t clock, unsigned rate_kbit, unsigned throttle_percent)
{
	if (rate_kbit != 150 && rate_kbit != 300 && rate_kbit != 600) {
		PX4_ERR("dshot rate must be 150, 300 or 600 kbit/s");
		return 1;
	}

	if (throttle_percent > 100) {
		PX4_ERR("usage: pwmv2test dshotbuf [150|300|600] [throttle_pct]");
		return 1;
	}

	const uint32_t bit_ns = 1000000000u / (rate_kbit * 1000u);
	const uint32_t reload = (uint32_t)((uint64_t)clock * bit_ns / 1000000000ULL) - 1;
	const uint16_t frame = dshot_frame_bits(throttle_percent);

	dshot_fill_buf(reload, throttle_percent);

	PX4_INFO("frame 0x%04x (value %u, throttle %u%%), reload %u, cmp1 %u, cmp0 %u",
		 (unsigned)frame, (unsigned)(frame >> 4), throttle_percent, (unsigned)reload,
		 (unsigned)dshot_cmp(reload, true), (unsigned)dshot_cmp(reload, false));

	PX4_INFO("bits: %d%d%d%d %d%d%d%d %d%d%d%d %d%d%d%d",
		 (frame >> 15) & 1, (frame >> 14) & 1, (frame >> 13) & 1, (frame >> 12) & 1,
		 (frame >> 11) & 1, (frame >> 10) & 1, (frame >> 9) & 1, (frame >> 8) & 1,
		 (frame >> 7) & 1, (frame >> 6) & 1, (frame >> 5) & 1, (frame >> 4) & 1,
		 (frame >> 3) & 1, (frame >> 2) & 1, (frame >> 1) & 1, frame & 1);

	PX4_INFO("ring[0..7] cmp: %u %u %u %u %u %u %u %u",
		 (unsigned)(dshot_buf[0] >> 8), (unsigned)(dshot_buf[1] >> 8),
		 (unsigned)(dshot_buf[2] >> 8), (unsigned)(dshot_buf[3] >> 8),
		 (unsigned)(dshot_buf[4] >> 8), (unsigned)(dshot_buf[5] >> 8),
		 (unsigned)(dshot_buf[6] >> 8), (unsigned)(dshot_buf[7] >> 8));
	PX4_INFO("ring[15] gap cmp %u, ring[%u] bit0 cmp %u",
		 (unsigned)(dshot_buf[15] >> 8), (unsigned)(15 + DSHOT_GAP_BITS),
		 (unsigned)(dshot_buf[15 + DSHOT_GAP_BITS] >> 8));

	return 0;
}

static void dshot_pins(void)
{
	HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
	HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
	HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
}

static int dshot_run(uint32_t clock, unsigned rate_kbit, unsigned throttle_percent, unsigned frames)
{
	if (rate_kbit != 150 && rate_kbit != 300 && rate_kbit != 600) {
		PX4_ERR("dshot rate must be 150, 300 or 600 kbit/s");
		return 1;
	}

	if (throttle_percent > 100 || frames == 0) {
		PX4_ERR("usage: pwmv2test dshot [150|300|600] [throttle_pct] [duration_ms]");
		return 1;
	}

	dshot_pins();

	/* Bit period in counter ticks. */
	const uint32_t bit_ns = 1000000000u / (rate_kbit * 1000u);
	const uint32_t reload = (uint32_t)((uint64_t)clock * bit_ns / 1000000000ULL) - 1;
	const uint16_t frame = dshot_frame_bits(throttle_percent);

	/* A value written by DMA at the end of period N is latched at reload N+1,
	 * so bit 0 is preloaded and the buffer starts with bit 1.  The trailing
	 * zero-duty entries provide the inter-frame gap; bit 0 is appended again
	 * so that the cyclic buffer keeps every frame aligned.
	 */
	for (int i = 1; i < 16; i++) {
		dshot_buf[i - 1] = dshot_cmp(reload, (frame >> (15 - i)) & 1) << 8;
	}

	for (int i = 0; i < DSHOT_GAP_BITS; i++) {
		dshot_buf[15 + i] = reload << 8;
	}

	dshot_buf[15 + DSHOT_GAP_BITS] = dshot_cmp(reload, (frame >> 15) & 1) << 8;


	pwmv2_deinit(PWM_BASE);
	pwmv2_shadow_register_unlock(PWM_BASE);

	pwmv2_set_shadow_val(PWM_BASE, DSHOT_PERIOD_SHADOW, reload, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, DSHOT_DUTY_SHADOW, dshot_cmp(reload, (frame >> 15) & 1), 0, false);
	pwmv2_set_shadow_val(PWM_BASE, DSHOT_EDGE_SHADOW, reload + 1, 0, false);

	pwmv2_counter_select_data_offset_from_shadow_value(PWM_BASE, DSHOT_COUNTER, DSHOT_PERIOD_SHADOW);
	pwmv2_counter_burst_disable(PWM_BASE, DSHOT_COUNTER);
	pwmv2_set_reload_update_time(PWM_BASE, DSHOT_COUNTER, pwm_reload_update_on_reload);

	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_DUTY0, cmp_value_from_shadow_val, DSHOT_DUTY_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_EDGE0, cmp_value_from_shadow_val, DSHOT_EDGE_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_DUTY1, cmp_value_from_shadow_val, DSHOT_DUTY_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_EDGE1, cmp_value_from_shadow_val, DSHOT_EDGE_SHADOW);

	pwmv2_shadow_register_lock(PWM_BASE);

	/* One DMA request per period: the counter 0 reload feeds pwm_dma_0. */
	pwmv2_enable_dma_at_reload_point(PWM_BASE, pwm_dma_0, DSHOT_COUNTER);

	pwmv2_disable_four_cmp(PWM_BASE, pwm_channel_0);
	pwmv2_channel_enable_output(PWM_BASE, pwm_channel_0);
	pwmv2_channel_enable_output(PWM_BASE, pwm_channel_1);

	pwmv2_enable_counter(PWM_BASE, DSHOT_COUNTER);
	pwmv2_start_pwm_output(PWM_BASE, DSHOT_COUNTER);

	/* Route the PWM DMA request through TRGM to a DMAMUX source for HDMA. */
	trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM1_REQ0);

	dma_mgr_init();

	dma_resource_t dma_res;

	if (dma_mgr_request_resource(&dma_res) != status_success) {
		PX4_ERR("no DMA channel");
		return 1;
	}

	dma_mgr_chn_conf_t conf;
	dma_mgr_get_default_chn_config(&conf);
	conf.en_dmamux = true;
	conf.dmamux_src = HPM_DMA_SRC_MOT_0;
	conf.src_mode = 0;                      /* normal */
	conf.dst_mode = 1;                      /* handshake */
	conf.src_width = DMA_TRANSFER_WIDTH_WORD;
	conf.dst_width = DMA_TRANSFER_WIDTH_WORD;
	conf.src_addr_ctrl = 0;                 /* increment */
	conf.dst_addr_ctrl = 2;                 /* fixed */
	conf.size_in_byte = sizeof(dshot_buf);
	conf.src_addr = (uint32_t)dshot_buf;
	conf.dst_addr = (uint32_t)&PWM_BASE->SHADOW_VAL[DSHOT_DUTY_SHADOW];
	conf.interrupt_mask = 0;
	conf.en_infiniteloop = true;
	conf.handshake_opt = DMA_HANDSHAKE_OPT_ONE_BURST;

	if (dma_mgr_setup_channel(&dma_res, &conf) != status_success) {
		PX4_ERR("dma_mgr_setup_channel failed");
		return 1;
	}

	if (dma_mgr_enable_channel(&dma_res) != status_success) {
		PX4_ERR("dma_mgr_enable_channel failed");
		return 1;
	}

	PX4_INFO("conf: en_dmamux=%d src=0x%02x dst=0x%08x size=%u",
		 (int)conf.en_dmamux, (unsigned)conf.dmamux_src,
		 (unsigned)conf.dst_addr, (unsigned)conf.size_in_byte);


	/* While the stream runs, sample the duty shadow: DMA must be cycling
	 * through the "0" and "1" comparator values.
	 */
	uint32_t shadow_min = 0xFFFFFFFFu, shadow_max = 0;

	for (unsigned ms = 0; ms < frames; ms++) {
		const uint32_t shadow = PWM_BASE->SHADOW_VAL[DSHOT_DUTY_SHADOW] >> 8;

		if (shadow < shadow_min) {
			shadow_min = shadow;
		}

		if (shadow > shadow_max) {
			shadow_max = shadow;
		}

		usleep(1000);
	}

	dma_mgr_disable_channel(&dma_res);
	pwmv2_disable_counter(PWM_BASE, DSHOT_COUNTER);
	pwmv2_channel_disable_output(PWM_BASE, pwm_channel_0);
	pwmv2_channel_disable_output(PWM_BASE, pwm_channel_1);

	PX4_INFO("dshot PE08/PE09 (PWM1 ch0/1): %u kbit/s, throttle %u%%, reload %u, frame 0x%04x, %u ms, DMA ch %d",
		 rate_kbit, throttle_percent, (unsigned)reload, (unsigned)frame, frames, (int)dma_res.channel);
	PX4_INFO("shadow min/max %u/%u (expect %u/%u), cnt %u", (unsigned)shadow_min, (unsigned)shadow_max,
		 (unsigned)dshot_cmp(reload, false), (unsigned)dshot_cmp(reload, true),
		 (unsigned)PWM_BASE->CNT_VAL[DSHOT_COUNTER]);
	PX4_INFO("buf: %u %u %u ... %u", (unsigned)dshot_buf[0], (unsigned)dshot_buf[1],
		 (unsigned)dshot_buf[2], (unsigned)dshot_buf[15 + DSHOT_GAP_BITS]);

	return 0;
}
extern "C" int pwmv2test_main(int argc, char *argv[])
{
	uint32_t frequency = 1000;
	unsigned duty_percent = 50;
	unsigned pairs = 30;
	bool loop = false;
	bool dshot = false;
	bool dshotbuf = false;
	unsigned dshot_rate = 600;
	unsigned dshot_throttle = 0;
	unsigned dshot_frames = 3000;   /* duration, ms */

	if (argc > 1 && strcmp(argv[1], "dshotbuf") == 0) {
		dshotbuf = true;

		if (argc > 2) {
			dshot_rate = strtoul(argv[2], nullptr, 0);
		}

		if (argc > 3) {
			dshot_throttle = strtoul(argv[3], nullptr, 0);
		}

	} else if (argc > 1 && strcmp(argv[1], "dshot") == 0) {
		dshot = true;

		if (argc > 2) {
			dshot_rate = strtoul(argv[2], nullptr, 0);
		}

		if (argc > 3) {
			dshot_throttle = strtoul(argv[3], nullptr, 0);
		}

		if (argc > 4) {
			dshot_frames = strtoul(argv[4], nullptr, 0);
		}

	} else if (argc > 1 && strcmp(argv[1], "loop") == 0) {
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
			PX4_ERR("usage: pwmv2test [frequency_hz] [duty_percent] | loop [pairs] | dshot [150|300|600] [throttle_pct] [duration_ms] | dshotbuf [150|300|600] [throttle_pct]");
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

	if (dshotbuf) {
		return dshot_dump(clock, dshot_rate, dshot_throttle);
	}

	if (dshot) {
		return dshot_run(clock, dshot_rate, dshot_throttle, dshot_frames);
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
