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
#include <drivers/drv_hrt.h>

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

	/* RGB LED pin: only the PWM/LED modes touch PE15. */
	HPM_IOC->PAD[IOC_PAD_PE15].FUNC_CTL = IOC_PE15_FUNC_CTL_PWM1_P_7;
	HPM_IOC->PAD[IOC_PAD_PE15].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);

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
/* share the duty shadow (slot 20) and the period of counter 0.              */
/*                                                                          */
/* One frame = one DMA transfer of 18 words (bit1..bit15 + 3 idle), one word  */
/* per PWM period from the counter reload request.  The transfer is armed     */
/* through dma_mgr (TC callback), the counter is started together with it and */
/* stopped when the frame is complete.                                       */
/* ------------------------------------------------------------------------ */

static constexpr uint8_t DSHOT_GAP_WORDS = 3;
static constexpr uint8_t DSHOT_FRAME_WORDS = 15 + DSHOT_GAP_WORDS;

static constexpr pwm_counter_t DSHOT_COUNTER = pwm_counter_0;
static constexpr uint8_t DSHOT_CMP_DUTY0 = 0;   /* channel 0: duty */
static constexpr uint8_t DSHOT_CMP_EDGE0 = 1;   /* channel 0: period edge */
static constexpr uint8_t DSHOT_CMP_DUTY1 = 2;   /* channel 1: duty */
static constexpr uint8_t DSHOT_CMP_EDGE1 = 3;   /* channel 1: period edge */
static constexpr uint8_t DSHOT_PERIOD_SHADOW = PWMV2_SHADOW_INDEX(0);
static constexpr uint8_t DSHOT_DUTY_SHADOW = PWMV2_SHADOW_INDEX(20);
static constexpr uint8_t DSHOT_EDGE_SHADOW = PWMV2_SHADOW_INDEX(2);

/* DMA-visible buffer: AHB SRAM, like the SDK pwmv2_cal_shared_delta example. */
static uint32_t dshot_buf[DSHOT_FRAME_WORDS] __attribute__((section(".ahb_sram")));

static volatile uint32_t dshot_tc_count;

static void dshot_tc_cb(DMA_Type *base, uint32_t channel, void *cb_data)
{
	(void)base;
	(void)channel;
	(void)cb_data;
	dshot_tc_count++;
}

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

/* Ring for one frame: bit 1..15, then idle words.  Bit 0 is preloaded. */
static void dshot_fill_buf(uint32_t reload, unsigned throttle_percent)
{
	const uint16_t frame = dshot_frame_bits(throttle_percent);

	for (int i = 1; i < 16; i++) {
		dshot_buf[i - 1] = dshot_cmp(reload, (frame >> (15 - i)) & 1) << 8;
	}

	for (int i = 15; i < DSHOT_FRAME_WORDS; i++) {
		dshot_buf[i] = reload << 8;
	}
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

	PX4_INFO("frame 0x%04x (value %u, throttle %u%%), reload %u, cmp1 %u, cmp0 %u, %u words",
		 (unsigned)frame, (unsigned)(frame >> 4), throttle_percent, (unsigned)reload,
		 (unsigned)dshot_cmp(reload, true), (unsigned)dshot_cmp(reload, false),
		 (unsigned)DSHOT_FRAME_WORDS);

	PX4_INFO("bits: %d%d%d%d %d%d%d%d %d%d%d%d %d%d%d%d",
		 (frame >> 15) & 1, (frame >> 14) & 1, (frame >> 13) & 1, (frame >> 12) & 1,
		 (frame >> 11) & 1, (frame >> 10) & 1, (frame >> 9) & 1, (frame >> 8) & 1,
		 (frame >> 7) & 1, (frame >> 6) & 1, (frame >> 5) & 1, (frame >> 4) & 1,
		 (frame >> 3) & 1, (frame >> 2) & 1, (frame >> 1) & 1, frame & 1);

	PX4_INFO("ring cmp: %u %u %u %u %u %u %u %u | %u %u %u",
		 (unsigned)(dshot_buf[0] >> 8), (unsigned)(dshot_buf[1] >> 8),
		 (unsigned)(dshot_buf[2] >> 8), (unsigned)(dshot_buf[3] >> 8),
		 (unsigned)(dshot_buf[4] >> 8), (unsigned)(dshot_buf[5] >> 8),
		 (unsigned)(dshot_buf[6] >> 8), (unsigned)(dshot_buf[7] >> 8),
		 (unsigned)(dshot_buf[15] >> 8), (unsigned)(dshot_buf[16] >> 8),
		 (unsigned)(dshot_buf[17] >> 8));

	return 0;
}

static void dshot_pins(void)
{
	HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
	HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
	HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
}

/* Configure the PWM for one frame; bit 0 is preloaded into the duty shadow. */
static void dshot_pwm_frame(uint32_t reload, uint16_t frame)
{
	pwmv2_deinit(PWM_BASE);

	/* DMA writes the duty shadow: unlock that slot (bit = 2 + slot). */
	PWM_BASE->UNLOCK |= (1u << (2u + DSHOT_DUTY_SHADOW));

	pwmv2_shadow_register_unlock(PWM_BASE);

	pwmv2_set_shadow_val(PWM_BASE, DSHOT_PERIOD_SHADOW, reload, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, DSHOT_DUTY_SHADOW, dshot_cmp(reload, (frame >> 15) & 1), 0, false);
	pwmv2_set_shadow_val(PWM_BASE, DSHOT_EDGE_SHADOW, reload + 1, 0, false);

	pwmv2_counter_select_data_offset_from_shadow_value(PWM_BASE, DSHOT_COUNTER, DSHOT_PERIOD_SHADOW);
	pwmv2_counter_burst_disable(PWM_BASE, DSHOT_COUNTER);
	pwmv2_set_reload_update_time(PWM_BASE, DSHOT_COUNTER, pwm_reload_update_on_shlk);

	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_DUTY0, cmp_value_from_shadow_val, DSHOT_DUTY_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_EDGE0, cmp_value_from_shadow_val, DSHOT_EDGE_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_DUTY1, cmp_value_from_shadow_val, DSHOT_DUTY_SHADOW);
	pwmv2_select_cmp_source(PWM_BASE, DSHOT_CMP_EDGE1, cmp_value_from_shadow_val, DSHOT_EDGE_SHADOW);

	pwmv2_shadow_register_lock(PWM_BASE);
	pwmv2_issue_shadow_register_lock_event(PWM_BASE);

	pwmv2_disable_four_cmp(PWM_BASE, pwm_channel_0);
	pwmv2_channel_enable_output(PWM_BASE, pwm_channel_0);
	pwmv2_channel_enable_output(PWM_BASE, pwm_channel_1);
}

/* Harness for the arch_dshot API (same functions as the STM32 port). */

extern "C" int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot);
extern "C" void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry);
extern "C" int up_dshot_arm(bool armed);
extern "C" void up_dshot_trigger(void);
extern "C" uint32_t up_dshot_tc_count(void);
extern "C" void up_dshot_tc_reset(void);
extern "C" void up_dshot_deinit(void);

static int dshot_run(uint32_t clock, unsigned rate_kbit, unsigned throttle_percent, unsigned frames)
{
	(void)clock;

	if (rate_kbit != 150 && rate_kbit != 300 && rate_kbit != 600) {
		PX4_ERR("dshot rate must be 150, 300 or 600 kbit/s");
		return 1;
	}

	if (throttle_percent > 100 || frames == 0) {
		PX4_ERR("usage: pwmv2test dshot [150|300|600] [throttle_pct] [frames]");
		return 1;
	}

	const unsigned rate_hz = rate_kbit * 1000u;

	if (up_dshot_init(0x3, rate_hz, false) != OK) {
		PX4_ERR("up_dshot_init failed");
		return 1;
	}

	uint32_t throttle = 48u + throttle_percent * 2000u / 100u;

	if (throttle > 2047u) {
		throttle = 2047u;
	}

	dshot_motor_data_set(0, (uint16_t)throttle, false);

	if (up_dshot_arm(true) != OK) {
		PX4_ERR("up_dshot_arm failed");
		up_dshot_deinit();
		return 1;
	}

	unsigned sent = 0, complete = 0;
	uint32_t t_min = 0xFFFFFFFFu, t_max = 0, t_sum = 0;

	for (unsigned f = 0; f < frames; f++) {
		up_dshot_tc_reset();
		const hrt_abstime t0 = hrt_absolute_time();
		up_dshot_trigger();

		uint32_t spin = 0;

		while (spin++ < 5000000u && up_dshot_tc_count() == 0) {
		}

		const uint32_t dt = hrt_elapsed_time(&t0);

		if (dt < t_min) {
			t_min = dt;
		}

		if (dt > t_max) {
			t_max = dt;
		}

		t_sum += dt;

		if (up_dshot_tc_count() != 0) {
			complete++;
		}

		sent++;
		usleep(200);
	}

	up_dshot_arm(false);
	up_dshot_deinit();

	PX4_INFO("dshot %u kbit/s, throttle %u (frame value), %u words, frames %u/%u",
		 rate_kbit, (unsigned)throttle, (unsigned)DSHOT_FRAME_WORDS, complete, sent);
	PX4_INFO("trigger time: min %u us, avg %u us, max %u us (bit period %u us)",
		 (unsigned)t_min, (unsigned)(t_sum / (sent ? sent : 1)), (unsigned)t_max,
		 (unsigned)(1000000u / rate_hz));

	return (complete == sent) ? 0 : 1;
}

/* ------------------------------------------------------------------------ */
/* DMA diagnostics: same PWM trigger, destination either RAM or SHADOW_VAL   */
/* ------------------------------------------------------------------------ */

static volatile uint32_t diag_tc_count;

static void diag_tc_cb(DMA_Type *base, uint32_t channel, void *cb_data)
{
	(void)base;
	(void)channel;
	(void)cb_data;
	diag_tc_count++;
}

static uint32_t diag_src[64] __attribute__((section(".ahb_sram")));
static uint32_t diag_dst[64] __attribute__((section(".ahb_sram")));

static int dmadiag_run(uint32_t clock, unsigned freq_hz, unsigned words, const char *dest_name)
{
	const bool mem_xdma = (strcmp(dest_name, "memcpyxdma") == 0);
	const bool mem_only = (strcmp(dest_name, "memcpy") == 0) || mem_xdma;
	const bool to_ram = (strcmp(dest_name, "ram") == 0) || mem_only;
	const bool to_shadow = (strcmp(dest_name, "shadow") == 0);
	const bool use_xdma = (strcmp(dest_name, "shadowxdma") == 0) || mem_xdma;

	if (freq_hz == 0 || words == 0 || words > 64 || (!to_ram && !to_shadow && !use_xdma && !mem_only)) {
		PX4_ERR("usage: pwmv2test dmadiag <freq_hz> <words> <ram|shadow|shadowxdma|memcpy|memcpyxdma>");
		return 1;
	}

	const uint32_t reload = clock / freq_hz - 1;

	HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
	HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
	HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);

	clock_add_to_group(clock_hdma, 0);
	clock_add_to_group(clock_xdma, 0);

	memset(diag_dst, 0, sizeof(diag_dst));

	for (unsigned i = 0; i < words; i++) {
		diag_src[i] = 0x1000u + i;
	}

	pwmv2_deinit(PWM_BASE);
	pwmv2_shadow_register_unlock(PWM_BASE);

	pwmv2_set_shadow_val(PWM_BASE, PWMV2_SHADOW_INDEX(0), reload, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, PWMV2_SHADOW_INDEX(1), reload / 2, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, PWMV2_SHADOW_INDEX(2), reload + 1, 0, false);

	pwmv2_counter_select_data_offset_from_shadow_value(PWM_BASE, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
	pwmv2_counter_burst_disable(PWM_BASE, pwm_counter_0);
	pwmv2_set_reload_update_time(PWM_BASE, pwm_counter_0, pwm_reload_update_on_shlk);

	pwmv2_select_cmp_source(PWM_BASE, 0, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(1));
	pwmv2_select_cmp_source(PWM_BASE, 1, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(2));
	pwmv2_select_cmp_source(PWM_BASE, 2, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(1));
	pwmv2_select_cmp_source(PWM_BASE, 3, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(2));

	pwmv2_shadow_register_lock(PWM_BASE);
	pwmv2_disable_four_cmp(PWM_BASE, pwm_channel_0);
	pwmv2_channel_enable_output(PWM_BASE, pwm_channel_0);
	pwmv2_channel_enable_output(PWM_BASE, pwm_channel_1);

	pwmv2_reset_counter(PWM_BASE, pwm_counter_0);
	pwmv2_enable_counter(PWM_BASE, pwm_counter_0);
	pwmv2_start_pwm_output(PWM_BASE, pwm_counter_0);

	if (!mem_only) {
		trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM1_REQ0);
		pwmv2_enable_dma_at_reload_point(PWM_BASE, pwm_dma_0, pwm_counter_0);
		PWM_BASE->UNLOCK |= (1u << (2u + PWMV2_SHADOW_INDEX(20)));
	}

	dma_mgr_init();

	dma_resource_t res;

	if ((use_xdma ? dma_mgr_request_specified_resource(&res, HPM_XDMA) : dma_mgr_request_resource(&res)) != status_success) {
		PX4_ERR("dmadiag: no DMA resource");
		return 1;
	}

	dma_mgr_chn_conf_t conf;
	dma_mgr_get_default_chn_config(&conf);
	conf.en_dmamux = !mem_only;
	conf.dmamux_src = HPM_DMA_SRC_MOT_0;
	conf.src_mode = 0;
	conf.dst_mode = to_ram ? 0 : 1;
	conf.src_width = DMA_TRANSFER_WIDTH_WORD;
	conf.dst_width = DMA_TRANSFER_WIDTH_WORD;
	conf.src_addr_ctrl = 0;
	conf.dst_addr_ctrl = to_ram ? 0 : 2;
	conf.src_addr = (uint32_t)diag_src;
	conf.dst_addr = to_ram ? (uint32_t)diag_dst : (uint32_t)&PWM_BASE->SHADOW_VAL[PWMV2_SHADOW_INDEX(20)];
	conf.size_in_byte = words * 4u;
	conf.en_infiniteloop = false;
	conf.handshake_opt = DMA_HANDSHAKE_OPT_ONE_BURST;
	conf.interrupt_mask = DMA_INTERRUPT_MASK_ALL & ~DMA_INTERRUPT_MASK_TERMINAL_COUNT;

	diag_tc_count = 0;
	dma_mgr_install_chn_tc_callback(&res, diag_tc_cb, NULL);

	if (dma_mgr_setup_channel(&res, &conf) != status_success) {
		PX4_ERR("dmadiag: setup failed");
		dma_mgr_release_resource(&res);
		return 1;
	}

	if (dma_mgr_enable_channel(&res) != status_success) {
		PX4_ERR("dmadiag: enable failed");
		dma_mgr_release_resource(&res);
		return 1;
	}

	uint32_t spin = 0;

	while (spin++ < 5000000u && diag_tc_count == 0) {
	}

	const bool tc = (diag_tc_count != 0);

	dma_mgr_disable_channel(&res);
	dma_mgr_release_resource(&res);
	pwmv2_disable_dma_at_reload_point(PWM_BASE, pwm_dma_0);
	pwmv2_disable_counter(PWM_BASE, pwm_counter_0);
	pwmv2_channel_disable_output(PWM_BASE, pwm_channel_0);
	pwmv2_channel_disable_output(PWM_BASE, pwm_channel_1);

	PX4_INFO("dmadiag: %s, %u Hz (reload %u), %u words, %s ch %u, tc %d",
		 dest_name, freq_hz, (unsigned)reload, words, use_xdma ? "XDMA" : "HDMA",
		 (unsigned)res.channel, tc ? 1 : 0);

	if (to_ram) {
		unsigned ok = 0;

		for (unsigned i = 0; i < words; i++) {
			if (diag_dst[i] == diag_src[i]) {
				ok++;
			}
		}

		PX4_INFO("dmadiag: ram match %u/%u, dst[0..3] %u %u %u %u", ok, words,
			 (unsigned)diag_dst[0], (unsigned)diag_dst[1],
			 (unsigned)diag_dst[2], (unsigned)diag_dst[3]);
	}

	return tc ? 0 : 1;
}
extern "C" int pwmv2test_main(int argc, char *argv[])
{
	uint32_t frequency = 1000;
	unsigned duty_percent = 50;
	unsigned pairs = 30;
	bool loop = false;
	bool dshot = false;
	bool dshotbuf = false;
	bool dmadiag = false;
	const char *dmadiag_dest = "ram";
	unsigned dshot_rate = 600;
	unsigned dshot_throttle = 0;
	unsigned dshot_frames = 3000;   /* duration, ms */

	if (argc > 1 && strcmp(argv[1], "dmadiag") == 0) {
		dmadiag = true;

		if (argc > 2) {
			dshot_rate = strtoul(argv[2], nullptr, 0);
		}

		if (argc > 3) {
			dshot_throttle = strtoul(argv[3], nullptr, 0);
		}

		if (argc > 4) {
			dmadiag_dest = argv[4];
		}

	} else if (argc > 1 && strcmp(argv[1], "dshotbuf") == 0) {
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


	clock_add_to_group(clock_pwm1, 0);

	const uint32_t clock = clock_get_frequency(clock_pwm1);

	if (clock == 0) {
		PX4_ERR("PWM1 clock is not running");
		return 1;
	}

	if (dmadiag) {
		return dmadiag_run(clock, dshot_rate, dshot_throttle, dmadiag_dest);
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
