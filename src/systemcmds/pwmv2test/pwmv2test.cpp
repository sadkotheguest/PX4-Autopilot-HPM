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

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include "hpm_pwmv2_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_soc_irq.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"
#include "hpm_dma_mgr.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_src.h"
#include "hpm_dmamux_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "hpm_gptmr_drv.h"

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

/* DMA-visible buffer: AHB SRAM, like the SDK pwmv2_cal_shared_delta example. */
static uint32_t dshot_buf[DSHOT_FRAME_WORDS] __attribute__((section(".ahb_sram")));

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

/* Harness for the arch_dshot API (same functions as the STM32 port). */

extern "C" int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot);
extern "C" void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry);
extern "C" int up_dshot_arm(bool armed);
extern "C" void up_dshot_trigger(void);
extern "C" uint32_t up_dshot_tc_count(void);
extern "C" unsigned up_dshot_frame_words(void);
extern "C" unsigned up_dshot_num_channels(void);
extern "C" void up_dshot_tc_reset(void);
extern "C" void up_dshot_deinit(void);
extern "C" void up_dshot_set_buffer_region(int region);

static int dshot_run(uint32_t clock, unsigned rate_kbit, unsigned throttle_percent, unsigned frames, bool region_nc)
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

	up_dshot_set_buffer_region(region_nc ? 1 : 0);

	if (up_dshot_init(0xF, rate_hz, false) != OK) {
		PX4_ERR("up_dshot_init failed");
		return 1;
	}

	uint32_t throttle = 48u + throttle_percent * 2000u / 100u;

	if (throttle > 2047u) {
		throttle = 2047u;
	}

	/* Different throttle per channel to verify independence. */
	const uint16_t thr[4] = {
		(uint16_t)throttle,
		(uint16_t)(2047u - throttle),
		(uint16_t)(throttle / 2u),
		(uint16_t)(2047u - throttle / 2u)
	};

	for (unsigned ch = 0; ch < up_dshot_num_channels(); ch++) {
		dshot_motor_data_set(ch, thr[ch], false);
	}

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

	PX4_INFO("dshot %u kbit/s, %u ch, thr %u/%u/%u/%u, %u words, frames %u/%u",
		 rate_kbit, up_dshot_num_channels(), (unsigned)thr[0], (unsigned)thr[1],
		 (unsigned)thr[2], (unsigned)thr[3], up_dshot_frame_words(), complete, sent);
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
/* Slow GPIO loopback: drive PE08 as GPIO, read PE09 (jumper). */
static int gpiotest_run(void)
{
	HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_GPIO_E_08;
	HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_GPIO_E_09;
	HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);

	HPM_GPIO0->OE[4].SET = 1u << 8;
	HPM_GPIO0->OE[4].CLEAR = 1u << 9;

	unsigned low = 0, high = 0;

	for (unsigned i = 0; i < 200u; i++) {
		HPM_GPIO0->DO[4].CLEAR = 1u << 8;
		usleep(20);

		if (((HPM_GPIO0->DI[4].VALUE >> 9) & 1u) == 0u) {
			low++;
		}

		HPM_GPIO0->DO[4].SET = 1u << 8;
		usleep(20);

		if (((HPM_GPIO0->DI[4].VALUE >> 9) & 1u) == 1u) {
			high++;
		}
	}

	HPM_GPIO0->DO[4].CLEAR = 1u << 8;

	PX4_INFO("gpiotest: PE09 low %u/200 high %u/200, DI=0x%08x DO=0x%08x OE=0x%08x",
		 low, high, (unsigned)HPM_GPIO0->DI[4].VALUE,
		 (unsigned)HPM_GPIO0->DO[4].VALUE, (unsigned)HPM_GPIO0->OE[4].VALUE);

	return (low == 200u && high == 200u) ? 0 : 1;
}
/* ------------------------------------------------------------------------ */
/* Isolated compare-point DMA test: cmp0 matches -> DMA copies the counter.  */
/* ------------------------------------------------------------------------ */

static uint32_t cmpdma_buf[32] __attribute__((section(".ahb_sram")));
static volatile uint32_t cmpdma_tc;

static void cmpdma_tc_cb(DMA_Type *base, uint32_t channel, void *cb_data)
{
	(void)base;
	(void)channel;
	(void)cb_data;
	cmpdma_tc++;
}

static int cmpdma_run(unsigned out_enable, unsigned cmp_index)
{
	clock_add_to_group(clock_pwm1, 0);
	clock_add_to_group(clock_hdma, 0);

	const uint32_t reload = 1000u;   /* 200 MHz / 1000 = 200 kHz */

	pwmv2_deinit(PWM_BASE);
	pwmv2_shadow_register_unlock(PWM_BASE);
	pwmv2_set_shadow_val(PWM_BASE, PWMV2_SHADOW_INDEX(0), reload, 0, false);
	pwmv2_set_shadow_val(PWM_BASE, PWMV2_SHADOW_INDEX(1), reload / 2u, 0, false);
	pwmv2_counter_select_data_offset_from_shadow_value(PWM_BASE, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
	pwmv2_counter_burst_disable(PWM_BASE, pwm_counter_0);
	pwmv2_set_reload_update_time(PWM_BASE, pwm_counter_0, pwm_reload_update_on_shlk);

	pwmv2_cmp_config_t cmp = {};
	cmp.cmp = reload / 2u;
	cmp.cmp_source = cmp_value_from_shadow_val;
	cmp.cmp_source_index = PWMV2_SHADOW_INDEX(1);
	cmp.update_trigger = pwm_shadow_register_update_on_reload;
	pwmv2_config_cmp(PWM_BASE, (uint8_t)cmp_index, &cmp);

	pwmv2_shadow_register_lock(PWM_BASE);
	pwmv2_issue_shadow_register_lock_event(PWM_BASE);

	HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
	HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	pwmv2_disable_four_cmp(PWM_BASE, pwm_channel_0);

	if (out_enable) {
		pwmv2_channel_enable_output(PWM_BASE, pwm_channel_0);
	}

	memset(cmpdma_buf, 0, sizeof(cmpdma_buf));

	trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_1, HPM_TRGM0_DMA_SRC_PWM1_REQ1);

	dma_mgr_init();

	dma_resource_t res;

	if (dma_mgr_request_resource(&res) != status_success) {
		PX4_ERR("cmpdma: no dma resource");
		return 1;
	}

	dma_mgr_chn_conf_t conf;
	dma_mgr_get_default_chn_config(&conf);
	conf.en_dmamux = true;
	conf.dmamux_src = HPM_DMA_SRC_MOT_1;
	conf.src_mode = 1;
	conf.dst_mode = 0;
	conf.src_width = DMA_TRANSFER_WIDTH_WORD;
	conf.dst_width = DMA_TRANSFER_WIDTH_WORD;
	conf.src_addr_ctrl = 2;                  /* fixed: the counter */
	conf.dst_addr_ctrl = 0;
	conf.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
	conf.src_addr = (uint32_t)&PWM_BASE->CNT_VAL[0];
	conf.dst_addr = (uint32_t)cmpdma_buf;
	conf.size_in_byte = sizeof(cmpdma_buf);
	conf.en_infiniteloop = false;
	conf.handshake_opt = DMA_HANDSHAKE_OPT_ONE_BURST;
	conf.interrupt_mask = DMA_INTERRUPT_MASK_ALL & ~DMA_INTERRUPT_MASK_TERMINAL_COUNT;

	if (dma_mgr_setup_channel(&res, &conf) != status_success) {
		PX4_ERR("cmpdma: setup failed");
		return 1;
	}

	dma_mgr_install_chn_tc_callback(&res, cmpdma_tc_cb, NULL);

	pwmv2_enable_dma_at_compare_point(PWM_BASE, pwm_dma_1, (uint8_t)cmp_index);
	const uint32_t dma_en = PWM_BASE->DMA_EN;

	cmpdma_tc = 0;
	dma_clear_transfer_status(res.base, res.channel);

	if (dma_mgr_enable_channel(&res) != status_success) {
		PX4_ERR("cmpdma: enable failed");
	}

	pwmv2_reset_counter(PWM_BASE, pwm_counter_0);
	pwmv2_enable_counter(PWM_BASE, pwm_counter_0);
	pwmv2_start_pwm_output(PWM_BASE, pwm_counter_0);

	uint32_t spin = 0;

	while (spin++ < 2000000u && cmpdma_tc == 0u) {
	}

	pwmv2_disable_counter(PWM_BASE, pwm_counter_0);
	pwmv2_disable_dma_at_compare_point(PWM_BASE, pwm_dma_1);
	dma_mgr_disable_channel(&res);
	dma_mgr_release_resource(&res);

	unsigned nz = 0;

	for (unsigned i = 0; i < 32u; i++) {
		if (cmpdma_buf[i] != 0u) {
			nz++;
		}
	}

	PX4_INFO("cmpdma: cmp %u out_enable %u, DMA_EN 0x%08x, tc %u, non-zero %u/32, buf %u %u %u %u",
		 cmp_index, out_enable, (unsigned)dma_en, (unsigned)cmpdma_tc, nz,
		 (unsigned)cmpdma_buf[0], (unsigned)cmpdma_buf[1],
		 (unsigned)cmpdma_buf[2], (unsigned)cmpdma_buf[3]);

	return (nz > 0u) ? 0 : 1;
}
/* ------------------------------------------------------------------------ */
/* Dump the PWMv2 register state after arming a DShot frame.                */
/* ------------------------------------------------------------------------ */

static int pwmregs_run(unsigned rate_kbit)
{
	up_dshot_deinit();

	if (up_dshot_init(0x1, rate_kbit * 1000u, false) != OK) {
		PX4_ERR("pwmregs: dshot init failed");
		return 1;
	}

	dshot_motor_data_set(0, 1048u, false);
	up_dshot_arm(true);
	usleep(1000);

	PX4_INFO("pwmregs: WORK_CTRL0 0x%08x WORK_CTRL1 0x%08x",
		 (unsigned)PWM_BASE->WORK_CTRL0, (unsigned)PWM_BASE->WORK_CTRL1);

	for (unsigned i = 0; i < 4u; i++) {
		const uint32_t cfg = PWM_BASE->CMP[i].CFG;
		PX4_INFO("pwmregs: CMP[%u].CFG 0x%08x update_time %u trig_sel %u",
			 i, (unsigned)cfg,
			 (unsigned)((cfg >> 24) & 0x7u),
			 (unsigned)((cfg >> 28) & 0x7u));
	}

	PX4_INFO("pwmregs: CNT[0].CFG0 0x%08x CFG1 0x%08x CFG2 0x%08x",
		 (unsigned)PWM_BASE->CNT[0].CFG0, (unsigned)PWM_BASE->CNT[0].CFG1,
		 (unsigned)PWM_BASE->CNT[0].CFG2);

	for (unsigned i = 0; i < 4u; i++) {
		PX4_INFO("pwmregs: SHADOW[%u] 0x%08x", i, (unsigned)PWM_BASE->SHADOW_VAL[i]);
	}

	up_dshot_arm(false);
	return 0;
}
/* ------------------------------------------------------------------------ */
/* Fully independent TX check: PE09 as GPIO input, CPU polls the level and   */
/* timestamps edges with a free-running GPTMR counter (no capture, no DMA).  */
/* ------------------------------------------------------------------------ */

#define POLLSCAN_EDGES 64u

static int pollscan_run(unsigned rate_kbit, unsigned throttle_percent)
{
	up_dshot_deinit();

	if (up_dshot_init(0x1, rate_kbit * 1000u, false) != OK) {
		PX4_ERR("pollscan: dshot init failed");
		return 1;
	}

	const uint16_t packet = (uint16_t)(((48u + throttle_percent * 2000u / 100u) << 1) & 0xFFFFu);
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
	const uint16_t sent = (uint16_t)((packet << 4) | crc);
	const uint32_t throttle = 48u + throttle_percent * 2000u / 100u;

	/* Free-running 100 MHz counter for timestamps (plain counter mode). */
	clock_add_to_group(clock_gptmr1, 0);

	gptmr_channel_config_t cfg;
	gptmr_channel_get_default_config(HPM_GPTMR1, &cfg);
	cfg.mode = gptmr_work_mode_no_capture;
	gptmr_channel_config(HPM_GPTMR1, 0u, &cfg, false);
	gptmr_channel_reset_count(HPM_GPTMR1, 0u);
	gptmr_start_counter(HPM_GPTMR1, 0u);

	/* PE09 as plain GPIO input. */
	HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
	HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);

	dshot_motor_data_set(0, (uint16_t)throttle, false);
	up_dshot_arm(true);
	usleep(100);

	uint32_t ts[POLLSCAN_EDGES];
	uint8_t lv[POLLSCAN_EDGES];
	unsigned edges = 0;

	up_dshot_trigger();

	unsigned last = (HPM_GPIO0->DI[GPIO_DI_GPIOE].VALUE >> 9) & 1u;

	for (uint32_t i = 0; i < 4000000u && edges < POLLSCAN_EDGES; i++) {
		const unsigned now = (HPM_GPIO0->DI[GPIO_DI_GPIOE].VALUE >> 9) & 1u;

		if (now != last) {
			ts[edges] = HPM_GPTMR1->CHANNEL[0].CNT;
			lv[edges] = (uint8_t)now;
			edges++;
			last = now;
		}
	}

	up_dshot_arm(false);
	gptmr_stop_counter(HPM_GPTMR1, 0u);

	const uint32_t bit_ticks = 100000000u / (rate_kbit * 1000u);
	uint16_t frame = 0;
	unsigned bits = 0;
	uint32_t rise = 0;
	bool have_rise = false;

	for (unsigned i = 0; i < edges && bits < 16u; i++) {
		if (lv[i] == 1u) {
			rise = ts[i];
			have_rise = true;

		} else if (have_rise) {
			const uint32_t width = ts[i] - rise;
			have_rise = false;
			frame = (uint16_t)((frame << 1) | ((width > (bit_ticks / 2u)) ? 1u : 0u));
			bits++;
		}
	}

	PX4_INFO("pollscan: %u kbit/s, sent 0x%04x polled 0x%04x (%u bits, %u edges) -> %s",
		 rate_kbit, (unsigned)sent, (unsigned)frame, bits, edges,
		 (bits == 16u && frame == sent) ? "MATCH" : "MISMATCH");

	for (unsigned i = 0; i + 1u < edges && i < 8u; i += 2u) {
		PX4_INFO("pollscan: pulse %u rise %u fall %u width %u", i / 2u,
			 (unsigned)ts[i], (unsigned)ts[i + 1u], (unsigned)(ts[i + 1u] - ts[i]));
	}

	return (bits == 16u && frame == sent) ? 0 : 1;
}
/* ------------------------------------------------------------------------ */
/* XDMA pair receiver: one (CAPPRD, CAPDTY) 64-bit read per capture event    */
/* ------------------------------------------------------------------------ */

extern "C" int up_bdshot_rx_init(uint32_t channel_mask);
extern "C" void up_bdshot_rx_deinit(void);
extern "C" void up_bdshot_rx_arm(void);
extern "C" void up_bdshot_rx_flush(void);
extern "C" void up_bdshot_rx_stop(void);
extern "C" bool up_bdshot_rx_done(uint8_t channel);
extern "C" const uint32_t *up_bdshot_rx_buffer(uint8_t channel);
extern "C" unsigned up_bdshot_rx_words(void);
extern "C" unsigned up_bdshot_rx_events(uint8_t channel);
extern "C" int up_bdshot_rx_frame(uint8_t channel, uint16_t *frame, unsigned *pairs_out);
extern "C" int up_bdshot_rx_wait(uint8_t channel, uint32_t timeout_us, uint16_t *frame);
extern "C" void up_dshot_idle(void);

static bool gcapx_quiet;

static int gcapx_run(unsigned rate_kbit, unsigned throttle_percent)
{
	up_bdshot_rx_deinit();
	up_dshot_deinit();

	if (up_dshot_init(0x1, rate_kbit * 1000u, false) != OK) {
		PX4_ERR("gcapx: dshot init failed");
		return 1;
	}

	if (up_bdshot_rx_init(0x1) != OK) {
		PX4_ERR("gcapx: rx init failed");
		return 1;
	}

	up_dshot_arm(true);
	up_bdshot_rx_arm();
	usleep(200);

	uint32_t throttle = 48u + throttle_percent * 2000u / 100u;

	if (throttle > 2047u) {
		throttle = 2047u;
	}

	const uint16_t packet = (uint16_t)((throttle << 1) & 0xFFFFu);
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
	const uint16_t sent = (uint16_t)((packet << 4) | crc);

	dshot_motor_data_set(0, (uint16_t)throttle, false);
	up_bdshot_rx_flush();
	usleep(100);
	up_dshot_trigger();

	/* Blocking receive: the decoder filters, assembles and CRC-checks the
	 * captured window; the harness only compares the returned frame.
	 */
	uint16_t frame = 0;
	const int rc = up_bdshot_rx_wait(0, 5000u, &frame);

	up_bdshot_rx_stop();

	/* TX stays armed (push-pull) between frames; force the idle low level from
	 * thread context so the next frame starts from a defined state.
	 */
	up_dshot_idle();

	const uint32_t *buf = up_bdshot_rx_buffer(0);
	const unsigned words = up_bdshot_rx_words();

	const bool match = (rc == OK) && (frame == sent);

	if (!gcapx_quiet) {
		PX4_INFO("gcapx: %u kbit/s, thr %u, sent 0x%04x recv 0x%04x (rc %d) -> %s",
		 rate_kbit, (unsigned)throttle, (unsigned)sent, (unsigned)frame, rc,
		 match ? "MATCH" : "MISMATCH");
	}

	if (!match) {
		PX4_INFO("gcapx: FAILED frame (rc %d), full pair dump:", rc);

		for (unsigned i = 0; i + 1u < words; i += 2u) {
			PX4_INFO("gcapx: [%2u] prd %6u dty %6u", i / 2u, (unsigned)buf[i], (unsigned)buf[i + 1u]);
		}

	} else if (!gcapx_quiet) {
		for (unsigned i = 0; i + 1u < words && i < 8u; i += 2u) {
			PX4_INFO("gcapx: [%u] prd %u dty %u", i / 2u, (unsigned)buf[i], (unsigned)buf[i + 1u]);
		}
	}

	return match ? 0 : 1;
}
/* ------------------------------------------------------------------------ */
/* XDMA sweep over 150/300/600 kbit and all setpoints                        */
/* ------------------------------------------------------------------------ */

static int gcapxsweep_run(unsigned runs)
{
	static const unsigned rates[] = {150u, 300u, 600u};
	static const unsigned throttles[] = {0u, 10u, 25u, 50u, 75u, 90u, 100u};
	unsigned total_pass = 0, total_frames = 0;

	gcapx_quiet = true;

	for (unsigned r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
		unsigned pass = 0, frames = 0;

		for (unsigned run = 0; run < runs; run++) {
			for (unsigned k = 0; k < sizeof(throttles) / sizeof(throttles[0]); k++) {
				if (gcapx_run(rates[r], throttles[k]) == 0) {
					pass++;
				}

				frames++;
			}
		}

		PX4_INFO("gcapxsweep: %3u kbit/s : %u/%u MATCH", rates[r], pass, frames);
		total_pass += pass;
		total_frames += frames;
	}

	gcapx_quiet = false;

	PX4_INFO("gcapxsweep: total %u/%u MATCH", total_pass, total_frames);

	return (total_pass == total_frames) ? 0 : 1;
}
/* ------------------------------------------------------------------------ */
/* Repeat one frame until it fails, then dump that capture                   */
/* ------------------------------------------------------------------------ */

static int gcapxloop_run(unsigned rate_kbit, unsigned throttle_percent, unsigned max)
{
	gcapx_quiet = true;

	for (unsigned i = 0; i < max; i++) {
		const int rc = gcapx_run(rate_kbit, throttle_percent);

		if (rc != 0) {
			gcapx_quiet = false;
			PX4_INFO("gcapxloop: failure at iteration %u (of %u)", i, max);
			(void)gcapx_run(rate_kbit, throttle_percent);
			PX4_INFO("gcapxloop: rerun above is the next frame; the failing dump was printed by iteration %u", i);
			return 1;
		}
	}

	gcapx_quiet = false;
	PX4_INFO("gcapxloop: %u frames, all MATCH", max);

	return 0;
}
/* ------------------------------------------------------------------------ */
/* Interrupted frame: the blocking receive must time out, then recover       */
/* ------------------------------------------------------------------------ */

static int gcapxtimeout_run(void)
{
	const unsigned rate_kbit = 600u;
	const uint32_t timeout_us = 500u;
	const uint32_t frame_timeout_us = 5000u;
	int failures = 0;

	up_bdshot_rx_deinit();
	up_dshot_deinit();

	if (up_dshot_init(0x1, rate_kbit * 1000u, false) != OK) {
		PX4_ERR("gcapxtimeout: dshot init failed");
		return 1;
	}

	if (up_bdshot_rx_init(0x1) != OK) {
		PX4_ERR("gcapxtimeout: rx init failed");
		return 1;
	}

	up_dshot_arm(true);
	up_bdshot_rx_arm();
	usleep(200);
	up_bdshot_rx_flush();
	usleep(100);

	/* Case 1: nothing on the wire.  The TX API has no way to arm a partial DMA
	 * chain, so the interrupted frame is represented by the absence of a frame:
	 * the blocking wait must return -ETIMEDOUT instead of hanging.
	 */
	uint16_t frame = 0;
	hrt_abstime t0 = hrt_absolute_time();
	const int rc_timeout = up_bdshot_rx_wait(0, timeout_us, &frame);
	const uint32_t us_timeout = (uint32_t)hrt_elapsed_time(&t0);
	const bool timeout_ok = (rc_timeout == -ETIMEDOUT) && (us_timeout >= (timeout_us / 2u));
	PX4_INFO("gcapxtimeout: no frame: wait rc %d after %u us (expect -ETIMEDOUT) -> %s",
		 rc_timeout, (unsigned)us_timeout, timeout_ok ? "OK" : "FAIL");

	if (!timeout_ok) {
		failures++;
	}

	/* Case 2: the receiver must still accept a normal frame after the timeout. */
	const uint16_t throttle = 1048u;
	const uint16_t packet = (uint16_t)((throttle << 1) & 0xFFFFu);
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
	const uint16_t sent = (uint16_t)((packet << 4) | crc);

	up_bdshot_rx_flush();
	usleep(100);
	dshot_motor_data_set(0, throttle, false);
	up_dshot_trigger();

	frame = 0;
	t0 = hrt_absolute_time();
	const int rc_frame = up_bdshot_rx_wait(0, frame_timeout_us, &frame);
	const uint32_t us_frame = (uint32_t)hrt_elapsed_time(&t0);
	const bool frame_ok = (rc_frame == OK) && (frame == sent);
	PX4_INFO("gcapxtimeout: recovery frame: wait rc %d, sent 0x%04x recv 0x%04x after %u us -> %s",
		 rc_frame, (unsigned)sent, (unsigned)frame, (unsigned)us_frame, frame_ok ? "OK" : "FAIL");

	if (!frame_ok) {
		failures++;
	}

	up_bdshot_rx_stop();
	up_bdshot_rx_deinit();
	up_dshot_idle();

	return (failures == 0) ? 0 : 1;
}
/* ------------------------------------------------------------------------ */
/* Rare-event diagnostic: per-frame capture-event count and raw registers    */
/* ------------------------------------------------------------------------ */

struct gcapxev_pwm_regs {
	uint32_t work1;
	uint32_t reload_w;
	uint32_t cmpval_w;
	uint32_t cnt_w;
	uint32_t shadow20;
	uint32_t cmp0cfg;
};

static void gcapxev_read_pwm_regs(gcapxev_pwm_regs *regs)
{
	regs->work1 = PWM_BASE->WORK_CTRL1;
	regs->reload_w = PWM_BASE->CNT_RELOAD_WORK[0];
	regs->cmpval_w = PWM_BASE->CMP_VAL_WORK[0];
	regs->cnt_w = PWM_BASE->CNT_VAL[0];
	regs->shadow20 = PWM_BASE->SHADOW_VAL[PWMV2_SHADOW_INDEX(20)];
	regs->cmp0cfg = PWM_BASE->CMP[0].CFG;
}

static void gcapxev_print_pwm_regs(const char *when, unsigned idx, const gcapxev_pwm_regs *regs)
{
	PX4_INFO("gcapxev: regs %s f%u WORK1 0x%08x RELOAD_W 0x%08x CMPVAL_W 0x%08x CNT_W 0x%08x SHADOW20 0x%08x CMP0CFG 0x%08x",
		 when, idx, (unsigned)regs->work1, (unsigned)regs->reload_w, (unsigned)regs->cmpval_w,
		 (unsigned)regs->cnt_w, (unsigned)regs->shadow20, (unsigned)regs->cmp0cfg);
}

/* One full receive frame in the gcapx_run() sequence.  Snapshots the PWM
 * working registers right after the trigger and right after the receive stop;
 * "print_now" prints them immediately (first frames), otherwise the caller can
 * print the snapshots later (first anomaly, where the count is only known
 * after the frame).  Returns -1 if the TX/RX init failed (already reported).
 */
static int gcapxev_frame(unsigned rate_kbit, uint32_t throttle, unsigned idx, bool print_now,
			 gcapxev_pwm_regs *regs_trigger, gcapxev_pwm_regs *regs_stop,
			 uint16_t *frame_out, unsigned *events_out)
{
	up_bdshot_rx_deinit();
	up_dshot_deinit();

	if (up_dshot_init(0x1, rate_kbit * 1000u, false) != OK) {
		PX4_ERR("gcapxev: dshot init failed");
		return -1;
	}

	if (up_bdshot_rx_init(0x1) != OK) {
		PX4_ERR("gcapxev: rx init failed");
		return -1;
	}

	up_dshot_arm(true);
	up_bdshot_rx_arm();
	usleep(200);

	dshot_motor_data_set(0, (uint16_t)throttle, false);
	up_bdshot_rx_flush();
	usleep(100);
	up_dshot_trigger();

	if (regs_trigger != nullptr) {
		gcapxev_read_pwm_regs(regs_trigger);

		if (print_now) {
			gcapxev_print_pwm_regs("after trigger", idx, regs_trigger);
		}
	}

	uint16_t frame = 0;
	const int rc = up_bdshot_rx_wait(0, 5000u, &frame);

	up_bdshot_rx_stop();

	if (regs_stop != nullptr) {
		gcapxev_read_pwm_regs(regs_stop);

		if (print_now) {
			gcapxev_print_pwm_regs("after stop", idx, regs_stop);
		}
	}

	up_dshot_idle();

	if (frame_out != nullptr) {
		*frame_out = frame;
	}

	if (events_out != nullptr) {
		*events_out = up_bdshot_rx_events(0);
	}

	return rc;
}
static int gcapxev_run(unsigned rate_kbit, unsigned throttle_percent, unsigned count, unsigned warmup_kbit)
{
	const unsigned expected_events = 16u;

	if (rate_kbit != 150 && rate_kbit != 300 && rate_kbit != 600) {
		PX4_ERR("usage: pwmv2test gcapxev [150|300|600] [throttle_pct] [count] [warmup_kbit]");
		return 1;
	}

	if (warmup_kbit != 0u && warmup_kbit != 150u && warmup_kbit != 300u && warmup_kbit != 600u) {
		PX4_ERR("gcapxev: warmup_kbit must be 0, 150, 300 or 600");
		return 1;
	}

	if (throttle_percent > 100 || count == 0) {
		PX4_ERR("usage: pwmv2test gcapxev [150|300|600] [throttle_pct] [count] [warmup_kbit]");
		return 1;
	}

	const uint32_t gclk = clock_get_frequency(clock_gptmr0);
	const uint32_t bit_ticks = (gclk != 0u) ? (gclk / (rate_kbit * 1000u)) : 0u;
	const uint32_t prd_lo = bit_ticks - (bit_ticks / 10u);
	const uint32_t prd_hi = bit_ticks + (bit_ticks / 10u);

	uint32_t throttle = 48u + throttle_percent * 2000u / 100u;

	if (throttle > 2047u) {
		throttle = 2047u;
	}

	const uint16_t packet = (uint16_t)((throttle << 1) & 0xFFFFu);
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
	const uint16_t sent = (uint16_t)((packet << 4) | crc);

	unsigned histogram[5] = {};     /* event counts 13, 14, 15, 16, 17 */
	unsigned hist_other = 0;
	unsigned anomalies = 0, decode_failures = 0;
	bool dumped = false;

	for (unsigned i = 0; i < count; i++) {
		/* Warmup frame at the other rate: every printed frame then has the
		 * "first frame after a transmit rate change" configuration.
		 */
		if (warmup_kbit != 0u) {
			(void)gcapxev_frame(warmup_kbit, throttle, i, false, nullptr, nullptr, nullptr, nullptr);
		}

		gcapxev_pwm_regs regs_trigger = {}, regs_stop = {};
		uint16_t frame = 0;
		unsigned events = 0;
		const bool print_regs = (i <= 1u);
		const int rc = gcapxev_frame(rate_kbit, throttle, i, print_regs,
					     &regs_trigger, &regs_stop, &frame, &events);

		if (rc == -1) {
			return 1;       /* TX/RX init failed, already reported */
		}

		const uint32_t *buf = up_bdshot_rx_buffer(0);
		const unsigned words = up_bdshot_rx_words();

		/* Period statistics: pairs with prd inside +-10 % of one bit period. */
		unsigned prd_in = 0, prd_out = 0;

		for (unsigned p = 0; p + 1u < words; p += 2u) {
			if (buf[p] == 0u && buf[p + 1u] == 0u) {
				continue;
			}

			if (buf[p] >= prd_lo && buf[p] <= prd_hi) {
				prd_in++;

			} else {
				prd_out++;
			}
		}

		const bool match = (rc == OK) && (frame == sent);

		PX4_INFO("gcapxev: %3u %3u kbit thr %4u ev %2u rc %4d recv 0x%04x sent 0x%04x prd_in %2u prd_out %2u %s",
			 i, rate_kbit, (unsigned)throttle, events, rc, (unsigned)frame, (unsigned)sent,
			 prd_in, prd_out, match ? "MATCH" : "MISMATCH");

		if (!match) {
			decode_failures++;
		}

		if (events >= 13u && events <= 17u) {
			histogram[events - 13u]++;

		} else {
			hist_other++;
		}

		if (events != expected_events) {
			anomalies++;

			if (!dumped) {
				dumped = true;

				PX4_INFO("gcapxev: ANOMALY at frame %u: ev %u (expected %u), full pair dump:",
					 i, events, expected_events);

				for (unsigned p = 0; p + 1u < words; p += 2u) {
					PX4_INFO("gcapxev: [%2u] prd %6u dty %6u", p / 2u,
						 (unsigned)buf[p], (unsigned)buf[p + 1u]);
				}

				const uint32_t sr = gptmr_get_status(HPM_GPTMR0);

				PX4_INFO("gcapxev: GPTMR0_CH3 CNT 0x%08x CAPPRD 0x%08x CAPDTY 0x%08x CAPPOS 0x%08x CAPNEG 0x%08x CR 0x%08x",
					 (unsigned)HPM_GPTMR0->CHANNEL[3].CNT,
					 (unsigned)HPM_GPTMR0->CHANNEL[3].CAPPRD,
					 (unsigned)HPM_GPTMR0->CHANNEL[3].CAPDTY,
					 (unsigned)HPM_GPTMR0->CHANNEL[3].CAPPOS,
					 (unsigned)HPM_GPTMR0->CHANNEL[3].CAPNEG,
					 (unsigned)HPM_GPTMR0->CHANNEL[3].CR);
				PX4_INFO("gcapxev: GPTMR0 SR 0x%08x (ch3 CAP_STAT %u)",
					 (unsigned)sr, (unsigned)(((sr & GPTMR_CH_CAP_STAT_MASK(3)) != 0u) ? 1u : 0u));
				PX4_INFO("gcapxev: PWM1 SHADOW20 0x%08x CMP0_CFG 0x%08x",
					 (unsigned)HPM_PWM1->SHADOW_VAL[PWMV2_SHADOW_INDEX(20)],
					 (unsigned)HPM_PWM1->CMP[0].CFG);

				/* For the first anomaly at frame >= 2 the two snapshots were
				 * taken at the right moments but not printed then (anomaly is
				 * only known after the frame); print them now.
				 */
				if (i > 1u) {
					gcapxev_print_pwm_regs("after trigger", i, &regs_trigger);
					gcapxev_print_pwm_regs("after stop", i, &regs_stop);
				}
			}
		}
	}

	PX4_INFO("gcapxev: %u frames, %u event anomalies, %u decode failures, %u matches",
		 count, anomalies, decode_failures, count - decode_failures);
	PX4_INFO("gcapxev: events histogram e13=%u e14=%u e15=%u e16=%u e17=%u other=%u",
		 histogram[0], histogram[1], histogram[2], histogram[3], histogram[4], hist_other);

	return (anomalies == 0u && decode_failures == 0u) ? 0 : 1;
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
	bool dshot_region_nc = false;
	const char *dmadiag_dest = "ram";
	unsigned dshot_rate = 600;
	unsigned dshot_throttle = 0;
	unsigned dshot_frames = 3000;   /* duration, ms */

	if (argc > 1 && strcmp(argv[1], "gcapxloop") == 0) {
		const unsigned rate = (argc > 2) ? strtoul(argv[2], nullptr, 0) : 150u;
		const unsigned thr = (argc > 3) ? strtoul(argv[3], nullptr, 0) : 50u;
		const unsigned max = (argc > 4) ? strtoul(argv[4], nullptr, 0) : 200u;

		return gcapxloop_run(rate, thr, max);
	}

	if (argc > 1 && strcmp(argv[1], "gcapxsweep") == 0) {
		return gcapxsweep_run((argc > 2) ? strtoul(argv[2], nullptr, 0) : 3u);
	}

	if (argc > 1 && strcmp(argv[1], "gcapxtimeout") == 0) {
		return gcapxtimeout_run();
	}

	if (argc > 1 && strcmp(argv[1], "gcapxev") == 0) {
		const unsigned rate = (argc > 2) ? strtoul(argv[2], nullptr, 0) : 150u;
		const unsigned thr = (argc > 3) ? strtoul(argv[3], nullptr, 0) : 0u;
		const unsigned count = (argc > 4) ? strtoul(argv[4], nullptr, 0) : 100u;
		const unsigned warmup = (argc > 5) ? strtoul(argv[5], nullptr, 0) : 0u;

		return gcapxev_run(rate, thr, count, warmup);
	}

	if (argc > 1 && strcmp(argv[1], "gcapx") == 0) {
		const unsigned rate = (argc > 2) ? strtoul(argv[2], nullptr, 0) : 600u;
		const unsigned thr = (argc > 3) ? strtoul(argv[3], nullptr, 0) : 50u;

		return gcapx_run(rate, thr);
	}

	if (argc > 1 && strcmp(argv[1], "pollscan") == 0) {
		const unsigned rate = (argc > 2) ? strtoul(argv[2], nullptr, 0) : 50u;
		const unsigned thr = (argc > 3) ? strtoul(argv[3], nullptr, 0) : 50u;

		return pollscan_run(rate, thr);
	}

	if (argc > 1 && strcmp(argv[1], "pwmregs") == 0) {
		return pwmregs_run(argc > 2 ? strtoul(argv[2], nullptr, 0) : 150u);
	}

	if (argc > 1 && strcmp(argv[1], "cmpdma") == 0) {
		const unsigned out_en = (argc > 2) ? strtoul(argv[2], nullptr, 0) : 0u;
		const unsigned cmpidx = (argc > 3) ? strtoul(argv[3], nullptr, 0) : 0u;

		return cmpdma_run(out_en, cmpidx);
	}

	if (argc > 1 && strcmp(argv[1], "gpiotest") == 0) {
		return gpiotest_run();
	}

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

		if (argc > 5 && strcmp(argv[5], "nc") == 0) {
			dshot_region_nc = true;
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
		return dshot_run(clock, dshot_rate, dshot_throttle, dshot_frames, dshot_region_nc);
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
