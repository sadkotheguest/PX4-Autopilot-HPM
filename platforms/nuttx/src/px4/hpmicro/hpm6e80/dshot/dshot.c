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
 * @file dshot.c
 *
 * HPM6E00 DShot output on PWMv2.
 *
 * The API follows the STM32 port (stm32_common/dshot/dshot.c):
 *   up_dshot_init()           - one-time setup
 *   dshot_motor_data_set()    - prepare the frame for one motor
 *   up_dshot_trigger()        - send one frame
 *   up_dshot_arm()            - enable/disable the outputs
 *
 * A frame is a single 18-word DMA transfer into the PWM shadow registers
 * (bit 1..15 + 3 idle words; bit 0 is preloaded by software).  One DMA
 * request is generated per PWM period (counter reload), so one word is
 * written per bit.  The transfer-complete callback stops the counter.
 *
 * Hardware notes (learned during the bring-up):
 *  - the duty shadow slot must be unlocked for DMA (UNLOCK bit = 2 + slot);
 *  - the DMA channel must be allocated through dma_mgr, otherwise its TC
 *    interrupt is not acknowledged by dma_mgr_isr_handler() and the board
 *    hangs in an interrupt storm;
 *  - the frame buffer lives in .ahb_sram, which is non-cacheable.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <errno.h>
#include <string.h>

#include "hpm_clock_drv.h"
#include "hpm_dma_mgr.h"
#include "hpm_dmamux_src.h"
#include "hpm_dmav2_drv.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"

#define DSHOT_FRAME_WORDS   18u   /* bit1..bit15 + 3 idle words */

static PWMV2_Type *const pwm = HPM_PWM1;

static const pwm_counter_t counter = pwm_counter_0;
static const pwm_channel_t channel0 = pwm_channel_0;
static const pwm_channel_t channel1 = pwm_channel_1;
static const pwm_dma_chn_t dma_chn = pwm_dma_0;

static const uint8_t cmp_duty0 = 0;
static const uint8_t cmp_edge0 = 1;
static const uint8_t cmp_duty1 = 2;
static const uint8_t cmp_edge1 = 3;

static const uint8_t sh_period = PWMV2_SHADOW_INDEX(0);
static const uint8_t sh_duty = PWMV2_SHADOW_INDEX(20);
static const uint8_t sh_edge = PWMV2_SHADOW_INDEX(2);

/* Non-cacheable DMA source buffers: AHB SRAM and the NONCACHEABLE_RAM region.
 * The region is selected with up_dshot_set_buffer_region() before init.
 */
static uint32_t frame_buf_ahb[DSHOT_FRAME_WORDS] __attribute__((section(".ahb_sram")));
static uint32_t frame_buf_nc[DSHOT_FRAME_WORDS] __attribute__((section(".noncacheable.bss")));

static uint32_t *frame_buf = frame_buf_ahb;

void up_dshot_set_buffer_region(int region)
{
	frame_buf = region ? frame_buf_nc : frame_buf_ahb;
}

static uint32_t reload;
static uint16_t frame_bits;
static uint8_t channel_mask;
static bool initialized;
static bool armed;
static volatile uint32_t tc_count;

static dma_resource_t dma_res;
static dma_mgr_chn_conf_t dma_conf;
static bool dma_allocated;

static uint32_t dshot_cmp(bool bit)
{
	/* Comparator 0 counts down from the period: 75 % for "1", 37.5 % for "0". */
	const uint32_t duty = bit ? (reload * 3u / 4u) : (reload * 3u / 8u);

	return reload - duty;
}

static void dshot_build_buffer(void)
{
	for (int i = 1; i < 16; i++) {
		frame_buf[i - 1] = dshot_cmp((frame_bits >> (15 - i)) & 1) << 8;
	}

	for (int i = 15; i < (int)DSHOT_FRAME_WORDS; i++) {
		frame_buf[i] = reload << 8;
	}
}

static void dshot_tc_cb(DMA_Type *base, uint32_t channel, void *cb_data)
{
	(void)base;
	(void)channel;
	(void)cb_data;

	pwmv2_disable_counter(pwm, counter);
	tc_count++;
}

static void dshot_pins(void)
{
	HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
	HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
	HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
}

static void dshot_pwm_init(void)
{
	pwmv2_deinit(pwm);

	/* The DMA writes the duty shadow: unlock that slot (bit = 2 + slot). */
	pwm->UNLOCK |= (1u << (2u + sh_duty));

	pwmv2_shadow_register_unlock(pwm);
	pwmv2_set_shadow_val(pwm, sh_period, reload, 0, false);
	pwmv2_set_shadow_val(pwm, sh_duty, reload, 0, false);
	pwmv2_set_shadow_val(pwm, sh_edge, reload + 1, 0, false);

	pwmv2_counter_select_data_offset_from_shadow_value(pwm, counter, sh_period);
	pwmv2_counter_burst_disable(pwm, counter);
	pwmv2_set_reload_update_time(pwm, counter, pwm_reload_update_on_shlk);

	pwmv2_select_cmp_source(pwm, cmp_duty0, cmp_value_from_shadow_val, sh_duty);
	pwmv2_select_cmp_source(pwm, cmp_edge0, cmp_value_from_shadow_val, sh_edge);
	pwmv2_select_cmp_source(pwm, cmp_duty1, cmp_value_from_shadow_val, sh_duty);
	pwmv2_select_cmp_source(pwm, cmp_edge1, cmp_value_from_shadow_val, sh_edge);

	pwmv2_shadow_register_lock(pwm);
	pwmv2_issue_shadow_register_lock_event(pwm);

	pwmv2_disable_four_cmp(pwm, channel0);
}

static int dshot_dma_init(void)
{
	trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM1_REQ0);

	dma_mgr_init();

	if (dma_mgr_request_resource(&dma_res) != status_success) {
		return -ENODEV;
	}

	dma_allocated = true;

	dma_mgr_chn_conf_t conf;
	dma_mgr_get_default_chn_config(&conf);
	conf.en_dmamux = true;
	conf.dmamux_src = HPM_DMA_SRC_MOT_0;
	conf.src_mode = 0;
	conf.dst_mode = 1;                        /* one word per PWM request */
	conf.src_width = DMA_TRANSFER_WIDTH_WORD;
	conf.dst_width = DMA_TRANSFER_WIDTH_WORD;
	conf.src_addr_ctrl = 0;                   /* increment */
	conf.dst_addr_ctrl = 2;                   /* fixed: the duty shadow */
	conf.src_addr = (uint32_t)frame_buf;
	conf.dst_addr = (uint32_t)&pwm->SHADOW_VAL[sh_duty];
	conf.size_in_byte = DSHOT_FRAME_WORDS * sizeof(uint32_t);
	conf.en_infiniteloop = false;
	conf.handshake_opt = DMA_HANDSHAKE_OPT_ONE_BURST;
	conf.interrupt_mask = DMA_INTERRUPT_MASK_ALL & ~DMA_INTERRUPT_MASK_TERMINAL_COUNT;

	memcpy(&dma_conf, &conf, sizeof(dma_conf));

	if (dma_mgr_setup_channel(&dma_res, &dma_conf) != status_success) {
		return -EIO;
	}

	if (dma_mgr_install_chn_tc_callback(&dma_res, dshot_tc_cb, NULL) != status_success) {
		return -EIO;
	}

	/* The request itself is enabled once; it only fires while the counter runs. */
	pwmv2_enable_dma_at_reload_point(pwm, dma_chn, counter);

	return OK;
}

int up_dshot_init(uint32_t channel_mask_in, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	(void)enable_bidirectional_dshot;   /* bidirectional telemetry is not implemented yet */

	if (dshot_pwm_freq == 0) {
		return -EINVAL;
	}

	if (initialized) {
		return OK;
	}

	channel_mask = channel_mask_in ? (channel_mask_in & 0x3) : 0x1;

	if (channel_mask == 0) {
		return -EINVAL;
	}

	dshot_pins();

	clock_add_to_group(clock_pwm1, 0);
	clock_add_to_group(clock_hdma, 0);

	const uint32_t clock = clock_get_frequency(clock_pwm1);

	if (clock == 0) {
		return -EIO;
	}

	reload = clock / dshot_pwm_freq - 1;

	dshot_pwm_init();
	dshot_build_buffer();

	const int ret = dshot_dma_init();

	if (ret != OK) {
		return ret;
	}

	initialized = true;
	armed = false;

	return OK;
}

void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry)
{
	(void)channel;   /* both outputs share the same frame for now */

	if (!initialized) {
		return;
	}

	const uint16_t packet = (uint16_t)((throttle << 1) | (telemetry ? 1u : 0u));
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;

	frame_bits = (uint16_t)((packet << 4) | crc);

	dshot_build_buffer();
}

int up_dshot_arm(bool arm)
{
	if (!initialized) {
		return -EINVAL;
	}

	armed = arm;

	if (arm) {
		pwmv2_channel_enable_output(pwm, channel0);
		pwmv2_channel_enable_output(pwm, channel1);

	} else {
		pwmv2_channel_disable_output(pwm, channel0);
		pwmv2_channel_disable_output(pwm, channel1);
	}

	return OK;
}

void up_dshot_trigger(void)
{
	if (!initialized || !armed) {
		return;
	}

	/* Preload bit 0: the DMA writes the value for the next period. */
	pwmv2_shadow_register_unlock(pwm);
	pwmv2_set_shadow_val(pwm, sh_duty, dshot_cmp((frame_bits >> 15) & 1), 0, false);
	pwmv2_shadow_register_lock(pwm);

	/* Drop any stale completion status before arming the next frame. */
	dma_clear_transfer_status(dma_res.base, dma_res.channel);

	if (dma_mgr_setup_channel(&dma_res, &dma_conf) != status_success) {
		return;
	}

	if (dma_mgr_enable_channel(&dma_res) != status_success) {
		return;
	}

	pwmv2_reset_counter(pwm, counter);
	pwmv2_enable_counter(pwm, counter);
	pwmv2_start_pwm_output(pwm, counter);
}

uint32_t up_dshot_tc_count(void)
{
	return tc_count;
}

void up_dshot_tc_reset(void)
{
	tc_count = 0;

	if (dma_allocated) {
		dma_clear_transfer_status(dma_res.base, dma_res.channel);
	}
}

void up_dshot_deinit(void)
{
	if (!initialized) {
		return;
	}

	pwmv2_disable_dma_at_reload_point(pwm, dma_chn);
	pwmv2_disable_counter(pwm, counter);
	pwmv2_channel_disable_output(pwm, channel0);
	pwmv2_channel_disable_output(pwm, channel1);

	if (dma_allocated) {
		dma_mgr_disable_channel(&dma_res);
		dma_mgr_release_resource(&dma_res);
		dma_allocated = false;
	}

	initialized = false;
	armed = false;
}

/* Bidirectional DShot is not implemented yet; keep the API consistent. */
int up_bdshot_num_erpm_ready(void) { return 0; }
int up_bdshot_get_erpm(uint8_t channel, int *erpm) { (void)channel; if (erpm) { *erpm = 0; } return -ENOSYS; }
int up_bdshot_channel_status(uint8_t channel) { (void)channel; return 0; }
void up_bdshot_status(void) { }