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
 * API follows the STM32 port (stm32_common/dshot/dshot.c):
 *   up_dshot_init()           - one-time setup
 *   dshot_motor_data_set()    - prepare the frame of one motor
 *   up_dshot_trigger()        - send one frame
 *   up_dshot_arm()            - enable/disable the outputs
 *
 * Frame layout mirrors STM32: the buffer is interleaved per motor
 * (buffer[bit * num_channels + channel]) and one PWM request writes the duty
 * shadows of ALL channels with one DMA burst, so a single DMA channel drives
 * the whole timer.  The burst repeats for every bit through a chain of 16
 * linked descriptors (one per bit period); the transfer-complete interrupt of
 * the last descriptor stops the counter.
 *
 * bit15 of every motor is written to the shadow by software before the
 * counter starts, the DMA feeds bit14..bit0 and one idle word.
 *
 * Hardware notes (learned during the bring-up):
 *  - duty shadow slots must be unlocked for DMA (UNLOCK bit = 2 + slot);
 *  - the DMA channel must be allocated through dma_mgr, otherwise its TC
 *    interrupt is not acknowledged by dma_mgr_isr_handler() and the board
 *    hangs in an interrupt storm;
 *  - buffers and descriptors live in non-cacheable memory.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <string.h>

#include <px4_arch/dshot.h>
#include <drivers/drv_dshot.h>

#include "hpm_clock_drv.h"
#include "hpm_soc_irq.h"
#include "hpm_dma_mgr.h"
#include "hpm_dmamux_src.h"
#include "hpm_dmav2_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"

/* 16 bit times per frame: bit15 preloaded, DMA sends bit14..bit0 + idle. */
#define DSHOT_FRAME_WORDS   17u   /* bit15..bit0 + idle (word0 is consumed at arm) */
#define DSHOT_DATA_WORDS    16u

#define DSHOT_MAX_CHANNELS  4u
#define DSHOT_BIT_SLOT0     PWMV2_SHADOW_INDEX(20)   /* duty slots 20..23 */

static PWMV2_Type *const pwm = HPM_PWM1;

static const pwm_counter_t counter = pwm_counter_0;
static const pwm_dma_chn_t dma_chn = pwm_dma_0;

static const pwm_channel_t dshot_channels[DSHOT_MAX_CHANNELS] = {
	pwm_channel_0, pwm_channel_1, pwm_channel_2, pwm_channel_3
};

static const uint8_t sh_period = PWMV2_SHADOW_INDEX(0);
static const uint8_t sh_edge = PWMV2_SHADOW_INDEX(2);

/* Non-cacheable buffer, DMA source; the descriptors are read by the DMA. */
static uint32_t frame_buf_ahb[DSHOT_FRAME_WORDS * DSHOT_MAX_CHANNELS]
	__attribute__((section(".ahb_sram")));
static uint32_t frame_buf_nc[DSHOT_FRAME_WORDS * DSHOT_MAX_CHANNELS]
	__attribute__((section(".noncacheable.bss")));

static dma_mgr_linked_descriptor_t dshot_desc[DSHOT_FRAME_WORDS]
	__attribute__((section(".ahb_sram"))) __attribute__((aligned(8)));

static uint32_t *frame_buf = frame_buf_ahb;

static uint32_t reload;
static uint32_t dshot_bit_hz;     /* bit rate, also needed by the receive decoder */
static uint8_t channel_mask;      /* requested channels */
static uint8_t num_channels;      /* burst size: 1, 2 or 4 */
static uint16_t channel_frame[DSHOT_MAX_CHANNELS];
static bool initialized;
static bool armed;
static volatile uint32_t tc_count;

static dma_resource_t dma_res;
static dma_mgr_chn_conf_t dma_conf;
static bool dma_allocated;

void up_dshot_set_buffer_region(int region)
{
	frame_buf = region ? frame_buf_nc : frame_buf_ahb;
}

void up_dshot_deinit(void);   /* used by up_dshot_init() to apply a new config */

static uint8_t dshot_shadow(unsigned channel)
{
	return DSHOT_BIT_SLOT0 + channel;
}

static uint32_t dshot_cmp(bool bit)
{
	/* Comparator counts down from the period: 75 % for "1", 37.5 % for "0". */
	const uint32_t duty = bit ? (reload * 3u / 4u) : (reload * 3u / 8u);

	return reload - duty;
}

/* Compare value of the DMA request point: early in the counter period, just
 * after the reload.  The counter counts down from `reload`, so a compare value
 * of `reload - reload/8` fires one eighth of a period after the reload, which
 * leaves ~7/8 of the period for the shadow write to land before the next reload
 * latches it into the comparator.  A mid-period point (reload/2) leaves only
 * half a period and made the first bits of a frame VULNERABLE to bus contention:
 * a write that missed the latch repeated the previous bit and produced an extra
 * pulse at the frame start.
 */
static uint32_t dshot_dma_request_point(void)
{
	uint32_t point = reload - (reload / 8u);

	/* keep a few ticks of separation from the reload itself */
	if ((reload > 8u) && (point > (reload - 4u))) {
		point = reload - 4u;
	}

	return point;
}

static void dshot_build_channel(unsigned channel)
{
	if (channel >= num_channels) {
		return;
	}

	/* Word 0 carries bit15: the DMA performs one transfer immediately when the
	 * channel is enabled (before the counter starts), so that transfer places
	 * bit15 into the shadow and the rest of the chain follows on compare events.
	 */
	for (unsigned i = 0; i < DSHOT_DATA_WORDS; i++) {
		frame_buf[i * num_channels + channel] =
			dshot_cmp((channel_frame[channel] >> (15 - i)) & 1) << 8;
	}

	/* Words past the data carry a compare value ABOVE the period: the comparator
	 * never matches, so no pulse is generated and the output stays low.  The last
	 * word written by the DMA is therefore an idle level - no need for Hi-Z and no
	 * need to stop the counter.
	 */
	for (unsigned i = DSHOT_DATA_WORDS; i < DSHOT_FRAME_WORDS; i++) {
		frame_buf[i * num_channels + channel] = (reload + (reload / 2u)) << 8;
	}
}

static void dshot_tc_cb(DMA_Type *base, uint32_t channel, void *cb_data)
{
	(void)base;
	(void)channel;
	(void)cb_data;

	/* Only account for the completion; the PWM counter keeps running with the
	 * trailing idle word so the line stays low until the next re-arm.
	 */
	tc_count++;
}

static void dshot_pins(void)
{
	if (channel_mask & 0x1) {
		HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
		HPM_IOC->PAD[IOC_PAD_PE08].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	}

	if (channel_mask & 0x2) {
		HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
		HPM_IOC->PAD[IOC_PAD_PE09].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	}

	if (channel_mask & 0x4) {
		HPM_IOC->PAD[IOC_PAD_PE10].FUNC_CTL = IOC_PE10_FUNC_CTL_PWM1_P_2;
		HPM_IOC->PAD[IOC_PAD_PE10].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	}

	if (channel_mask & 0x8) {
		HPM_IOC->PAD[IOC_PAD_PE11].FUNC_CTL = IOC_PE11_FUNC_CTL_PWM1_P_3;
		HPM_IOC->PAD[IOC_PAD_PE11].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_SPD_SET(2);
	}
}

static void dshot_pwm_init(void)
{
	pwmv2_deinit(pwm);

	pwmv2_shadow_register_unlock(pwm);

	for (unsigned ch = 0; ch < num_channels; ch++) {
		const uint8_t shadow = dshot_shadow(ch);

		/* DMA writes this shadow: unlock that slot (bit = 2 + slot). */
		pwm->UNLOCK |= (1u << (2u + shadow));

		/* "No pulse" from the very first period: a compare value ABOVE the period
		 * never matches, so the line stays low until the frame data arrives.
		 * Using `reload` here would produce a full-period pulse right after
		 * initialisation and a junk capture event on the first frame.
		 */
		pwmv2_set_shadow_val(pwm, shadow, reload + (reload / 2u), 0, false);

		/* The comparator has to reload its working value from the shadow on
		 * every counter period, otherwise the duty written by the DMA for the
		 * next bit never reaches the output.
		 */
		pwmv2_cmp_config_t cmp = {};
		cmp.cmp = reload + (reload / 2u);
		cmp.cmp_source = cmp_value_from_shadow_val;
		cmp.cmp_source_index = shadow;
		cmp.update_trigger = pwm_shadow_register_update_on_reload;
		pwmv2_config_cmp(pwm, 2 * ch, &cmp);

		pwmv2_cmp_config_t cmp_edge = {};
		cmp_edge.cmp = reload + 1u;
		cmp_edge.cmp_source = cmp_value_from_shadow_val;
		cmp_edge.cmp_source_index = sh_edge;
		cmp_edge.update_trigger = pwm_shadow_register_update_on_reload;
		pwmv2_config_cmp(pwm, 2 * ch + 1, &cmp_edge);

		pwmv2_disable_four_cmp(pwm, dshot_channels[ch]);
	}

	pwmv2_set_shadow_val(pwm, sh_period, reload, 0, false);
	pwmv2_set_shadow_val(pwm, sh_edge, reload + 1, 0, false);

	/* Trigger comparator for the DMA request.  The point sits EARLY in the counter
	 * period (just after the reload), not in the middle: every duty-shadow write
	 * has to reach the shadow before the next reload latches it, and under bus
	 * contention a half-period window was not always enough - a write that missed
	 * its deadline repeated the previous bit on the wire and produced an extra
	 * pulse at the frame start.  Early request leaves ~7/8 of the period.
	 */
	const uint8_t sh_dma_trig = PWMV2_SHADOW_INDEX(4);
	const uint32_t request_point = dshot_dma_request_point();
	pwmv2_set_shadow_val(pwm, sh_dma_trig, request_point, 0, false);

	pwmv2_cmp_config_t cmp_trig = {};
	cmp_trig.cmp = request_point;
	cmp_trig.cmp_source = cmp_value_from_shadow_val;
	cmp_trig.cmp_source_index = sh_dma_trig;
	cmp_trig.update_trigger = pwm_shadow_register_update_on_reload;
	pwmv2_config_cmp(pwm, 2u, &cmp_trig);

	pwmv2_counter_select_data_offset_from_shadow_value(pwm, counter, sh_period);
	pwmv2_counter_burst_disable(pwm, counter);
	pwmv2_set_reload_update_time(pwm, counter, pwm_reload_update_on_shlk);

	pwmv2_shadow_register_lock(pwm);
	pwmv2_issue_shadow_register_lock_event(pwm);
}

static void dshot_build_descriptors(void)
{
	for (unsigned k = 0; k < DSHOT_FRAME_WORDS; k++) {
		dma_mgr_chn_conf_t conf = dma_conf;

		conf.src_addr = (uint32_t)&frame_buf[k * num_channels];
		conf.dst_addr = (uint32_t)&pwm->SHADOW_VAL[DSHOT_BIT_SLOT0];
		conf.size_in_byte = num_channels * sizeof(uint32_t);
		conf.linked_ptr = (k + 1 < DSHOT_FRAME_WORDS) ? (uint32_t)&dshot_desc[k + 1] : 0;
		conf.interrupt_mask = (k + 1 == DSHOT_FRAME_WORDS)
				      ? (DMA_INTERRUPT_MASK_ALL & ~DMA_INTERRUPT_MASK_TERMINAL_COUNT)
				      : DMA_INTERRUPT_MASK_ALL;

		if (dma_mgr_config_linked_descriptor(&dma_res, &conf, &dshot_desc[k]) != status_success) {
			PX4_ERR("dshot: descriptor %u failed", k);
		}
	}
}

static int dshot_dma_init(void)
{
	trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_1, HPM_TRGM0_DMA_SRC_PWM1_REQ1);

	dma_mgr_init();

	if (dma_mgr_request_resource(&dma_res) != status_success) {
		return -ENODEV;
	}

	dma_allocated = true;

	dma_mgr_get_default_chn_config(&dma_conf);
	dma_conf.en_dmamux = true;
	dma_conf.dmamux_src = HPM_DMA_SRC_MOT_1;
	dma_conf.src_mode = 0;
	dma_conf.dst_mode = 1;                    /* one request -> one burst */
	dma_conf.src_width = DMA_TRANSFER_WIDTH_WORD;
	dma_conf.dst_width = DMA_TRANSFER_WIDTH_WORD;
	dma_conf.src_addr_ctrl = 0;               /* increment */
	dma_conf.dst_addr_ctrl = 0;               /* increment: burst writes 20..20+n-1 */
	dma_conf.src_burst_size = (num_channels == 1) ? DMA_NUM_TRANSFER_PER_BURST_1T :
				  (num_channels == 2) ? DMA_NUM_TRANSFER_PER_BURST_2T :
				  DMA_NUM_TRANSFER_PER_BURST_4T;
	dma_conf.src_addr = (uint32_t)frame_buf;
	dma_conf.dst_addr = (uint32_t)&pwm->SHADOW_VAL[DSHOT_BIT_SLOT0];
	dma_conf.size_in_byte = num_channels * sizeof(uint32_t);
	/* In the free-running counter scheme the whole frame is fed by the chain: the
	 * channel-control transfer that used to supply word 0 is not performed when
	 * the channel is enabled mid-period, so descriptor 0 (word 0 = bit15) has to
	 * be the first node.  Starting at descriptor 1 dropped bit15 - the frame went
	 * out as word1..word16, i.e. 15 pulses and the idle word.
	 */
	dma_conf.linked_ptr = (uint32_t)&dshot_desc[0];
	dma_conf.en_infiniteloop = false;
	dma_conf.handshake_opt = DMA_HANDSHAKE_OPT_ONE_BURST;
	dma_conf.interrupt_mask = DMA_INTERRUPT_MASK_ALL & ~DMA_INTERRUPT_MASK_TERMINAL_COUNT;

	dshot_build_descriptors();

	if (dma_mgr_setup_channel(&dma_res, &dma_conf) != status_success) {
		return -EIO;
	}

	if (dma_mgr_install_chn_tc_callback(&dma_res, dshot_tc_cb, NULL) != status_success) {
		return -EIO;
	}

	/* The request stays enabled; it only fires while the counter runs. */
	pwmv2_enable_dma_at_compare_point(pwm, pwm_dma_1, 2u);

	return OK;
}

int up_dshot_init(uint32_t channel_mask_in, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	(void)enable_bidirectional_dshot;   /* bidirectional telemetry is not implemented yet */

	if (dshot_pwm_freq == 0) {
		return -EINVAL;
	}

	/* Re-initialise when the requested configuration differs from the live one:
	 * silently keeping the old mask or bit rate made a second up_dshot_init() a
	 * no-op (the harness ended up transmitting on one channel after asking for
	 * four, and a rate change would have been ignored as well).
	 */
	if (initialized) {
		if ((channel_mask == (uint8_t)(channel_mask_in & 0xF)) && (dshot_bit_hz == dshot_pwm_freq)) {
			return OK;
		}

		up_dshot_deinit();
	}

	channel_mask = (uint8_t)(channel_mask_in & 0xF);

	if (channel_mask == 0) {
		return -EINVAL;
	}

	/* The DMA burst is one word per channel; keep it a power of two. */
	num_channels = (channel_mask & 0x8) ? 4 :
		       (channel_mask & 0x2) ? 2 : 1;

	dshot_pins();

	clock_add_to_group(clock_pwm1, 0);
	clock_add_to_group(clock_hdma, 0);

	const uint32_t clock = clock_get_frequency(clock_pwm1);

	if (clock == 0) {
		return -EIO;
	}

	reload = clock / dshot_pwm_freq - 1;
	dshot_bit_hz = dshot_pwm_freq;

	for (unsigned ch = 0; ch < DSHOT_MAX_CHANNELS; ch++) {
		channel_frame[ch] = 0;      /* throttle 0 */
	}

	dshot_pwm_init();

	for (unsigned ch = 0; ch < num_channels; ch++) {
		dshot_build_channel(ch);
	}

	const int ret = dshot_dma_init();

	if (ret != OK) {
		return ret;
	}

	/* The PWM counter runs from configuration time: the duty shadow holds an
	 * out-of-range value (no pulse, line low), and a frame is started later by
	 * letting the DMA feed the shadow from a defined point of the free-running
	 * period.  Starting the counter per frame instead made the first bit depend
	 * on a race against the first reload.
	 */
	pwmv2_reset_counter(pwm, counter);
	pwmv2_enable_counter(pwm, counter);
	pwmv2_start_pwm_output(pwm, counter);

	initialized = true;
	armed = false;

	return OK;
}

void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry)
{
	if (!initialized || channel >= num_channels) {
		return;
	}

	const uint16_t packet = (uint16_t)((throttle << 1) | (telemetry ? 1u : 0u));
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;

	channel_frame[channel] = (uint16_t)((packet << 4) | crc);

	dshot_build_channel(channel);

	/* Keep the duty shadow out of range while the frame is being prepared: the
	 * counter runs freely, and this value never matches, so the line stays low.
	 * The frame itself is fed by the DMA (word 0 = bit15 is the first transfer).
	 */
	pwmv2_shadow_register_unlock(pwm);
	pwmv2_set_shadow_val(pwm, dshot_shadow(channel), reload + (reload / 2u), 0, false);
	pwmv2_shadow_register_lock(pwm);
}

int up_dshot_arm(bool arm)
{
	if (!initialized) {
		return -EINVAL;
	}

	armed = arm;

	for (unsigned ch = 0; ch < num_channels; ch++) {
		if (arm) {
			pwmv2_channel_enable_output(pwm, dshot_channels[ch]);

		} else {
			pwmv2_channel_disable_output(pwm, dshot_channels[ch]);
		}
	}

	return OK;
}

void up_dshot_trigger(void)
{
	if (!initialized || !armed) {
		return;
	}

	/* Re-arm the descriptor chain from its first node. */
	dma_clear_transfer_status(dma_res.base, dma_res.channel);

	if (dma_mgr_setup_channel(&dma_res, &dma_conf) != status_success) {
		return;
	}

	/* The counter runs freely, so the frame has to start at a defined phase of the
	 * period: the duty shadow written by the DMA is latched into the comparator at
	 * the next reload, and the compare request that feeds the following word must
	 * not fire before that.  Prime the frame just past the compare request point,
	 * with a margin so the request line has definitely dropped before the channel
	 * is enabled; the first period then stays idle and the frame starts on the
	 * next one, with almost a whole period between each write and its latch.
	 * (Priming right after the reload instead was measured to produce an extra
	 * edge on ~50 % of the frames - see BDSHOT_PLAN.md.)
	 */
	const uint32_t request_point = dshot_dma_request_point();
	const uint32_t margin = (reload / 32u) + 1u;
	const uint32_t limit = (request_point > margin) ? (request_point - margin) : 0u;

	for (unsigned i = 0; i < 1000000u; i++) {
		if (pwm->CNT_VAL[counter] < limit) {
			break;
		}
	}

	if (dma_mgr_enable_channel(&dma_res) != status_success) {
		return;
	}
}

void up_dshot_idle(void)
{
	if (!initialized) {
		return;
	}

	/* Force "no pulse" so the line returns to idle low.  Called from thread
	 * context - writing the shadows from the DMA TC interrupt deadlocks against
	 * the DMA writing the same registers.
	 */
	pwmv2_shadow_register_unlock(pwm);

	for (unsigned ch = 0; ch < num_channels; ch++) {
		pwmv2_set_shadow_val(pwm, dshot_shadow(ch), reload + (reload / 2u), 0, false);
	}

	pwmv2_shadow_register_lock(pwm);
}

bool up_dshot_frame_done(void)
{
	if (!dma_allocated) {
		return true;
	}

	/* Poll the channel state instead of using an interrupt: the linked list ends
	 * at the last descriptor, so the controller disables the channel itself. */
	return (dma_res.base->CHEN & (1u << dma_res.channel)) == 0u;
}

unsigned up_dshot_frame_words(void)
{
	return DSHOT_FRAME_WORDS;
}

unsigned up_dshot_num_channels(void)
{
	return num_channels;
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

	for (unsigned ch = 0; ch < num_channels; ch++) {
		pwmv2_channel_disable_output(pwm, dshot_channels[ch]);
	}

	if (dma_allocated) {
		dma_mgr_disable_channel(&dma_res);
		dma_mgr_release_resource(&dma_res);
		dma_allocated = false;
	}

	initialized = false;
	armed = false;
}

/* ---------------------------------------------------------------------- */
/* BDShot receive: GPTMR measure mode + XDMA pair capture                  */
/* ---------------------------------------------------------------------- */
/*
 * Every pulse of the ESC reply is measured by a GPTMR channel in measure
 * mode: the hardware latches the pulse period and width by itself.  XDMA
 * moves both registers (CAPPRD, CAPDTY) in one 64-bit transfer per capture
 * event, so one event yields one (period, width) pair and the CPU is not
 * involved while the frame arrives.  The decoder below turns a window of
 * pairs into a 16 bit DShot frame and validates its CRC.
 *
 * GPTMR channels 0 and 1 have dedicated input pads, channels 2 and 3 take
 * their signal from TRGM, which is why the routing table is explicit.
 * XDMA is a separate controller from the HDMA feeding the transmit chain, so
 * transmit and receive can run at the same time; one GPTMR plus one XDMA
 * channel per motor leaves room for four motors.
 *
 * Measure mode hands out the PREVIOUS pulse on every capture event, so the
 * first pair of a window belongs to whatever happened before the frame.
 */

#define BD_RX_CHANNELS   4u
#define BD_RX_EVENTS     32u   /* pairs per channel, one pair per capture event */

typedef struct bd_rx_ch_s {
	uint8_t  gptmr;     /* 0 = GPTMR0, 1 = GPTMR1 */
	uint8_t  ch;        /* GPTMR capture channel */
	uint8_t  dma_ch;    /* XDMA channel */
	uint8_t  trgm_out;  /* TRGM output routed to the capture input */
	uint8_t  pin;       /* receive lead */
	uint32_t dma_src;   /* DMAMUX request source */
} bd_rx_ch_t;

/* The receive leads are the jumper rig used during the bring-up (motor 0 line
 * on PE08 looped back to PE09).  A real bidirectional ESC answers on the
 * transmit line itself, so board integration has to point the table at the
 * transmit pads and switch the pad function around the frame.
 */
static const bd_rx_ch_t bd_rx_ch[BD_RX_CHANNELS] = {
	{ 0, 2, 0, HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN2, 9,  HPM_DMA_SRC_GPTMR0_2 },
	{ 0, 3, 1, HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN3, 10, HPM_DMA_SRC_GPTMR0_3 },
	{ 1, 2, 2, HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN2, 11, HPM_DMA_SRC_GPTMR1_2 },
	{ 1, 3, 3, HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN3, 12, HPM_DMA_SRC_GPTMR1_3 },
};

static uint32_t bd_rx_buf[BD_RX_CHANNELS][BD_RX_EVENTS * 2u]
	__attribute__((section(".ahb_sram")));
static uint8_t bd_rx_mask;

static GPTMR_Type *bd_rx_gptmr(const bd_rx_ch_t *r)
{
	return r->gptmr ? HPM_GPTMR1 : HPM_GPTMR0;
}

static bool bd_rx_channel_valid(uint8_t channel)
{
	return (channel < BD_RX_CHANNELS) && ((bd_rx_mask & (1u << channel)) != 0u);
}

/* Force the receive pad to input before muxing it to TRGM: a pad that was an
 * output keeps driving the net otherwise.
 */
static void bd_rx_pad_input(unsigned pin)
{
	HPM_GPIO0->OE[GPIO_DI_GPIOE].VALUE &= ~(1u << pin);
	HPM_IOC->PAD[IOC_PAD_PE08 + pin - 8u].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(17);
	/* keep the pull-down: the line is idle low, a floating input produces
	 * junk capture events right after the counter starts
	 */
	HPM_IOC->PAD[IOC_PAD_PE08 + pin - 8u].PAD_CTL =
		IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
}

static void bd_rx_dma_reset(unsigned index)
{
	const bd_rx_ch_t *r = &bd_rx_ch[index];

	dma_disable_channel(HPM_XDMA, r->dma_ch);
	dma_abort_channel(HPM_XDMA, 1u << r->dma_ch);

	for (unsigned i = 0; i < 100000u; i++) {
		if ((HPM_XDMA->CHEN & (1u << r->dma_ch)) == 0u) {
			break;
		}
	}

	/* Only the abort above resets the internal transfer counter and the
	 * destination pointer; writing CTRL = 0 does not.
	 *
	 * The status has to go first: CTRL = 0 also clears the per-channel
	 * interrupt mask bits, i.e. it enables every interrupt source, and a
	 * pending terminal count would then raise the XDMA interrupt.  This
	 * channel is not registered with dma_mgr, so dma_mgr_isr_handler() cannot
	 * acknowledge it and the CPU would spin in the ISR forever.
	 */
	dma_clear_transfer_status(HPM_XDMA, r->dma_ch);
	HPM_XDMA->CHCTRL[r->dma_ch].CTRL = 0;
}

static void bd_rx_dma_config(unsigned index)
{
	const bd_rx_ch_t *r = &bd_rx_ch[index];
	dma_channel_config_t conf;

	dma_default_channel_config(HPM_XDMA, &conf);
	conf.src_addr = (uint32_t)&bd_rx_gptmr(r)->CHANNEL[r->ch].CAPPRD;
	conf.dst_addr = (uint32_t)bd_rx_buf[index];
	conf.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
	conf.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
	conf.src_width = DMA_TRANSFER_WIDTH_DOUBLE_WORD;
	conf.dst_width = DMA_TRANSFER_WIDTH_DOUBLE_WORD;
	conf.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
	conf.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
	conf.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
	conf.size_in_byte = BD_RX_EVENTS * 2u * sizeof(uint32_t);
	/* The capture window is polled, so the channel must not raise interrupts:
	 * the XDMA IRQ is enabled at the controller level by riscv_dma_initialize(),
	 * but this channel is not registered with dma_mgr, and dma_mgr_isr_handler()
	 * only acknowledges channels it owns.  A terminal count on this channel would
	 * therefore leave the status bit set and spin the CPU in the ISR forever.
	 */
	conf.interrupt_mask = DMA_INTERRUPT_MASK_ALL;

	if (dma_setup_channel(HPM_XDMA, r->dma_ch, &conf, false) != status_success) {
		return;
	}

	dmamux_config(HPM_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_XDMA, r->dma_ch), r->dma_src, true);
}

int up_bdshot_rx_init(uint32_t mask)
{
	mask &= (1u << BD_RX_CHANNELS) - 1u;

	if (mask == 0u) {
		mask = 0x1u;
	}

	if (mask == bd_rx_mask) {
		return OK;
	}

	if (bd_rx_mask != 0u) {
		up_bdshot_rx_deinit();
	}

	clock_add_to_group(clock_gptmr0, 0);
	clock_add_to_group(clock_gptmr1, 0);
	clock_add_to_group(clock_xdma, 0);

	for (unsigned index = 0; index < BD_RX_CHANNELS; index++) {
		if ((mask & (1u << index)) == 0u) {
			continue;
		}

		const bd_rx_ch_t *r = &bd_rx_ch[index];
		GPTMR_Type *gptmr = bd_rx_gptmr(r);

		bd_rx_pad_input(r->pin);

		trgm_filter_t filter = {};
		filter.mode = trgm_filter_mode_rapid_change;
		filter.filter_length = 8;
		trgm_filter_config(HPM_TRGM0, r->pin, &filter);

		trgm_output_t out = {};
		out.input = HPM_TRGM0_INPUT_SRC_TRGM0_P00 + r->pin;
		out.type = trgm_output_same_as_input;
		out.invert = false;
		trgm_output_config(HPM_TRGM0, r->trgm_out, &out);

		gptmr_channel_config_t cfg;
		gptmr_channel_get_default_config(gptmr, &cfg);
		cfg.mode = gptmr_work_mode_measure_width;
		cfg.cmp_initial_polarity_high = false;
		cfg.dma_request_event = gptmr_dma_request_on_input_signal_toggle;
		gptmr_channel_enable_dma_request(gptmr, r->ch, false);

		if (gptmr_channel_config(gptmr, r->ch, &cfg, false) != status_success) {
			up_bdshot_rx_deinit();
			return -EIO;
		}

		bd_rx_dma_reset(index);
		bd_rx_dma_config(index);
		gptmr_channel_enable_dma_request(gptmr, r->ch, true);
	}

	bd_rx_mask = (uint8_t)mask;

	return OK;
}

void up_bdshot_rx_deinit(void)
{
	if (bd_rx_mask == 0u) {
		return;
	}

	for (unsigned index = 0; index < BD_RX_CHANNELS; index++) {
		if ((bd_rx_mask & (1u << index)) == 0u) {
			continue;
		}

		const bd_rx_ch_t *r = &bd_rx_ch[index];

		gptmr_stop_counter(bd_rx_gptmr(r), r->ch);
		gptmr_channel_enable_dma_request(bd_rx_gptmr(r), r->ch, false);
		dma_disable_channel(HPM_XDMA, r->dma_ch);
		dma_abort_channel(HPM_XDMA, 1u << r->dma_ch);

		/* Drop the DMAMUX mapping: the released channel would otherwise
		 * keep listening on the GPTMR request and starve the next user.
		 */
		HPM_DMAMUX->MUXCFG[DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_XDMA, r->dma_ch)] = 0;
	}

	bd_rx_mask = 0u;
}

bool up_bdshot_rx_ready(void)
{
	return bd_rx_mask != 0u;
}

void up_bdshot_rx_arm(void)
{
	for (unsigned index = 0; index < BD_RX_CHANNELS; index++) {
		if ((bd_rx_mask & (1u << index)) == 0u) {
			continue;
		}

		const bd_rx_ch_t *r = &bd_rx_ch[index];
		GPTMR_Type *gptmr = bd_rx_gptmr(r);

		gptmr_channel_reset_count(gptmr, r->ch);
		gptmr_clear_status(gptmr, GPTMR_CH_CAP_STAT_MASK(r->ch));
		memset(bd_rx_buf[index], 0, sizeof(bd_rx_buf[index]));

		bd_rx_dma_reset(index);
		bd_rx_dma_config(index);
		dma_enable_channel(HPM_XDMA, r->dma_ch);
		gptmr_start_counter(gptmr, r->ch);
	}
}

void up_bdshot_rx_flush(void)
{
	/* The line shows a short spike right after the counter starts; drop
	 * whatever the arming sequence caught and restart the transfer from the
	 * first entry while the capture counter keeps running.
	 */
	for (unsigned index = 0; index < BD_RX_CHANNELS; index++) {
		if ((bd_rx_mask & (1u << index)) == 0u) {
			continue;
		}

		const bd_rx_ch_t *r = &bd_rx_ch[index];

		memset(bd_rx_buf[index], 0, sizeof(bd_rx_buf[index]));

		bd_rx_dma_reset(index);
		bd_rx_dma_config(index);
		dma_enable_channel(HPM_XDMA, r->dma_ch);
	}
}

void up_bdshot_rx_stop(void)
{
	for (unsigned index = 0; index < BD_RX_CHANNELS; index++) {
		if ((bd_rx_mask & (1u << index)) == 0u) {
			continue;
		}

		const bd_rx_ch_t *r = &bd_rx_ch[index];

		gptmr_stop_counter(bd_rx_gptmr(r), r->ch);
		dma_disable_channel(HPM_XDMA, r->dma_ch);
		dma_abort_channel(HPM_XDMA, 1u << r->dma_ch);

		/* Leave no pending interrupt behind: the next re-arm clears CTRL,
		 * which unmasks every source.
		 */
		dma_clear_transfer_status(HPM_XDMA, r->dma_ch);
	}
}

const uint32_t *up_bdshot_rx_buffer(uint8_t channel)
{
	return bd_rx_channel_valid(channel) ? bd_rx_buf[channel] : NULL;
}

unsigned up_bdshot_rx_words(void)
{
	return BD_RX_EVENTS * 2u;
}

unsigned up_bdshot_rx_events(uint8_t channel)
{
	if (!bd_rx_channel_valid(channel)) {
		return 0;
	}

	const uint32_t *buf = bd_rx_buf[channel];
	unsigned events = 0;

	for (unsigned i = 0; i + 1u < BD_RX_EVENTS * 2u; i += 2u) {
		if (buf[i] != 0u || buf[i + 1u] != 0u) {
			events++;
		}
	}

	return events;
}

bool up_bdshot_rx_done(uint8_t channel)
{
	/* A frame is 16 pulses and measure mode reports the previous one, so the
	 * window of a complete frame holds 16 pairs: the stale one from before the
	 * frame plus 15 measured widths (see up_bdshot_rx_frame()).
	 */
	return up_bdshot_rx_events(channel) >= 16u;
}

static bool bd_rx_crc_ok(uint16_t frame)
{
	const uint16_t packet = frame >> 4;
	const uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;

	return (frame & 0x0F) == crc;
}

static uint32_t bd_rx_bit_ticks(void)
{
	const uint32_t gclk = clock_get_frequency(clock_gptmr0);

	if (gclk == 0u || dshot_bit_hz == 0u) {
		return 0u;
	}

	return gclk / dshot_bit_hz;
}

int up_bdshot_rx_frame(uint8_t channel, uint16_t *frame, unsigned *pairs_out)
{
	if (frame == NULL) {
		return -EINVAL;
	}

	if (!bd_rx_channel_valid(channel)) {
		return -EINVAL;
	}

	const uint32_t bit = bd_rx_bit_ticks();

	if (bit == 0u) {
		return -EINVAL;
	}

	const uint32_t half = bit / 2u;
	const uint32_t lo = bit / 4u;           /* a "0" is 37.5 % of a bit time */
	const uint32_t hi = bit + half;         /* generous upper bound */
	const uint32_t *buf = bd_rx_buf[channel];
	uint16_t value = 0;
	unsigned pairs = 0;
	unsigned bits = 0;

	for (unsigned i = 0; i + 1u < BD_RX_EVENTS * 2u; i += 2u) {
		const uint32_t prd = buf[i];
		const uint32_t dty = buf[i + 1u];

		if (prd == 0u && dty == 0u) {
			continue;               /* entry the DMA never wrote */
		}

		pairs++;

		if (pairs == 1u) {
			/* stale measurement: the pulse that was running when the
			 * counter started, not part of the frame
			 */
			continue;
		}

		if (dty < lo || dty > hi) {
			continue;               /* sub-bit glitch on the line */
		}

		if (bits < 16u) {
			value = (uint16_t)((value << 1) | ((dty > half) ? 1u : 0u));
			bits++;
		}
	}

	if (pairs_out != NULL) {
		*pairs_out = pairs;
	}

	if (bits == 16u && bd_rx_crc_ok(value)) {
		*frame = value;
		return OK;
	}

	if (bits == 15u) {
		/* The width of the last pulse is latched by the next capture event
		 * and a frame has no pulse after its own: bit0 is not measured.
		 * The CRC determines it, so try both values and keep the valid one.
		 */
		for (unsigned lsb = 0; lsb < 2u; lsb++) {
			const uint16_t candidate = (uint16_t)((value << 1) | lsb);

			if (bd_rx_crc_ok(candidate)) {
				*frame = candidate;
				return OK;
			}
		}
	}

	return -EAGAIN;
}

int up_bdshot_rx_wait(uint8_t channel, uint32_t timeout_us, uint16_t *frame)
{
	const unsigned step_us = 50u;
	const unsigned polls = timeout_us / step_us + 1u;

	for (unsigned i = 0; i < polls; i++) {
		const int ret = up_bdshot_rx_frame(channel, frame, NULL);

		if (ret == OK) {
			return OK;
		}

		if (ret != -EAGAIN) {
			return ret;
		}

		up_udelay(step_us);
	}

	return -ETIMEDOUT;
}

/* Bidirectional DShot is not implemented yet; keep the API consistent. */
int up_bdshot_num_erpm_ready(void) { return 0; }
int up_bdshot_get_erpm(uint8_t channel, int *erpm) { (void)channel; if (erpm) { *erpm = 0; } return -ENOSYS; }
int up_bdshot_channel_status(uint8_t channel) { (void)channel; return 0; }
void up_bdshot_status(void) { }
