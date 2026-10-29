/****************************************************************************
 *
 * Copyright (C) 2023 PX4 Development Team. All rights reserved.
 * Author: Peter van der Perk <peter.vanderperk@nxp.com>
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

#pragma once

#include <drivers/drv_pwm_output.h>


/* The structure which contains configuration for DShot
 */
typedef struct dshot_conf_t {
	uint32_t			pinmux;
	uint32_t			flexio_pin;
} dshot_conf_t;

/* BDShot receive: hardware capture of the ESC reply.
 *
 * The receive engine is independent of the transmit chain: one GPTMR capture
 * channel (in measure mode) plus one XDMA channel per motor deliver a stream
 * of (period, width) pairs, and up_bdshot_rx_frame() decodes and CRC-checks
 * one frame from them.  Nothing here interrupts the CPU.
 */

/**
 * Initialise the receive engine.
 *
 * @param mask	bit mask of the DShot channels to listen on (0 = channel 0)
 * @return OK on success, < 0 on error
 */
__EXPORT int up_bdshot_rx_init(uint32_t mask);

/**
 * Release the capture channels, timers and DMAMUX mappings.
 */
__EXPORT void up_bdshot_rx_deinit(void);

/**
 * True when the receive engine is initialised.
 */
__EXPORT bool up_bdshot_rx_ready(void);

/**
 * Start a capture window: clear the buffers and arm the DMA from entry 0.
 */
__EXPORT void up_bdshot_rx_arm(void);

/**
 * Drop what the arming sequence captured and restart the DMA transfer while
 * the capture counter keeps running.
 */
__EXPORT void up_bdshot_rx_flush(void);

/**
 * Stop the capture window.
 */
__EXPORT void up_bdshot_rx_stop(void);

/**
 * True when the window already holds a complete frame (more capture events
 * than a frame has pulses).  A hint for polling callers, not a decode.
 */
__EXPORT bool up_bdshot_rx_done(uint8_t channel);

/**
 * Number of capture events written into the channel buffer.
 */
__EXPORT unsigned up_bdshot_rx_events(uint8_t channel);

/**
 * Raw capture buffer of one channel: (period, width) pairs, two words each.
 */
__EXPORT const uint32_t *up_bdshot_rx_buffer(uint8_t channel);

/**
 * Words per channel buffer (two per capture event).
 */
__EXPORT unsigned up_bdshot_rx_words(void);

/**
 * Decode one DShot frame from the capture buffer and check its CRC.
 *
 * @param channel	DShot channel
 * @param frame		written with the decoded frame on success
 * @param pairs_out	set to the number of capture events seen, may be NULL
 * @return OK on a CRC valid frame, -EAGAIN while the frame is incomplete,
 *	   -EINVAL on bad arguments
 */
__EXPORT int up_bdshot_rx_frame(uint8_t channel, uint16_t *frame, unsigned *pairs_out);

/**
 * Wait until a CRC valid frame arrives.
 *
 * @param channel	DShot channel
 * @param timeout_us	give up after this long
 * @param frame		written with the decoded frame on success
 * @return OK on success, -ETIMEDOUT when nothing valid arrived, < 0 on error
 */
__EXPORT int up_bdshot_rx_wait(uint8_t channel, uint32_t timeout_us, uint16_t *frame);
