# Bidirectional DShot (BDShot) — implementation plan

Status: **not implemented yet**.  The output direction (DShot frames) is done:
see `dshot.c` (`up_dshot_init/dshot_motor_data_set/up_dshot_arm/up_dshot_trigger`),
4 channels on PWM1 (PE08..PE11), 16-bit frame fed by a 16-node linked
descriptor chain, one DMA burst per bit period.

This file records everything learned so the receive path can be implemented
later without re-doing the investigation.

## What the ESC does

1. It decodes the 16-bit command frame after receiving **all** bits and
   validates the 4-bit CRC.
2. Only if the CRC is correct it answers, ~30 us after the frame, with a pulse
   train (GCR 5b/4b encoded, 20 bits of period + 4-bit CRC) on the same wire.
3. The flight controller must release the wire (tri-state) before the reply,
   otherwise both sides drive the line.

## HPM6E00 hardware facts (verified)

- PWMv2 has input capture per channel:
  - comparator mode `pwm_cmp_mode_input_capture`, source
    `cmp_value_from_capture_posedge`;
  - results in `PWMV2->CAPTURE_POS[ch]` / `CAPTURE_NEG[ch]`;
  - helpers: `pwmv2_capture_selection_input_source(pwm, ch, pwm_capture_from_gpio)`,
    `pwmv2_set_capture_counter_index()`, `pwmv2_get_capture_posedge_value()`,
    `pwmv2_enable_capture_posedge_irq()`, IRQ status/clear helpers.
- DMA requests of PWMv2 exist **only** at compare points and reload points:
  `pwmv2_enable_dma_at_compare_point(pwm, dma_ch, cmp_index)` and
  `pwmv2_enable_dma_at_reload_point(pwm, dma_ch, counter)`.
  A comparator running in input-capture mode is therefore the only way to get a
  DMA request per captured edge; DMA source would be `&CAPTURE_POS[ch]`
  (address fixed) with one word per edge.
- **No internal loopback**: `hpm_trgmmux_src.h` for 6E00 has no
  `PWMx_CHyREF` inputs, so `pwm_capture_from_trigmux` cannot be fed from the
  PWM output.  Capture must come from the pad (`pwm_capture_from_gpio`).
- Duty shadow slots 20..23 are unlocked for DMA (`UNLOCK` bit = 2 + slot).
- The DMA channel must be allocated through `dma_mgr` (otherwise the TC
  interrupt is not acknowledged and the board hangs in an interrupt storm).
- Buffers/descriptors live in non-cacheable memory (`.ahb_sram` or
  `.noncacheable.bss`); both were validated for the TX path.

## Direction switching

The wire must be released before the ESC answers.  Two options:

1. **Pad function switch (preferred, cheap):** for the capture window set
   `HPM_IOC->PAD[PE0x].FUNC_CTL` to GPIO input (and enable the GPIO input
   path), then switch back to `PWM1_P_x` for the next frame.  This is two
   register writes per frame, comparable to what the STM32 port does.
2. **Open-drain / released output:** keep the PWM function and configure the
   pad open-drain, relying on the ESC to pull the line.  Needs checking
   whether PWMv2 releases the pad when the channel output is disabled; if it
   does not, contention remains.

### Considered alternative: sample the GPIO IDR with a second DMA

Reading `HPM_GPIO0->DI` (or the port's IDR) with a second DMA channel and
oversampling the line was considered to avoid the direction switch.
Trade-offs:

- GPIO `DI` is usually fed from the pad regardless of `FUNC_CTL`, so the level
  can be observed even while the pad is in PWM mode — but the ESC still cannot
  drive the line while the PWM output drives it, so the pad must be released
  anyway.
- Oversampling gives poor time resolution: 1 us accuracy at 1 MHz sampling,
  and BDShot pulses are 167 ns..1 us.  HW input capture gives one timer tick
  (5 ns at 200 MHz) and no CPU cost until the window is decoded.
- Conclusion: use PWMv2 input capture; the IDR/DMA idea is only useful as a
  debugging aid (e.g. checking that the wire is actually released).

## Implementation outline

1. `up_dshot_init(..., bidirectional = true)`:
   - keep the TX setup as is;
   - allocate a **second** `dma_mgr` resource for capture;
   - prepare the capture buffer (32 words per channel, non-cacheable) and the
     capture DMA config: `src = &CAPTURE_POS[ch]` (address fixed),
     `dst = capture buffer`, width WORD, `interrupt_mask` TC only.
2. `up_dshot_trigger()` (TX) as today; in the TC callback:
   - stop the counter, disable channel outputs;
   - switch pads to GPIO input (`FUNC_CTL`), clear capture IRQ status;
   - enable the comparator in input-capture mode and the capture DMA;
   - start a one-shot window timer (hrt call, ~30 us after the last bit).
3. Window end:
   - disable capture DMA and the comparator, back to PWM output;
   - decode: edges -> 20-bit GCR -> 5b/4b -> 20-bit period, verify CRC;
   - compute eRPM from the period, store per channel;
   - report through `up_bdshot_num_erpm_ready()` / `up_bdshot_get_erpm()` /
     `up_bdshot_channel_status()` (already stubbed in `dshot.c`).
4. Frame rate: BDShot needs one frame per telemetry request; the driver loop
   (`DShot.cpp`) already alternates telemetry index, so one channel is polled
   per frame.

## Test plan

1. **Self-test without an ESC:** jumper PE08 -> PE09.  Send a normal DShot
   frame on PE08, capture the same signal on PE09 (GPIO input), decode the
   pulse widths back into bits and compare with `channel_frame[0]`.  This
   validates pad input, capture comparator, DMA and the timing math without
   any ESC.
2. **Scope:** verify the window timing (capture armed after the last bit, wire
   released before the ESC reply).
3. **With an ESC:** check CRC-correct replies, eRPM values and their scaling;
   compare with a known ERPM (e.g. `pwm_out` at fixed throttle + tachometer).
4. **Robustness:** no reply at all (window expires), CRC error, short frame,
   double edges; make sure the board never hangs (this was the failure mode of
   the earlier DMA bring-up).

## Open questions

- Does `CAPTURE_POS[ch]` keep updating while the pad is a plain GPIO input, or
  must the pad stay in a PWM alternate function for the capture path?
- Is a DMA request actually generated by a comparator in input-capture mode
  (`pwmv2_enable_dma_at_compare_point`), and does it fire per edge?
- Capture timer resolution and overflow handling (the counter runs at
  200 MHz; the reply period is ~1 us..20 us, so no overflow).
- Whether the pad must be open-drain during the reply (depends on the ESC
  driver strength).
