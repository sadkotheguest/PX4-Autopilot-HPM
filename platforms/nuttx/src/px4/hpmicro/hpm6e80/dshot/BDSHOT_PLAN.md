# Bidirectional DShot (BDShot) — implementation plan

Status (final bring-up build): **bring-up done, not integrated**.
The output direction is verified at 150/300/600 kbit/s: 16 data bits + one idle
word (17-node linked descriptor chain), 4 channels on PWM1 (PE08..PE11),
push-pull output, line idle low, frame end detected by polling the DMA channel.
The receive direction is verified on a PE08 -> PE09 jumper: GPTMR measure mode
+ XDMA pair capture, `gcapxsweep 5` -> 105/105 MATCH.  The production receive
API (`up_bdshot_rx_*`, declared in `px4_arch/dshot.h`) is in place and replaced
the experimental `up_dshot_rxd_*` block.  It decodes our own transmitted frame;
an ESC reply (GCR, eRPM) is not implemented.  Nothing calls the driver yet
(`CONFIG_DRIVERS_DSHOT` is off, no io_timer / PWM_OUT glue).

The sections below are the chronological design and bring-up log, including the
abandoned experiments (see "Dead-code inventory" for what can be deleted).
Current status, the resource map, the receive API contract and the open items
are collected in the sections at the end.

This file records everything learned so the production receive path can be
completed without re-doing the investigation.

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

## Direction switching (resolved: no pad mux change)

The pad must STAY on the PWM alternate function - that is what feeds the
capture input (`CAPTURE_POS/NEG[n]` come from the PWMv2 port `P[n]`).
Switching the pad to a plain GPIO function kills the capture input (measured:
`CAPTURE_POS[1]` stayed at the invalid `0xFFFFFF00`).

The wire is released with the PWM channel output instead:

- `pwmv2_channel_disable_output(pwm, ch)` clears `CFG1.HIGHZ_EN_N`, which puts
  the pad into high-Z (the bit is literally the "high-Z enable" control);
- `pwmv2_channel_enable_output(pwm, ch)` restores the push-pull output.

Both are plain work-register read-modify-writes (no shadow/lock event needed),
so they apply immediately.  Cost per frame: one RMW per channel for release
and one for re-enable (4 channels -> 8 accesses), plus the counter enable
(`CNT_GLBCFG`).  An AHB read+write pair to PWM registers costs on the order
of 50-100 ns, i.e. well below 1 us per frame boundary, against a 33 us frame
at 600 kbit/s or a 1 ms frame period at 1 kHz.  Negligible.

If a defined level is needed while the line floats (between our last bit and
the ESC answer), enable the pad pull-down once in `PAD_CTL` at init - it is a
one-time write, not a per-frame operation.

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

## Capture bring-up log (self-test, jumper PE08 -> PE09)

First attempt, `pwmv2test captest <rate> <throttle>` in pwmv2test:

- PE08 stays the DShot output (channel 0), PE09 is configured as GPIO input
  with a pull-down and `pwmv2_capture_selection_input_source(..., pwm_channel_1,
  pwm_capture_from_gpio)`;
- comparator 2 was put into input-capture mode with
  `cmp_source = cmp_value_from_capture_posedge`, capture counter index 0;
- the status registers do report edges (~20 rising / 20 falling for one
  frame), so the pad input path and the capture event logic work;
- but `pwmv2_get_capture_posedge_value()/negedge_value()` return 0xFFFFFF
  (the "no capture" value), i.e. the counter snapshot is not associated with
  the capture register.

Next experiments:

1. Use the comparator index that belongs to the capture channel (channel 1 ->
   comparator 1) instead of comparator 2 - the SDK pwm_cap sample passes the
   *channel* to `pwm_config_cmp()`, not an arbitrary comparator.
2. Check whether the capture counter has to be put into a capture counter type
   (the v1 sample uses `pwm_counter_type_capture_falling_edge`); look for the
   PWMv2 equivalent.
3. Try reading both `CAPTURE_POS` and `CAPTURE_NEG` for every channel and log
   which index actually holds the snapshot.
4. Only after the snapshot works: replace polling by the capture DMA request
   (`pwmv2_enable_dma_at_compare_point`) and decode the DShot frame from the
   captured edges (compare with `channel_frame[0]`), then add GCR decoding.
## Capture bring-up: resolved facts (UM HPM6E00UMENV08, ch. 42.4.12, 42.6.17)

- Input capture is a **dedicated module per PWM channel** (not the comparator):
  on a rising edge the counter value is latched into `CAPTURE_POS[n]`, on a
  falling edge into `CAPTURE_NEG[n]` (bits 31:8 = 24-bit counter value,
  bits 1:0 = `CNT_INDEX`, bit 4 = `CAPTURE_SELGPIO`).
- `CAPTURE_SELGPIO = 0` selects the PWMv2 port `P[n]` (the pad), `1` selects
  the TRGM input.  Note: the SDK enum has the opposite naming
  (`pwm_capture_from_gpio = 1`), so pass 0 for the pad.
- The captured signal comes from the PWM port, therefore the pad **must stay
  on its PWM alternate function** (`PWM1_P_x`).  Configuring the pad as plain
  GPIO disconnects the capture input (observed: `CAPTURE_POS[1]` stayed
  `0xFFFFFF00`, the "no capture" value).
- A channel can capture its own output: with the pad in `PWM1_P_0` mode,
  `CAPTURE_POS[0] = 0x0000f300` (counter value 0xF3 = 243) was latched from
  the DShot output on PE08 without any external wiring - useful for a
  self-test.
- `IRQ_STS_CAP_POS/NEG` are W1C bitmaps (bit n = channel n); `IRQ_EN_CAP_POS/NEG`
  enable the interrupt.

### Important correction for the receive path

UM 42.4.16 "DMA Requests": PWMv2 generates DMA requests **only** for compare
value match events or counter reload events.  The input capture module has no
DMA request, so a "capture DMA" as in the STM32 port is **not available**.
BDShot receive must therefore be **interrupt driven**: one ISR per captured
edge that stores the counter value into a ring buffer.

Edge budget: a DShot frame has ~32 edges, a GCR reply ~21; at 1 kHz frames and
telemetry requested every N frames this is a few thousand interrupts per
second - acceptable.  Revisit if the frame rate is raised.

### Current self-test state

`pwmv2test captest <rate> <throttle>` sends one frame on PE08 and captures it
on PE09 (jumper) or on PE08/PE08 (pad self-capture).  Verified so far:
`CAPTURE_POS[0]` latches real counter values; channel 1 with the pad in
`PWM1_P_1` mode latches values as well.  Polling for the events is racy (the
registers only hold the last edge, and several edges may arrive between two
polls), so the decode shows only the last pair.  Next step: replace the
polling loop with the capture interrupt and a ring buffer, then decode the
16 bits of the transmitted frame and compare them with `channel_frame[0]`.
### Considered: toggling high-Z from the DMA

Technically possible: the DMA can write peripheral registers (the TX path already
writes SHADOW_VAL through DMA), so a descriptor with `dst = &PWM[ch].CFG1` and a
precomputed value (`CFG1` with `HIGHZ_EN_N` cleared) would tri-state the pad
without CPU involvement.  To time it correctly it needs its own trigger: an
extra compare value at the end of the frame routed through a TRGM DMA output and
a second DMAMUX/DMA channel; re-enabling for the next frame needs a second such
event or one CPU write.

Why it is not worth it:

- the CPU already runs in the TC interrupt at exactly the right moment, so
  `pwmv2_channel_disable_output()` there costs essentially nothing extra;
- the per-frame CPU cost of the RMW writes is <1 us (see above);
- DMA writes are full 32-bit values without read-modify-write, so the `CFG1`
  shadow-configuration bits (polarity, dead zone, pair mode...) must be
  duplicated and kept in sync with the PWM setup - a silent source of corruption;
- it adds one comparator, one TRGM output and one DMA channel to debug.

UM reference: `PWM_CFG1.HIGHZ_EN_N` (bit 28) - "0 to highz pwm outputs
(pad_oe*=0), software need set this bit to 1 to enable pwm output".  The fault
path can also tri-state the pin (`FAULT_MODE = 2'b1x` -> output high-Z), which is
reserved for fault handling, not for the DShot reply window.
### ISR capture: current state (ring buffer works, latch values frozen)

Implemented `up_dshot_capture_init/enable/reset/count/get` in `dshot.c`:
PWM1 IRQ attached, one ring-buffer entry per captured edge (channel, edge
kind, counter value), 128 entries, dropped counter.

Result of `pwmv2test captest 600 50` (jumper PE08->PE09):

    edges 28 (dropped 0)
    [0] ch1 pos 90   [1] ch1 pos 90   [2] ch1 neg 7   [3] ch1 pos 90 ...

The ISR runs and the ring buffer fills, but every captured value is the same
pair (pos 90, neg 7, difference 83 = the comparator value of a "1" bit), i.e.
the `CAPTURE_POS/NEG[1]` registers are not being updated per edge - they keep
a value latched once.  The status bits still get set (so the capture module
detects edges), but the snapshot does not change.

Hypotheses to test next:

1. `CNT_INDEX` selects the counter, but the counter must be the one that
   actually runs while capturing; try each of the 4 counters and log
   `CAPTURE_POS/CNT_INDEX` bits after writing.
2. Read the raw 32-bit `CAPTURE_POS[1]` inside the ISR (including SELGPIO and
   CNT_INDEX bits) to verify the configuration is still what we wrote.
3. Capture a slow, known signal instead of a 1.6 us DShot bit: e.g. drive
   PE09 from another PWM channel at 1 kHz / 50 % and check that pos/neg values
   change with the period (this separates "latch works" from "too fast").
4. Check whether the capture input for channel 1 needs the channel output
   enabled (the pad is in PWM1_P_1 with the channel output in high-Z).
### Capture diagnostics, items 1-4 (capdiag)

`pwmv2test capdiag` builds a 1 kHz / 50 % PWM on channel 0 and channel 1 and
sweeps the capture counter index and the output enable:

    capdiag: 1 kHz, 50% duty (reload 199999, duty cmp 99999)
    capdiag: ch 0 out 0 edges 0 raw POS 0xffffff00
    capdiag: ch 0 out 1 edges 0 raw POS 0xffffff00
    capdiag: ch 1 out 0 edges 0 raw POS 0xffffff00
    capdiag: ch 1 out 1 edges 0 raw POS 0xffffff00

Findings:

- For the slow 1 kHz signal the capture module reports **no edges at all** and
  `CAPTURE_POS` stays at the invalid `0xFFFFFF00`, for both channels, with the
  output enabled or tri-stated, and for every `CNT_INDEX` value.
- With the DShot frame (600 kbit) the status bits do get set, but the latched
  values never change (always pos 90 / neg 7), which is consistent with those
  status bits being caused by the output enable/high-Z transitions rather than
  by real capture events.
- Writing the shadow lock event before starting the counter (a genuine bug in
  the first capdiag version) did not change the result.
- `CAPTURE_POS/CNT_INDEX` bits read back exactly as written, so the register
  configuration itself is accepted; the pad input simply does not reach the
  capture module in this configuration.

### Next experiment: GPTMR input capture instead of PWMv2 capture

The SDK ships `samples/drivers/gptmr/input_capture`, i.e. GPTMR has a known
working edge-capture path.  For BDShot the plan becomes:

1. Validate GPTMR input capture on this board (pin -> GPTMR capture, IRQ or DMA
   of the capture register), measure the timestamp resolution.
2. If it works, use GPTMR for the reply window: one capture channel per motor
   (or one for the first motor, then extend), timestamps -> edge list ->
   GCR decode.
3. Keep the PWMv2 capture code (`up_dshot_capture_*`) as a fallback and revisit
   it only if the GPTMR path has a drawback (e.g. not enough capture channels).

The DShot transmit path is unaffected: it is verified and stays as is.
### SDK survey: PWMv2 capture is not used anywhere, GPTMR capture is

- No SDK code (drivers or samples) touches `CAPTURE_POS/CAPTURE_NEG` of PWMv2;
  the only PWM capture sample (`samples/drivers/pwm/pwm_cap`) uses the v1
  `pwm_*` API, which does not exist for the PWMv2 IP used by 6E00.  So PWMv2
  input capture is effectively an untested path in the SDK, which matches our
  failures.
- Pin muxing: `PE09` has `IOC_PE09_FUNC_CTL_GPTMR0_CAPT_1`
  (PE08/PE10 only expose `GPTMR0_COMP_*`), i.e. the DShot wire can be fed into
  a GPTMR capture input on the same pad.
- GPTMR capture is a supported, documented path (sample
  `samples/drivers/gptmr/input_capture`): `gptmr_channel_config()` with
  `gptmr_work_mode_capture_at_both_edge`, capture IRQ, and
  `gptmr_channel_get_counter(..., gptmr_counter_type_rising_edge/falling_edge)`.

### gptmrcap result

Implemented `pwmv2test gptmrcap <rate> <throttle>`: PE08 sends the DShot frame,
PE09 = GPTMR0_CAPT_1, both-edge capture, 100 MHz GPTMR clock.

    600 kbit/s: bit = 166 ticks, 32 edges, widths seen 125 ticks (= 75 %, "1")
    150 kbit/s: bit = 666 ticks, widths 250 ticks (= 37.5 %, "0")

So the capture hardware and the tick resolution are fine, but the ISR-driven
edge collection loses/misorders edges when the spacing gets short (at 600 kbit
a "1" pulse is 125 ticks and the gap to the next edge is only 41 ticks = 410 ns,
which is shorter than the interrupt latency), and at 150 kbit the decoded frame
does not match yet either (all widths read as "0").

Next steps for the receive path:

1. Use the GPTMR **measure** modes (`gptmr_work_mode_measure_width`,
   `gptmr_counter_type_measured_period/duty_cycle`) so the hardware latches the
   pulse length itself; then one capture event per bit instead of two, and a
   late ISR only loses a bit instead of corrupting the pairing.
2. Check whether GPTMR capture can raise a DMA request, so the captured values
   are moved by DMA (no per-edge CPU work) - this is the STM32-like solution.
3. Re-verify the decoded bits against a scope trace before trusting the decoder:
   the widths seen at 150 kbit all equal 37.5 % of the bit time, which hints at
   an ordering/selection problem rather than at the transmitter.
### PWMv2 capture: the documented path goes through TRGM

UM facts (ch. 42.4.12, 42.4.16, 22.3.3, 55.2.4, pin mux):

- A capture channel can take its input from the PWMv2 port `P[n]`
  (`CAPTURE_SELGPIO = 0`) or from "other IOs", i.e. TRGM (`= 1`).
- TRGM.P[15:8] are wired to PWM1 capture inputs [7:0], TRGM.P[23:16] to PWM2,
  TRGM.P[7:0] to PWM0, TRGM.P[31:24] to PWM3.
- The pads of all four motor outputs can be routed into TRGM:
  PE08 -> `TRGM_P_08`, PE09 -> `TRGM_P_09`, PE10 -> `TRGM_P_10`,
  PE11 -> `TRGM_P_11` (pin mux alt 17).  This is the only documented way to
  capture the ESC reply on a motor pad, because the pad cannot stay in the PWM
  output function and be an input at the same time.
- The TRGM input filter sits in front of the capture: `trgm_filter_config()`
  with `trgm_filter_mode_bypass` (or `rapid_change` with a short length) must be
  used, otherwise the filter delay distorts the edge timing.
- `pwmv2test trgmcap` implements exactly this: PE09 -> TRGM.P[9] -> PWM1
  capture[1], filter bypass, capture IRQ, DShot frame sent on PE08.

Result: the captured values now depend on the bit rate (600 kbit: pos 90 / neg 7
with reload 332; 150 kbit: pos 840 / neg 7 with reload 1332), i.e. the TRGM path
is alive - but the per-edge value still does not refresh (identical pairs), so
the status/clear or the per-channel update still needs work.

Important constraints for the BDShot design:

- PWMv2 generates DMA requests only on compare-match and reload events
  (UM 42.4.16); **input capture has no DMA request**, so a capture value can
  only be moved by the CPU (interrupt or polling).
- GPTMR, in contrast, can select the capture event as its DMA request
  (`gptmr_dma_request_on_input_signal_toggle`), i.e. DMA-driven edge capture is
  possible - but GPTMR0 only exposes CAPT_0/CAPT_1 and only on a few pads
  (PA06/PC06/PE06/PV06 and PA09/PC09/PE09/PV09), so it cannot serve all four
  motor pads without rewiring.
- Interrupt load is not a problem in practice: PX4 requests telemetry from one
  ESC at a time (round-robin), so the capture IRQ is only enabled for that
  motor's reply window (~21 edges per telemetry frame), not continuously.
### bdscap: counter reset on the rising edge (STM32-style window)

`pwmv2test bdscap <rate> <throttle>` implements the planned scheme:

- PE09 -> TRGM.P[9] (mux alt 17), TRGM filter bypass;
- TRGM output `PWM1_TRIG_IN0` <- `TRGM0_P09`;
- `pwmv2_set_counter_reload_trigmux_index(counter0, 0)` so the counter restarts
  on every rising edge;
- capture input from TRGM (`CAPTURE_SELGPIO = 1`), and only the **falling edge**
  interrupt is enabled, so one ISR per bit and `CAPTURE_NEG` should hold the
  pulse width directly.

Measured result:

    600 kbit/s: bit 333 ticks -> entries pos 90 / neg 7 (constant)
    150 kbit/s: bit 1333 ticks -> entries pos 840 / neg 7 (constant)

So the status flags fire per edge, but the latched value still does not change
within a frame (it is constant per bit rate).  Together with the earlier
observations this means the PWMv2 capture value latch is not behaving as the UM
describes in our configuration; the TRGM routing, the filter and the counter
reset are all confirmed to be applied.

Practical conclusion for BDShot:

- GPTMR capture is the only path where we have seen the latched value change
  per edge (it also supports DMA on the capture event); it can serve **one**
  motor at a time, which is exactly what PX4 needs because telemetry is
  requested from one ESC per frame and rotated (`select_next_capture_channel()`
  in the STM32 port does the same).
- Polling every channel is therefore not a limitation: with round-robin each
  motor is sampled every N frames (N = number of motors), the same as on STM32.
- PWMv2 capture would remove the pin restriction, but its value latch needs more
  investigation (candidate causes: capture counter selection semantics, the
  TRGM trigger interacting with the capture module, or a silicon/SDK quirk).

## Current status (final bring-up build; RX through the production API)

**Transmit: verified.**  The frame scheme changed after the sections above were
written; this is the current one.

- One frame is 17 words: 16 data bits + one idle word.  The frame buffer is
  interleaved per motor and one DMA burst writes the duty shadows of all
  enabled channels; the linked descriptor chain has one node per word.
- bit15 is preloaded into the duty shadow by `dshot_motor_data_set()`; the
  channel-control transfer that runs when the DMA channel is enabled consumes
  chain word 0, so the linked chain starts at descriptor 1 (bit14) and the
  burst writes bit14..bit0 plus the idle word.  This is why the chain is 17
  nodes and not 16.
- "1" is 75 % duty, "0" is 37.5 % (`dshot_cmp()`); `reload = PWM1 clock /
  bitrate - 1`, so the same code serves 150/300/600 kbit/s.
- The pad is push-pull while armed: `up_dshot_arm(true)` enables the channel
  outputs and the frame path never releases the line.  A unidirectional DShot
  output must drive the wire; the high-Z scheme in the older sections is not
  used for transmitting.
- Idle is low, not high-Z: the idle word carries a compare value above the
  period (`reload + reload/2`), the comparator never matches, no pulse is
  generated and the counter keeps running.  The next frame is armed by
  resetting the counter and re-programming the chain.
- Frame end is detected by polling the DMA channel-enable bit in
  `up_dshot_frame_done()`; the DMA transfer-complete callback only maintains
  `tc_count` for the bring-up harness and does not drive the frame sequence.
  (The TC interrupt is still enabled on the last descriptor - not removed,
  just unused by the frame path.)
- `up_dshot_idle()` forces the idle level from thread context; writing the
  shadows from the DMA TC interrupt deadlocks against the DMA writing the same
  registers.
- The independent `pollscan` CPU-sampler check prints MISMATCH (polled 0x030b
  vs sent 0x830b) by design: it samples the pin without frame phase and is a
  waveform dump, not a decoder.  Its widths are correct (247/253 ticks =
  37.5 %/75 % of the 662-tick bit at 150 kbit).  Do not read it as a TX
  regression.

**Receive: verified against our own frame, not yet against an ESC.**  The
receiver is GPTMR measure mode + XDMA, one GPTMR capture channel and one XDMA
channel per motor (implementation in `dshot.c`, prototypes in
`px4_arch/dshot.h`):

- The receive pad is muxed to TRGM alt function 17, the TRGM filter runs in
  `rapid_change` mode with length 8, the TRGM input feeds a GPTMR channel in
  `gptmr_work_mode_measure_width` with
  `gptmr_dma_request_on_input_signal_toggle`.
- One XDMA channel moves `(CAPPRD, CAPDTY)` as one double-word transfer per
  capture event into a 32-event buffer in `.ahb_sram`.
- Decoding (`up_bdshot_rx_frame()`): skip the first written pair (stale
  measurement), reject widths outside `bit/4 .. 1.5*bit`, take 16 bits
  MSB-first ("1" when width > bit/2), check the DShot CRC.
- Measure mode latches the width of the *previous* pulse, so the last pulse of
  a frame has no successor edge and only 15 of the 16 widths are ever
  measured.  The decoder therefore accepts either 16 bits with a valid CRC, or
  15 bits plus the missing LSB recovered from the CRC (try both values, keep
  the one that validates).  This is a property of the hardware path, not a
  workaround; it is why the harness now compares the full 16-bit frame instead
  of `frame == sent >> (16 - bits)`.  Captured evidence from the earlier build:
  `recv 0x4185 (15 bits, 16 events) -> MATCH`, and `0x4185 == 0x830b >> 1` -
  the frame decoded with the last width missing and the LSB taken from the CRC.
- Measurements on the final build (`gcapx` path through the production API,
  PE08 -> PE09 jumper; the sweeps transmit on channel 0 only):
  - `pwmv2test gcapxsweep 5` -> **105/105 MATCH**: 35/35 frames at each of
    150/300/600 kbit/s, setpoints 0/10/25/50/75/90/100 % (the same 105/105 was
    already recorded with the earlier inline decoder at f33d3ab926);
  - `pwmv2test gcapxloop 600 50 300` -> **300 frames, all MATCH**;
  - `pwmv2test gcapx 300 75` -> **MATCH** (`sent 0xc185 recv 0xc185`); the pair
    dump shows the stale entry `[0] prd 394849 dty 0` (skipped) followed by
    widths 250/250/125 against a 333-tick bit, i.e. the 75 %/37.5 % levels;
  - `pwmv2test gcapxsweep 3` -> **63/63 MATCH** (earlier run of the same
    build, recorded in the harness console, not in the commit log);
  - `pwmv2test gcapxtimeout` -> wait returns `-ETIMEDOUT` (-116) with no frame
    on the wire (617 us wall time for a 500 us timeout), then a normal frame
    still decodes (`sent 0x830b recv 0x830b`, 66 us).
- Cleanup build (dead receive experiments removed, see "Dead-code inventory").
  The deletion itself is behaviour neutral, but it exposed two real bugs in the
  production receive path, both fixed in the same commit:
  - `bd_rx_dma_reset()` wrote `CTRL = 0` before clearing the DMA status.  That
    write also clears the per-channel interrupt mask bits, i.e. it enables
    every source, and a pending terminal count then raises the XDMA interrupt.
    The channel is not registered with `dma_mgr`, so `dma_mgr_isr_handler()`
    cannot acknowledge it and the CPU stays in the ISR forever (console dead,
    board needs a reset).  Diagnosed by halting the stuck board: PLIC pending
    bit 140 (`IRQn_XDMA`) set, `mcause` 0x8000_000b, PC inside
    `dma_mgr_isr_handler()`.  Fixed by clearing the status first, by masking
    all channel interrupts (the capture window is polled, so the RX channel
    must not interrupt at all) and by clearing the status in
    `up_bdshot_rx_stop()` as well.
  - Result after the fix: three `gcapxsweep` runs back to back (105/105,
    104/105, 62/63) with no hang, `gcapxloop 600 50 300` -> 300/300,
    `gcapxtimeout` both cases OK.
- Known residual, still open: at 150 kbit/s a frame occasionally loses one
  capture event (the dump shows `prd 417` / `prd 917` pairs around a 666-tick
  bit, i.e. one measurement taken at the wrong edge), which fails the
  15-bit + CRC path about one frame in a hundred at that rate (the 104/105 and
  62/63 above).  300 and 600 kbit/s were clean in every run.  This is the same
  signature as the first frame after a cold boot, so it is one measurement
  quality problem at the slowest rate, not a decoding problem.

Not integrated yet: no io_timer / PWM_OUT glue, `CONFIG_DRIVERS_DSHOT` is off,
GCR decoding and eRPM are not implemented, and the test decodes our own frame
instead of an ESC reply.

## Resource map (per motor)

Transmit (shared by all motors):

- PWM1 counter 0, one PWM channel per motor on PE08..PE11
  (`PWM1_P_0..P_3`), duty/edge comparators, one HDMA channel with the 17-node
  descriptor chain, TRGM DMACFG_1 / `PWM1_REQ1` as request source, buffers and
  descriptors in `.ahb_sram`.

Receive (one GPTMR capture channel + one XDMA channel per motor).  The current
implementation is table-driven (`bd_rx_ch[]` in `dshot.c`):

| motor | RX pad (jumper rig) | TRGM input | GPTMR capture | XDMA | DMAMUX source |
| --- | --- | --- | --- | --- | --- |
| 0 | PE09 | `TRGM.P[9]` | GPTMR0 CH2 (`GPTMR0_IN2`) | ch0 | `GPTMR0_2` |
| 1 | PE10 | `TRGM.P[10]` | GPTMR0 CH3 (`GPTMR0_IN3`) | ch1 | `GPTMR0_3` |
| 2 | PE11 | `TRGM.P[11]` | GPTMR1 CH2 (`GPTMR1_IN2`) | ch2 | `GPTMR1_2` |
| 3 | PE12 | `TRGM.P[12]` | GPTMR1 CH3 (`GPTMR1_IN3`) | ch3 | `GPTMR1_3` |

GPTMR channels 2 and 3 have no direct pin, which is why their signal comes in
through TRGM.  The RX pads above are the **jumper rig** used during the
bring-up: the motor 0 line on PE08 is looped back to PE09, and the other
receive leads are PE10..PE12.  A real bidirectional ESC answers on its own
transmit line, so board integration has to repoint the table at the transmit
pads (PE08..PE11) and switch the pad function around the frame (see "Direction
switching"); the four rig entries are not the production wiring.  Only channel
0 (PE08 -> PE09) has been received so far.

Why HDMA (TX) and XDMA (RX) do not contend:

- they are separate DMA controllers (`HPM_HDMA` / `HPM_XDMA`) with separate
  interrupt vectors (`IRQn_HDMA` / `IRQn_XDMA`), separate DMAMUX entries and
  separate channel numbers;
- TX runs its linked chain on an HDMA channel allocated through `dma_mgr`; RX
  programs an XDMA channel directly with `dma_setup_channel()`;
- each RX channel has its own GPTMR capture request source (`GPTMR0_2`,
  `GPTMR0_3`, `GPTMR1_2`, `GPTMR1_3`), so no two channels listen on one request
  line.  An earlier experiment with two HDMA channels on GPTMR capture requests
  hung the board (see the dead-code inventory).

## Decision log (facts that used to live only in code comments)

1. **Measure mode has a one-event lag.** `gptmr_work_mode_measure_width`
   reports the period/width of the pulse that ended *before* the capture
   event, so the first entry belongs to whatever was on the line when the
   window opened (a stale measurement) and the last pulse of the frame has no
   successor edge at all.  Only 15 of the 16 widths can therefore be measured;
   the decoder skips the stale first entry and recovers the missing LSB from
   the CRC (see `up_bdshot_rx_frame()`).  This is a property of the hardware
   path, not a workaround - it is why the harness can compare the full 16-bit
   frame instead of allowing a missing leading bit.  Console evidence from the
   earlier build: `recv 0x4185 (15 bits, 16 events) -> MATCH`, with
   `0x4185 == 0x830b >> 1`.

2. **dty filter window `bit/4 .. 1.5*bit` (inclusive).**  An unwritten entry
   is `(0,0)` and the line shows ~10 ns spikes at the frame start; both would
   decode as a spurious "0".  A real pulse is 37.5 % ("0") or 75 % ("1") of a
   bit period, i.e. `bit*3/8` and `bit*3/4` in GPTMR ticks, so the accepted
   window keeps both with margin and rejects everything else.

3. **Comparator-own-events rule: only channel-own comparators fire.**  The
   `DMA_EN` selector accepts the full compare-index range, but a comparator
   that does not belong to an enabled channel never generates a DMA request.
   Measured with `pwmv2test cmpdma <out_en> <cmp_index>` at 200 kHz: with
   channel 0 active, cmp0..cmp2 fire (32/32 words captured); cmp4..cmp14 set
   `DMA_EN` but produce no request at all (0/32).  This is why the first TX
   attempt with a dedicated cmp8 trigger never fired; the working TX chain uses
   compare index 2 with channel 0 enabled.

4. **DMAV2 needs an abort before the channel is re-configured.**  A receive
   window usually ends before the capture buffer is full (the 48-event chain
   experiment; the production buffer is 32 events), so the transfer is still
   active at stop.  Disabling the channel and writing `CHCTRL[x].CTRL = 0`
   leaves the internal transfer counter and destination pointer untouched; the
   next arm then resumes inside the old buffer (observed: `TRANSIZE 94,
   DST base+8` instead of `96, base`) and the decoder reads `(0,0)` at entry 0.
   The fix is an explicit `dma_abort_channel()` (CHABORT) plus a wait for
   `CHEN` to clear before re-programming - as in `rxf_channel_reset()` and
   `bd_rx_dma_reset()`.

5. **DMAMUX `MUXCFG` is write-only.**  `hpm_dmamux_regs.h` marks it `__W`, so a
   read-back always returns 0 and cannot be used to verify the request routing.
   The mapping must be written explicitly after every channel setup (see
   `bd_rx_dma_config()`; the old experiment did the same in
   `up_dshot_rxd_start/_flush()`) and cleared on deinit: a released channel
   that keeps its MUXCFG entry stays subscribed to the same request source and
   starves the next one (seen as zero captured events on re-arm).

6. **RX buffers in `.ahb_sram`.**  Same rule as the TX buffers: DMA-visible
   memory must be non-cacheable so a window needs no cache maintenance.  All
   receiver buffers (`bd_rx_buf`, `rxg_buf`, `rxf_buf`, `rxp_buf`, `rxi_buf`,
   `rx_buf`, capture ring `cap_buf`) are attributed to `.ahb_sram`.

7. **Build: the arch_dshot target needs the HPM platform include dir.**  The
   receive API is declared in the HPM platform header
   (`hpm_common/include/px4_arch/dshot.h`), which is not on the arch library
   include path by default; `CMakeLists.txt` needs
   `target_include_directories(arch_dshot PRIVATE
   ${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/hpmicro/hpm_common/include)`
   or the header is not found.  (Same file also defines
   `-DHPMSOC_HAS_HPMSDK_DMAV2` for the DMAv2 API.)

## Production receive API contract

The receive path is behind the following API: prototypes in
`platforms/nuttx/src/px4/hpmicro/hpm_common/include/px4_arch/dshot.h`,
implementation in `dshot.c` (`bd_rx_*` helpers, table-driven per channel).
This replaced the experimental `up_dshot_rxd_*` block.

- `up_bdshot_rx_init(mask)` - one-time setup: clocks, receive pads to TRGM
  (alt 17, pull-down), TRGM filter + route, GPTMR measure mode, XDMA channel
  config and DMAMUX mapping.  `mask = 0` means channel 0; calling it again
  with the same mask is a no-op.  Returns OK or a negative errno (`-EIO` if the
  GPTMR channel cannot be configured).
- `up_bdshot_rx_deinit()` - stop the counters, disable the capture DMA
  requests, disable/abort the XDMA channels and clear their DMAMUX mappings.
  No `dma_mgr` resource is involved: XDMA is programmed directly.
- `up_bdshot_rx_ready()` - true when the engine is initialised.
- `up_bdshot_rx_arm()` - clear the buffers, reset the capture count/status,
  re-arm the XDMA transfer from entry 0 and start the capture counter.
- `up_bdshot_rx_flush()` - drop what the arming sequence captured (zero the
  buffer, abort/reset/re-arm the XDMA transfer from entry 0) while the capture
  counter keeps running.  This is what removes the spike latched right after
  the counter starts.
- `up_bdshot_rx_stop()` - stop the capture counter and the XDMA channel
  (disabled and aborted).
- `up_bdshot_rx_done(ch)` - true when more than 16 receive events are buffered
  (a 16-pulse frame plus the stale entry).  A polling hint for callers, not a
  decode; use `_frame()` for the verdict.
- `up_bdshot_rx_events(ch)` - number of capture events (written pairs) in the
  channel buffer.
- `up_bdshot_rx_buffer(ch)` - raw `(period, width)` word pairs of channel
  `ch`; NULL for an invalid/unconfigured channel.
- `up_bdshot_rx_words()` - words per channel buffer: 32 events x 2 words = 64.
- `up_bdshot_rx_frame(ch, *frame, *pairs)` - decode the completed window:
  skip the first written pair (stale measurement), filter dty with the
  `bit/4 .. 1.5*bit` (inclusive) window, assemble 16 bits MSB-first, check the
  DShot CRC.  Because measure mode never latches the width of the last pulse,
  only 15 widths may be available: in that case bit0 is recovered from the CRC
  (both candidate values tried, the one with a valid CRC is kept).  Returns
  `OK` for a CRC-valid frame, `-EAGAIN` while the frame is incomplete,
  `-EINVAL` for `frame == NULL`, an invalid channel or an unknown bit rate.
  `pairs_out` may be NULL.
- `up_bdshot_rx_wait(ch, timeout_us, *frame)` - poll `_frame()` until a valid
  frame arrives or `timeout_us` expires (50 us steps); returns `OK`,
  `-ETIMEDOUT` on timeout, or the `_frame()` error.

`pwmv2test gcapxtimeout` exercises the interrupted-frame path: no frame on the
wire must return `-ETIMEDOUT`, and the next frame must still decode.

## Dead-code inventory (removed)

Five receive experiments (PWMv2 capture, `rxg`, `rxi`, `rxp`, `rxf`) plus the
GPIO port sampler (`rx`) used to be compiled into `dshot.c`; no production code
called them, only the matching `pwmv2test` commands.  They are **gone now**, in
one contiguous cut: `dshot.c` 2286 -> 993 lines and `pwmv2test.cpp`
3322 -> 1195 lines (dshot.c: 19 insertions / 1312 deletions, pwmv2test.cpp:
128 insertions / 2255 deletions; the only non-deletion change in the harness is
one `#include` moved to the top of the file).
The old `up_dshot_rxd_*` block had already been replaced by the production
`up_bdshot_rx_*` implementation (previous section).

The table below stays as the record of what was tried and why it was abandoned.

| block | symbols | what it does | tried / result |
| --- | --- | --- | --- |
| PWMv2 input capture | `up_dshot_capture_init*`, `_set_counter`, `_enable_edges`, `_enable`, `_reset`, `_count`, `_dropped`, `_get` | PWM1 capture IRQ, ring buffer of (channel, edge, counter) | `captest`, `capdiag`, `capscan`, `gcapisr`: status bits fired but the latch did not refresh per edge (constant pos 90 / neg 7 in our configuration), so the edge timestamps were useless.  Abandoned after the SDK survey showed PWMv2 capture is unused by the SDK. |
| GPTMR capture + HDMA (`rxg`) | `up_dshot_rxg_*`, `up_dshot_rxcfg` | GPTMR measure mode, one CAPDTY (or CAPPRD) word per event over a per-channel HDMA channel; 4-channel map | `gcap`, `gcapseq`, `gcapm`, `gcapmseq`: measure mode works, but at 600 kbit entries are lost (1/7) and at 150 kbit only 6/7 matched.  Superseded by the single-channel XDMA receiver. |
| GPIO port sampler (`rx`) | `up_dshot_rx_*` | HDMA samples `GPIO0->DI[E]` at 8 MHz, decodes offline | `rxscan`, `rxscan all`, `rxslow`, `gpiotest`: 8 MHz is 125 ns per sample and DShot pulses are 167 ns..1 us, so the widths are quantised and noisy.  Only useful as a level/release check. |
| GPTMR both-edge ISR (`rxi`) | `up_dshot_rxi_*` | GPTMR both-edge capture, one ISR timestamp per edge, ring buffer | `gcapisr`: ISR latency loses/misorders edges at 600 kbit (41 ticks = 410 ns between edges).  Kept only as a reference receiver. |
| GPTMR + HDMA chain (`rxp`) | `up_dshot_rxp_*` | one linked descriptor per event writes the (CAPPOS, CAPNEG) pair | `gcapd`: captures correctly, but a 48-node chain per frame is heavy; replaced by the fixed-burst variant (`rxf`). |
| GPTMR + HDMA fixed burst (`rxf`) | `up_dshot_rxf_*`, `up_dshot_rxf_state` | one HDMA channel, 2-word `SRC_FIXBURST` reads CAPPOS+CAPNEG per request, decoder pairs the edges | `gcapf`, `gcapfsweep`: 600 kbit 21/21, low rates still lost the first pulse; superseded by the measure-mode receiver, which needs no edge pairing. |

The GPIO sampler block also contains the only `dshot_rx_pins_input/_output`
helpers (`up_dshot_rx_restore_outputs`); the production receiver uses
`bd_rx_pad_input()` instead.  The matching `pwmv2test` commands can be deleted
together with each `dshot.c` block.

### Cleanup result

Performed as described in the checklist above.  The build is the proof: after
the cut `dshot.c` and `pwmv2test.cpp` compile with no unused-function or
unused-variable warnings (the only remaining warning is the SDK's
`gptmr_channel_monitor_config` shadow warning from `hpm_gptmr_drv.h`).
Hardware: three `gcapxsweep` runs back to back, `gcapxloop` 300/300,
`gcapxtimeout` both cases and an unchanged `pollscan` - see "Current status".
Commands kept: `dshot`, `dshotbuf`, `dmadiag`, `pwmregs`, `cmpdma`,
`gpiotest`, `pollscan`, `gcapx`, `gcapxsweep`, `gcapxloop`, `gcapxtimeout`.

## Open items

- **4-motor receive**: only channel 0 (PE09 receive lead) has been exercised.
  The implementation already has four table entries (see "Resource map"), but
  they are the bring-up jumper rig, not production wiring.  For a real ESC the
  table must point at the transmit pads (PE08..PE11) and the pad function must
  be switched around the frame: the TX channel output has to be released for
  the reply window (see "Direction switching").  Neither the four-channel rig
  (receive leads PE09..PE12) nor the single-wire turnaround has been run.
- **1200 kbit/s**: untested.  At the 100 MHz GPTMR capture clock that is ~83
  ticks per bit; the dty window and the one-event lag have to be re-checked.
- **GCR / eRPM**: not implemented.  The receiver decodes the 16-bit DShot
  command frame; the ESC reply is GCR 5b/4b with a 4-bit CRC and must be
  decoded into periods/eRPM before this is bidirectional DShot.
- **Oscilloscope waveform verification**: pending since the idle-word change.
  The current bit timing is inferred from the GPTMR capture only.
- **io_timer / PWM_OUT integration**: `CONFIG_DRIVERS_DSHOT` is still off and
  nothing calls the driver; the bring-up lives in `pwmv2test`.  Integration must
  keep the round-robin telemetry model (one ESC reply window per frame) and
  handle the shared pad (TX push-pull, RX through TRGM).

## Open bug: bit15 is dropped on the first frame after a slower-rate switch

Found while chasing the "one frame in a hundred at 150 kbit/s" residual.  It is
a transmit bug, not a capture or decoder bug, and it is deterministic:

- `pwmv2test gcapxev 150 50 5 600` (frames at 150 kbit/s, one silent 600 kbit/s
  warm-up frame before every printed one) fails 5/5.  With the old commands:
  alternating `gcapx 600 0` / `gcapx 150 0` failed 15/15 on the 150 kbit/s frame,
  while `gcapx 150 0` back to back passed 15/15.
- Direction matters (`gcapxev <rate> 50 2 <warmup>` matrix): 150 after 300
  fails, 300 after 600 fails, 150 after 600 fails; 300 after 150 passes, 600
  after 150 passes, 600 after 600 passes.  Rule: switching to a *slower* rate
  (new reload larger) drops the first pulse, switching to a faster rate is fine.
- The independent CPU sampler (`pollscan 150 50`, no GPTMR/XDMA involved) run
  right after a 600 kbit/s frame decodes 15 bits / 30 edges instead of 16 / 32,
  and the GPTMR capture then starts at b14: the wire has no bit15 pulse at all.
  Everything after it is aligned (14 widths = b14..b1; the last width is the
  usual measure-mode lag), so this is a missing first pulse, not a shift and not
  a decoder problem.

What was tried and ruled out (both reverted, no library change kept):

1. Writing bit15's real comparator value into the duty shadow and latching it
   while the counter is stopped (and stopping the counter at re-arm): no effect,
   still 5/5 failures.  The `on_reload` comparator does not take its value from
   the shadow-lock event.
2. Waiting for the DMA channel-control transfer of word 0 (bit15) to land in the
   shadow (poll `SHADOW_VAL[20]` until it equals `frame_buf[0]`) before starting
   the counter: no effect either, still 5/5.

What the register snapshots say (`gcapxev` prints WORK_CTRL1, CNT_RELOAD_WORK[0],
CMP_VAL_WORK[0], CNT_VAL[0], SHADOW_VAL[20], CMP[0].CFG right after trigger and
right after stop): `CNT_RELOAD_WORK[0]` is correct in both cases
(0x53400 = 1332 ticks = 200 MHz / 150 kHz), so the new period is programmed
before the frame starts.  The working duty value read immediately after
`up_dshot_trigger()` differs: 1998 (the 150 kbit/s idle value) in the working
case, 498 (the 600 kbit/s idle value) in the failing case - the comparator
working register still carries the previous rate's idle value, i.e. the first
latch after the start does not pick up word 0 when the rate went down.

Working hypothesis (not proven at the time): bit15 is played from whatever the
comparator latches at the first reload after the counter starts, so the
channel-control transfer of word 0 has to win that race.  It wins in steady
state and after a switch to a faster rate, but not after a switch to a slower
rate.

### Resolution: free-running counter, chain feeds word 0, frame primed after the request point

The fix removes the race instead of tuning it.  The transmit side now works the
way the counter/shadow hardware wants:

- the PWM counter and the channel output run from configuration time
  (`up_dshot_init()`); the duty shadow holds an out-of-range value, so the line
  is low whenever no frame is being sent;
- the frame is fed entirely by the DMA chain, starting at descriptor 0
  (word 0 = bit15).  The channel-control initial transfer that used to supply
  word 0 is not performed when the channel is enabled mid-period - with a
  free-running counter the chain simply starts at `linked_ptr`, which is why
  `linked_ptr` is `&dshot_desc[0]` now.  (With the old "reset the counter per
  frame" scheme the initial transfer did happen, hence the previous
  `&dshot_desc[1]`.)
- `up_dshot_trigger()` does not touch the counter; it waits for the free-running
  period to pass the compare request point (`CNT < reload/2` minus a margin so
  the request line has definitely dropped) and then enables the channel.  The
  next reload latches word 0 into the comparator, the following requests feed
  word 1..word 16, and the last word keeps the line low.

Rejected alternative, measured: priming the channel right after the reload
(almost a whole period of margin, the current period becomes the start gap)
produced an extra edge on roughly half of the frames at every rate
(`gcapxev 600/300/150 50 300` -> 151, 139, 155 anomalies out of 300;
`gcapxsweep 5` -> 52/105).  Not used.

Measured with the final scheme (`gcapxev` 300 frames per rate unless noted):
150 kbit/s 300/300, 300 kbit/s 300/300, 600 kbit/s 1000/1000 (1000-frame run),
the 600 -> 150 repro `gcapxev 150 50 5 600` 5/5 MATCH (it was 5/5 failure
before), `gcapxsweep 5` twice -> 105/105 both, `gcapxloop 600 50 300` failed
once at iteration 53, `gcapxtimeout` both cases OK, and the independent
`pollscan 150 50` right after a 600 kbit/s frame now sees 16 bits / 32 edges
instead of 15 / 30.

Residual, still open: very rarely (order 1 frame in a few hundred at 600 kbit/s)
the capture reports one event too many (`e17`), i.e. one extra edge appears in
the frame; the frame then fails the 16-bit compare.  It did not appear in the
1000-frame 600 kbit/s run, so it is below 0.1 % there.  Next step if it matters:
compare the wire with `pollscan` at 600 kbit/s in a long loop to decide whether
the extra edge is generated by the transmit side (comparator latch jitter) or by
the capture.

`gcapxev` stays in `pwmv2test` because it reproduces this on demand.