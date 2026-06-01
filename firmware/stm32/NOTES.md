# STM32 Firmware — Engineering Notes

## 2026-04-18 — Main-thread stack overflow (root cause of field IMU flicker + UNKNOWN cascade)

### Observed field symptoms

- System boots and dashboard works correctly at startup.
- After some minutes of operation:
  - IMU pill on the UI begins flickering.
  - `imu_sample_counter` in the Pi telemetry file becomes stuck oscillating
    between exactly two frozen values (e.g. `21535 ↔ 21547`) at the SPI
    transfer cadence.
  - UART commands from the Pi (`STATUS?`, `SAFE`, `IMUON`, `SET_VR_PARAMS`)
    begin to time out repeatedly.
  - The Pi-side recovery state machine (`ws-teleop`) promotes the robot
    to `UNKNOWN` and eventually performs a hard service restart.
- The SPI dataplane on the Pi remains healthy throughout (30 Hz, no CRC
  or length errors) — the telemetry bytes are fresh but their *content*
  is frozen.

### Confirmed root cause

The STM32 is **k_fatal_halted** while the SPI slave DMA continues running
as a hardware peripheral. The two 64-byte TX halves of `spi_tx_buf[]`
were filled by `spi_service_thread_entry` just before the halt, each
containing a successive IMU snapshot. After the halt, DMA alternately
clocks half-0 and half-1 to the master — the exact ping-pong pattern
observed in field.

The kernel halt is triggered by **silent stack overflow on the main
thread**. The main thread (see `src/core/main.c`) is the only dispatcher
of UART commands and runs worst-case handlers on the stack:

- `VR?` — `char msg[320]` + `snprintf` with 40+ varargs compiled with
  `CONFIG_CBPRINTF_FP_SUPPORT=y` (the FP formatter is a ~200 B scratch
  consumer).
- `SET_VR_PARAMS` — 16 floats + 18 uints parsed into local arrays.

With `CONFIG_MAIN_STACK_SIZE=1536` (which slim had reduced from the
baseline 2048), these handlers left negligible headroom on top of the
standing scheduler / IRQ frames. Stack overflow was intermittent and
depended on command sequencing. With `CONFIG_STACK_SENTINEL=n`, the
overflow produced no fault — it silently corrupted adjacent kernel
structures, and the resulting damage led to a delayed hard fault (→
`k_sys_fatal_error_handler` → `k_fatal_halt`) minutes later. The CPU
stopped, the DMA continued, and the Pi observed the signature.

### Minimal fix applied

Two lines in `firmware/stm32/zephyr/prj.conf`:

```diff
-CONFIG_MAIN_STACK_SIZE=1536
+CONFIG_MAIN_STACK_SIZE=2048      # restore baseline headroom
-CONFIG_STACK_SENTINEL=n
+CONFIG_STACK_SENTINEL=y          # detect any future overflow cleanly
```

No code was modified. No devicetree, driver, or runtime behavior change.
The extra 512 B of main stack + sentinel canary overhead fit easily:
post-fix RAM usage is 27712 B / 131072 B (21.1 %).

### Verification after flash (2026-04-18, 6-minute live observation)

Same Raspberry-side sampler methodology, comparing before and after
flashing the fix onto the Nucleo-F446RE via ST-Link (COM3 = VCP).

| Metric | Before fix | After fix |
|---|---|---|
| `imu_sample_counter` unique values in 6 min | **2** (frozen) | **176** (one per sample) |
| Monotonic advance | no | yes, 5881 → 149157 |
| IMU publish rate | ~0 Hz (halted) | **399.8 Hz** (spec 400 Hz) |
| `imu_valid` toggles in 358 s | **140** | **0** |
| `imu_valid=True` fraction | 49 % | **100 %** |
| UART `STATUS?` timeouts | 170 | **0** |
| UART timeouts (any command) | 300 | **0** |
| `STATUS?` successful replies | 0 | **486** @ ~13 ms latency |
| `[STATUS RECOVERY]` events | 66 | **0** |
| HARD ws-teleop restarts | 5 | **0** |
| Robot state on Pi | UNKNOWN cascade | steady `STATUS:IDLE` |
| SPI errors / length mismatches | 0 | 0 |

### Invariants that must be preserved

- **`CONFIG_MAIN_STACK_SIZE` must not fall below 2048.** Any addition of
  new UART commands or of format-heavy response paths in
  `uart/uart_control.c` must be stack-budgeted against the `VR?` /
  `SET_VR_PARAMS` baseline.
- **`CONFIG_STACK_SENTINEL` must remain `y`.** The cost is a few bytes
  per thread; the value is converting silent corruption into a
  debuggable fatal log on USART2.
- The on-board ST-Link VCP (USART2) is the *only* console fatal output
  path — USART1 is reserved for the Pi control plane. Do not redirect.

### Related files (for reference; not modified by this fix)

- `src/core/main.c` — hosts the UART dispatch loop on the main thread.
- `src/uart/uart_control.c` — defines the heavy `VR?` / `SET_VR_PARAMS`
  handlers.
- `src/core/fatal_diag.c` — `k_sys_fatal_error_handler` override that
  confirmed the halt-not-reset behavior.
- `src/spi/hal_spi_slave.c` — double-buffered DMA TX whose stale halves
  reproduced the ping-pong IMU counter signature after halt.
