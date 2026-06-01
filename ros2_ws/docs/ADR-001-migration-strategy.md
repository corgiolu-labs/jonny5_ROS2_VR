# ADR-001 — JONNY5 ROS2 Migration Strategy

- Status: **Accepted**
- Date: 2026-06-01
- Context owner: A. Corgiolu
- Supersedes: the implicit "thin /dev/shm bridge" approach of the initial migration baseline

## Context

JONNY5 is a VR-teleoperated 6-DoF arm. The legacy runtime is mature and tuned:

- **STM32 / Zephyr firmware**: 1 kHz hard real-time loop, IMU at 400 Hz (Madgwick),
  event-driven SPI slave, watchdog 500 ms → IDLE/SAFE. Talks to the Pi over a fixed
  **64-byte J5VR SPI protocol** (`src/spi/j5_protocol.h`).
- **Raspberry Pi controller** (Python): a WebSocket server (`ws_server.py`, port 8557),
  IK (PoE/scipy), DLS head-assist (mode 5), and a separate 100 Hz SPI TX process
  (`teleop/spi_j5vr_tx.py`). The two processes exchange state through JSON files in
  `/dev/shm` (`teleop/shared_state.py`).
- **Video**: MediaMTX / WebRTC / WHEP, end-to-end latency **37–38 ms** (vs ~306 ms MJPEG).
  This is a cardinal thesis result.

The first migration baseline (commits `1f8ec28…c985858`) wrapped the legacy stack in ROS2
by reading/writing the same `/dev/shm` JSON files, validated only with **circular
simulators** (the sim emits exactly the keys the bridge reads). Analysis showed the bridge
publishes several **hollow fields** (`robot_state`, `deadman`, `input_active`, echoed
`mode`/`heartbeat`, `diag_mask`, `rt_step_us`) because the legacy telemetry JSON never
carries them — even though the legacy SPI bridge already *parses* them from the RX frame
and merely logs them.

The target is an **end-to-end test on the real robot Pi, freshly imaged with a clean
Linux install**. micro-ROS on the STM32 is an option if it adds value.

## Decision

Adopt a **native ROS2 control plane on the Pi that reuses the proven SPI data-plane
codec**, and keep the STM32 firmware and the J5VR SPI protocol **unchanged** for the
working-demo milestone. micro-ROS on the STM32 is **gated future work**, not a prerequisite.

Concretely:

1. **Reuse, don't reimplement, the data plane.** `spi_dataplane/j5vr_frame.py`
   (64-byte codec), `spi_dataplane/spi_worker.py` (spidev), and
   `spi_dataplane/j5vr_spi_bridge.py` (frame build + RX telemetry parsing, IMU grace
   window, TELEOPPOSE ACK) are pure, well-isolated logic. They stay byte-for-byte
   identical so the tuned timing/latency results are preserved.

2. **Exploit the existing seam.** `J5VRSPIBridge` already accepts a generic
   `state_provider` (a callable returning the intent dict, or an object with
   `read_intent_from_file()`) and a generic telemetry writer (`write_telemetry_to_file`).
   A native ROS2 `spi_driver_node` injects:
   - a **state provider** backed by the latest `TeleopIntent` subscription, and
   - a **telemetry sink** that publishes ROS2 topics directly from the parsed RX dict.

   This removes the `/dev/shm` JSON round-trip and the asyncio/WebSocket legacy stack,
   and — because we publish from the *real* parsed RX dict — the hollow-field problem
   disappears (deadman / input / mode / heartbeat / diag are all present at that point).

3. **Keep the video path out of ROS2.** MediaMTX/WebRTC stays as-is; it owns the
   37–38 ms result and gains nothing from being wrapped.

4. **Fresh-Pi deployment = ROS2 only.** The freshly imaged Pi runs: clean Linux + SPI
   overlay + ROS2 Jazzy + `spidev` + this workspace (systemd) + MediaMTX. The legacy
   Python WS/IK/DLS services are **not** required to be installed once their essential
   logic is ported (see phases).

5. **STM32 untouched now.** The 64-byte SPI protocol and the 1 kHz loop are the
   load-bearing, measured part of the thesis. They change only in Phase D, behind a
   no-regression gate.

## Alternatives considered

| Option | Pros | Cons | Verdict |
|---|---|---|---|
| **A. Thin `/dev/shm` bridge** (baseline) | Lowest effort; legacy untouched | ROS2 is a shim, not a real migration; lossy JSON → hollow topics; needs the whole legacy stack on the fresh Pi; not defensible as "migration" | Rejected as end-state (kept only as the conceptual starting point) |
| **B. Native ROS2 reusing the codec** (chosen) | Genuine ROS2 graph; preserves tuned codec + results; clean fresh-Pi deploy; faithful telemetry | Must port orchestration (loop, modes, IK/DLS) incrementally | **Chosen** |
| **C. micro-ROS on STM32 now** | "Full ROS2 down to the MCU"; impressive | High risk to the 1 kHz loop and 37–38 ms result; XRCE-DDS agent + serialization; transport usually serial/UDP, not SPI-slave; large debug surface on fresh hw | Deferred to Phase D, gated |

## Consequences

**Positive**
- The migration is real and demonstrable, not a wrapper — good for both the thesis and
  the engineering showcase ("every claim must be true").
- The hardest, most-tuned code (frame codec, RX parsing) is preserved verbatim → low risk
  to the cardinal latency/timing results.
- The fresh Pi gets a self-contained ROS2 deployment.

**Negative / risks**
- Advanced behaviors (IK live, DLS mode 5, camctrl, TELEOPPOSE) must be ported before
  full feature parity; the first hardware light targets core teleop (manual/relative/head).
- `ik_solver` / `head_assist_dls` carry legacy config-path coupling
  (`poe_params_manager`, `runtime_config_paths`) to untangle in Phase C.
- A single intent owner must be enforced: the ROS2 `vr_bridge` vs the legacy `ws_server` (8557)
  must not both feed intent. Resolved: in the native deployment the legacy `ws_server` is not run;
  the WebXR viewer's intent stream is repointed to the ROS2 bridge (configurable `bind_port`,
  default 8567 — repoint the HTTPS `/ws` proxy or the viewer URL).

### Teleop transport boundary (resolved)

The SPI `TeleopIntent` carries only what the 64-byte J5VR frame encodes: motion (mode, joy,
pitch, yaw, intensity, grip), headset orientation, buttons, the mode-5 arm extension, and
**camctrl** (focus/zoom/conv -> frame marker `C`, validated). Everything else on the legacy WS
API is *not* SPI intent and is deferred to Phase C:

- **TELEOPPOSE / HOME / PARK** travel over **UART** (`{type:"uart"}` -> `ws_handlers_uart`), not the
  SPI frame -> a dedicated ROS2 UART node.
- **set_vr_mode, settings, poe_params, vr_calib, vr_zoom_state, self_test** are control-plane RPCs
  of the legacy `ws_server` -> ROS2 services/params in Phase C.

## Phased rollout

- **Phase A — software + dry-run (no hardware, validated on WSL/ROS2 Jazzy)**
  Native `spi_driver_node` wrapping `J5VRSPIBridge` with a mock SPI (J5 loopback) for
  dry-run; refine `vr_bridge_node`; native bringup/params; fresh-Pi deploy assets.
- **Phase B — real-hardware bring-up (fresh Pi)**
  Deploy, enable SPI, first light: real telemetry on topics; manual/head teleop moves the
  arm; **measure end-to-end latency and confirm no regression vs 37–38 ms**.
- **Phase C — advanced parity**
  Port IK live + DLS assist (mode 5, gains gainM=0.15 / lambdaMax=0.12 / …) + camctrl +
  TELEOPPOSE into ROS2 nodes/libraries, preserving tuning numbers.
- **Phase D — micro-ROS evaluation (gated)**
  Prototype micro-ROS on the STM32 as an alternative transport. Adopt **only** if it
  preserves the 1 kHz loop and the latency budget; otherwise it remains documented
  future work.

## Verification

- Phase A: `colcon build` + `ros2 launch` in mock mode; assert the graph publishes
  `joint_states`, `imu/data`, `jonny5/status`, `jonny5/spi/telemetry` with realistic
  values, and that `TeleopIntent` reaches the driver — all without the legacy stack.
- Phase B: latency measurement compared against the documented MJPEG/WebRTC baselines.
