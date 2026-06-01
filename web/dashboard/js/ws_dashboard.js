/**
 * ws_dashboard.js — Thin wrapper per la Home dashboard JONNY5-4.0
 *
 * Delega connessione WS, routing messaggi e utility comuni a j5_common.js.
 * Registra qui solo la logica specifica della Home (aggiornamento card
 * Servo, IMU, VR Headset, System, Temp pill, SPI Hz).
 */

import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerAckHandler,
  registerOpenHandler,
  registerSelfTestStatusHandler,
  registerSelfTestResultHandler,
  registerSetposeDoneHandler,
  quatToEuler,
  setPill,
  sendCommand,
  addLog,
  s,
} from "../../shared/js/j5_common.js";

function quatMul(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

function quatInvUnit(q) {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

function setDisplayRefLabel(text) {
  const el = document.getElementById("imu-display-ref");
  if (!el) return;
  el.textContent = text || "–";
  // Raw = grigio neutro; qualsiasi altra ref (es. "Body") = evidenziato
  el.className = "diag-value state-pill " + (text && text !== "Raw" ? "state-on" : "state-unknown");
}

function setSelfTestCell(id, value) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = value || "–";
}

function onSelfTestStatus(msg) {
  const state = String(msg?.state || "");
  const message = String(msg?.message || "");
  if (state === "running") {
    _selfTestActive = true;
    if (message.includes("Starting self-test")) {
      _selfTestImuZeroQuat = null;
      _selfTestImuZeroPending = false;
      setDisplayRefLabel(_homeImuRefQuat ? "Home" : "Raw");
    }
    if (message.includes("Zeroing IMU")) {
      _selfTestImuZeroQuat = null;
      _selfTestImuZeroPending = true;
      if (_latestImuQuat) {
        _selfTestImuZeroQuat = { ..._latestImuQuat };
        _selfTestImuZeroPending = false;
        setDisplayRefLabel("Zeroed for Self-Test");
      }
    }
  } else {
    _selfTestActive = false;
    _selfTestImuZeroQuat = null;
    _selfTestImuZeroPending = false;
    setDisplayRefLabel(_homeImuRefQuat ? "Home" : "Raw");
  }
  setSelfTestCell("self-test-status", msg?.state || "Idle");
  setSelfTestCell("self-test-message", msg?.message || "Ready");
  if (state === "running") {
    setSelfTestCell("self-test-bands", "–");
  }
  if (state === "running" && message.includes("Starting self-test")) {
    setSelfTestCell("self-test-dynamic-motion", "–");
    setSelfTestCell("self-test-vibration", "–");
    setSelfTestCell("self-test-modal-vibration", "–");
  }
  if (state === "failed") {
    setSelfTestCell("self-test-dynamic-motion", "–");
    setSelfTestCell("self-test-vibration", "–");
    setSelfTestCell("self-test-modal-vibration", "–");
    setSelfTestCell("self-test-bands", "–");
  }
}

function formatVibrationPeaks(msg) {
  const peaks = msg?.imu_vibration_peaks_hz;
  if (Array.isArray(peaks) && peaks.length > 0) {
    return peaks.map((hz) => `${Number(hz).toFixed(2)} Hz`).join(", ");
  }
  const err = msg?.imu_vibration_error;
  if (typeof err === "string" && err.trim()) {
    return "Unavailable";
  }
  return "–";
}

function formatModalVibrationPeaks(msg) {
  const peaks = msg?.imu_modal_peaks_hz;
  if (Array.isArray(peaks) && peaks.length > 0) {
    return peaks.map((hz) => `${Number(hz).toFixed(2)} Hz`).join(", ");
  }
  const err = msg?.imu_modal_error;
  if (typeof err === "string" && err.trim()) {
    return "Unavailable";
  }
  return "–";
}

function formatDynamicResponsePeaks(msg) {
  const peaks = msg?.dynamic_response_peaks_hz ?? msg?.dynamic_motion_peaks_hz;
  if (Array.isArray(peaks) && peaks.length > 0) {
    return peaks.map((hz) => `${Number(hz).toFixed(2)} Hz`).join(", ");
  }
  const err = msg?.dynamic_response_error ?? msg?.dynamic_motion_error;
  if (typeof err === "string" && err.trim()) {
    return "Unavailable";
  }
  return "–";
}

/** Raccoglie tutti i picchi numerici dai tre canali (dedup stretto). */
function _selfTestAllPeaksHz(msg) {
  const raw = [];
  const pushArr = (arr) => {
    if (!Array.isArray(arr)) return;
    for (const x of arr) {
      const n = Number(x);
      if (Number.isFinite(n) && n > 0.05) raw.push(n);
    }
  };
  pushArr(msg?.dynamic_response_peaks_hz);
  pushArr(msg?.dynamic_motion_peaks_hz);
  pushArr(msg?.imu_vibration_peaks_hz);
  pushArr(msg?.imu_modal_peaks_hz);
  raw.sort((a, b) => a - b);
  const dedup = [];
  for (const p of raw) {
    if (!dedup.length || Math.abs(p - dedup[dedup.length - 1]) > 0.2) dedup.push(p);
  }
  return dedup;
}

/**
 * Sintesi compatta (max 3 bande) da picchi dyn / exp / modal.
 * Es.: ~2–4, ~5–7, ~11–12 Hz
 */
function formatSelfTestBands(msg) {
  const peaks = _selfTestAllPeaksHz(msg);
  if (peaks.length < 2) return "–";

  const MERGE_HZ = 2.75;
  const clusters = [];
  let cur = [peaks[0]];
  for (let i = 1; i < peaks.length; i++) {
    if (peaks[i] - peaks[i - 1] <= MERGE_HZ) {
      cur.push(peaks[i]);
    } else {
      clusters.push(cur);
      cur = [peaks[i]];
    }
  }
  clusters.push(cur);

  const weight = (c) => c.length * (Math.max(...c) - Math.min(...c) + 1);
  const top = clusters
    .slice()
    .sort((a, b) => weight(b) - weight(a))
    .slice(0, 3)
    .sort((a, b) => a[0] - b[0]);

  const nd = "\u2013";
  const parts = top.map((c) => {
    const mn = Math.min(...c);
    const mx = Math.max(...c);
    const lo = Math.max(0, Math.floor(mn));
    const hi = Math.ceil(mx);
    if (hi <= lo + 1 && mx - mn < 1.25) {
      return `~${Math.round((mn + mx) / 2)}`;
    }
    return `~${lo}${nd}${hi}`;
  });
  return `${parts.join(", ")} Hz`;
}

function onSelfTestResult(msg) {
  _selfTestActive = false;
  _selfTestImuZeroQuat = null;
  _selfTestImuZeroPending = false;
  setDisplayRefLabel(_homeImuRefQuat ? "Home" : "Raw");
  setSelfTestCell("self-test-status", "Completed");
  setSelfTestCell("self-test-message", msg?.message || "Completed");
  setSelfTestCell("self-test-dynamic-motion", formatDynamicResponsePeaks(msg));
  setSelfTestCell("self-test-vibration", formatVibrationPeaks(msg));
  setSelfTestCell("self-test-modal-vibration", formatModalVibrationPeaks(msg));
  setSelfTestCell("self-test-bands", formatSelfTestBands(msg));
  setSelfTestCell("self-test-result", msg?.result || "–");
  const axes = msg?.axes || {};
  setSelfTestCell("self-test-base", axes.base?.classification || "–");
  setSelfTestCell("self-test-shoulder", axes.spalla?.classification || "–");
  setSelfTestCell("self-test-elbow", axes.gomito?.classification || "–");
  setSelfTestCell("self-test-yaw", axes.yaw?.classification || "–");
  setSelfTestCell("self-test-pitch", axes.pitch?.classification || "–");
  setSelfTestCell("self-test-roll", axes.roll?.classification || "–");
  addLog(`SELF TEST ${msg?.result || "completed"}`);
}

// ---------------------------------------------------------------------------
// IMU display: riferimento visivo = ultima HOME completata (dopo setpose_done).
// ---------------------------------------------------------------------------
const HOME_SETTLE_AFTER_DONE_MS = 1200;
const HOME_CAPTURE_TIMEOUT_MS = 35000;

let _spiLastPkt = null;
let _spiLastPktTime = null;
const _spiHzSamples = [];
let _latestImuQuat = null;
let _selfTestImuZeroQuat = null;
let _selfTestImuZeroPending = false;
let _selfTestActive = false;
let _homeImuRefQuat = null;
let _pendingHomeImuCapture = false;
let _homeCaptureDeadlineTs = 0;
let _homeCaptureGeneration = 0;
let _homeSettleTimer = null;

function onSetposeDone() {
  if (!_pendingHomeImuCapture) return;
  if (Date.now() > _homeCaptureDeadlineTs) {
    _pendingHomeImuCapture = false;
    return;
  }
  const g = _homeCaptureGeneration;
  _pendingHomeImuCapture = false;
  if (_homeSettleTimer) {
    clearTimeout(_homeSettleTimer);
    _homeSettleTimer = null;
  }
  _homeSettleTimer = setTimeout(() => {
    _homeSettleTimer = null;
    if (g !== _homeCaptureGeneration) return;
    if (_selfTestActive) return;
    if (!_latestImuQuat) return;
    _homeImuRefQuat = { ..._latestImuQuat };
    setDisplayRefLabel("Home");
  }, HOME_SETTLE_AFTER_DONE_MS);
}

if (typeof window !== "undefined") {
  window.j5NotifyHomeRequested = function j5NotifyHomeRequested() {
    if (_homeSettleTimer) {
      clearTimeout(_homeSettleTimer);
      _homeSettleTimer = null;
    }
    _homeCaptureGeneration += 1;
    _pendingHomeImuCapture = true;
    _homeCaptureDeadlineTs = Date.now() + HOME_CAPTURE_TIMEOUT_MS;
  };
  window.j5CancelHomeImuCapture = function j5CancelHomeImuCapture() {
    _pendingHomeImuCapture = false;
    if (_homeSettleTimer) {
      clearTimeout(_homeSettleTimer);
      _homeSettleTimer = null;
    }
  };
}

// ---------------------------------------------------------------------------
// Callback telemetria — specifica della Home dashboard
// ---------------------------------------------------------------------------
function onTelemetry(data) {
  if (data.servo_deg_B !== undefined) s("s-base",   data.servo_deg_B);
  if (data.servo_deg_S !== undefined) s("s-spalla",  data.servo_deg_S);
  if (data.servo_deg_G !== undefined) s("s-gomito",  data.servo_deg_G);
  if (data.servo_deg_R !== undefined) s("s-roll",    data.servo_deg_R);
  if (data.servo_deg_P !== undefined) s("s-pitch",   data.servo_deg_P);
  if (data.servo_deg_Y !== undefined) s("s-yaw",     data.servo_deg_Y);

  if (
    data.imu_q_w !== undefined &&
    data.imu_q_x !== undefined &&
    data.imu_q_y !== undefined &&
    data.imu_q_z !== undefined
  ) {
    _latestImuQuat = {
      w: data.imu_q_w,
      x: data.imu_q_x,
      y: data.imu_q_y,
      z: data.imu_q_z,
    };
    if (_selfTestImuZeroPending && _latestImuQuat) {
      _selfTestImuZeroQuat = { ..._latestImuQuat };
      _selfTestImuZeroPending = false;
      setDisplayRefLabel("Zeroed for Self-Test");
    }
    let displayQuat = _latestImuQuat;
    if (_selfTestActive && _selfTestImuZeroQuat) {
      displayQuat = quatMul(quatInvUnit(_selfTestImuZeroQuat), _latestImuQuat);
      setDisplayRefLabel("Zeroed for Self-Test");
    } else if (_homeImuRefQuat) {
      displayQuat = quatMul(quatInvUnit(_homeImuRefQuat), _latestImuQuat);
      setDisplayRefLabel("Home");
    } else if (!_selfTestActive) {
      setDisplayRefLabel("Raw");
    }
    const e = quatToEuler(displayQuat.w, displayQuat.x, displayQuat.y, displayQuat.z);
    s("imu-roll",  e.roll.toFixed(1));
    s("imu-pitch", e.pitch.toFixed(1));
    s("imu-yaw",   e.yaw.toFixed(1));
    s("imu-qw", data.imu_q_w.toFixed(3));
    s("imu-qx", data.imu_q_x.toFixed(3));
    s("imu-qy", data.imu_q_y.toFixed(3));
    s("imu-qz", data.imu_q_z.toFixed(3));
  }
  if (data.imu_temp !== undefined) {
    const el = document.getElementById("imu-temp");
    if (el) {
      el.textContent = data.imu_temp.toFixed(1) + " °C";
      el.className = "diag-value state-pill " + (data.imu_temp > 60 ? "state-off" : "state-on");
    }
  }

  if (data.vr_roll  !== undefined) s("vr-roll",  data.vr_roll.toFixed(1));  else s("vr-roll",  "–");
  if (data.vr_pitch !== undefined) s("vr-pitch", data.vr_pitch.toFixed(1)); else s("vr-pitch", "–");
  if (data.vr_yaw   !== undefined) s("vr-yaw",   data.vr_yaw.toFixed(1));   else s("vr-yaw",   "–");
  if (data.vr_quat_w !== undefined) s("vr-qw", data.vr_quat_w.toFixed(3)); else s("vr-qw", "–");
  if (data.vr_quat_x !== undefined) s("vr-qx", data.vr_quat_x.toFixed(3)); else s("vr-qx", "–");
  if (data.vr_quat_y !== undefined) s("vr-qy", data.vr_quat_y.toFixed(3)); else s("vr-qy", "–");
  if (data.vr_quat_z !== undefined) s("vr-qz", data.vr_quat_z.toFixed(3)); else s("vr-qz", "–");

  const VR_MODE_LABELS = { 0: "CALIB", 1: "Pose", 2: "Manual", 3: "Head", 4: "Hybrid", 5: "Assist" };

  if (data.spi_packet_index !== undefined) {
    s("sys-pkt", data.spi_packet_index);
    const now = Date.now();
    if (_spiLastPkt !== null && _spiLastPktTime !== null && data.spi_packet_index !== _spiLastPkt) {
      const dtSec = (now - _spiLastPktTime) / 1000;
      const dpkt  = data.spi_packet_index - _spiLastPkt;
      if (dtSec > 0 && dpkt > 0) {
        const hz = dpkt / dtSec;
        _spiHzSamples.push(hz);
        if (_spiHzSamples.length > 8) _spiHzSamples.shift();
        const avg = _spiHzSamples.reduce((a, b) => a + b, 0) / _spiHzSamples.length;
        s("sys-spi-hz", avg.toFixed(1) + " Hz");
      }
    }
    _spiLastPkt = data.spi_packet_index;
    _spiLastPktTime = now;
  }

  if (data.intent_mode !== undefined) {
    s("sys-vr-mode", VR_MODE_LABELS[data.intent_mode] ?? ("?" + data.intent_mode));
  }

  if (data.intent_heartbeat !== undefined) s("sys-hb", data.intent_heartbeat);

  if (data.intent_age_ms !== undefined) {
    const age = data.intent_age_ms;
    s("sys-intent-age", age < 2000 ? age + " ms" : (age / 1000).toFixed(1) + " s");
  }

  if (data.robot_state !== undefined) {
    const el = document.getElementById("pill_robot_state");
    if (el) {
      const st = data.robot_state;
      el.textContent = st;
      if (st === "IDLE") {
        el.className = "diag-value state-pill state-on";
      } else if (st === "STOPPED") {
        el.className = "diag-value state-pill state-off";
      } else if (st === "SAFE") {
        el.className = "diag-value state-pill state-warn";
      } else {
        el.className = "diag-value state-pill state-unknown";
      }
    }
  }
}

function onAck(data) {
  if (data.teleop_pose_ack) {
    return;
  }
  if (data.status) {
    const v = data.status;
    if (v === "RUNNING" || v === "IDLE" || v === "SAFE") setPill("pill_stm32", "on");
    else if (v === "STOPPED")                             setPill("pill_stm32", "off");
    else                                                  setPill("pill_stm32", "unknown");
  }
}

registerTelemetryHandler(onTelemetry);
registerAckHandler(onAck);
registerSelfTestStatusHandler(onSelfTestStatus);
registerSelfTestResultHandler(onSelfTestResult);
registerSetposeDoneHandler(onSetposeDone);

registerOpenHandler(() => {
  sendCommand("apply_saved_vr_config", {});
});

connectJ5Dashboard();

export { sendCommand };
