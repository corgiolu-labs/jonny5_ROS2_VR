/**
 * vr.js — Dashboard VR: anteprima video low-latency (MediaMTX WebRTC /cam0 /cam1) + calibrazione stereo + telemetria
 *
 * Flusso dati:
 *   - Video:  iframe same-origin → whep_cam_embed.html (WHEP via /api/webrtc-whep); richiede video_pipeline: webrtc da /api/video-config
 *   - Calib:  parametri inviati/ricevuti via WS-TELEOP (porta 8557)
 *             tipo "vr_calib" → broadcast a tutti i client, incluso viewer_stereo_xr.html
 *   - Telemetria / comandi UART: via WS-TELEOP (già gestito da j5_common.js)
 */

import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerVrCalibHandler,
  registerVrZoomCommandHandler,
  quatToEuler,
  sendCommand,
  debounce,
  s,
} from "../../../shared/js/j5_common.js";

function _el(id) { return document.getElementById(id); }

// ────────────────────────────────────────────────────────────────────────────
// Telemetria (servo + IMU + VR headset)
// ────────────────────────────────────────────────────────────────────────────
// mode → button ID per il pannello mirror (display-only)
const _MODE_BTN_MAP = { 0: "btnVR", 2: "btnTeleop", 3: "btnTeleopHead", 4: "btnTeleopHybrid", 5: "btnTeleopIk" };

function _syncMirrorModeButtons(activeMode) {
  Object.values(_MODE_BTN_MAP).forEach((id) => {
    const b = _el(id);
    if (b) b.classList.remove("active");
  });
  const activeId = _MODE_BTN_MAP[activeMode];
  if (activeId) {
    const b = _el(activeId);
    if (b) b.classList.add("active");
  }
}

registerTelemetryHandler((data) => {
  if (data.servo_deg_B !== undefined) s("vr-s-base",   data.servo_deg_B);
  if (data.servo_deg_S !== undefined) s("vr-s-spalla",  data.servo_deg_S);
  if (data.servo_deg_G !== undefined) s("vr-s-gomito",  data.servo_deg_G);
  if (data.servo_deg_R !== undefined) s("vr-s-roll",    data.servo_deg_R);
  if (data.servo_deg_P !== undefined) s("vr-s-pitch",   data.servo_deg_P);
  if (data.servo_deg_Y !== undefined) s("vr-s-yaw",     data.servo_deg_Y);

  if (data.imu_q_w !== undefined) {
    const e = quatToEuler(data.imu_q_w, data.imu_q_x, data.imu_q_y, data.imu_q_z);
    s("vr-imu-roll",  e.roll.toFixed(1));
    s("vr-imu-pitch", e.pitch.toFixed(1));
    s("vr-imu-yaw",   e.yaw.toFixed(1));
    s("vr-imu-qw", data.imu_q_w.toFixed(3));
    s("vr-imu-qx", data.imu_q_x.toFixed(3));
    s("vr-imu-qy", data.imu_q_y.toFixed(3));
    s("vr-imu-qz", data.imu_q_z.toFixed(3));
  }

  if (data.vr_roll  !== undefined) s("vr-hmd-roll",  data.vr_roll.toFixed(1));
  if (data.vr_pitch !== undefined) s("vr-hmd-pitch", data.vr_pitch.toFixed(1));
  if (data.vr_yaw   !== undefined) s("vr-hmd-yaw",   data.vr_yaw.toFixed(1));
  if (data.vr_quat_w !== undefined) s("vr-hmd-qw", data.vr_quat_w.toFixed(3)); else s("vr-hmd-qw", "–");
  if (data.vr_quat_x !== undefined) s("vr-hmd-qx", data.vr_quat_x.toFixed(3)); else s("vr-hmd-qx", "–");
  if (data.vr_quat_y !== undefined) s("vr-hmd-qy", data.vr_quat_y.toFixed(3)); else s("vr-hmd-qy", "–");
  if (data.vr_quat_z !== undefined) s("vr-hmd-qz", data.vr_quat_z.toFixed(3)); else s("vr-hmd-qz", "–");

  if (data.robot_state !== undefined) {
    const cls = {IDLE: "state-on", STOPPED: "state-off", SAFE: "state-warn"};
    const badge = _el("vr-robot-state-badge");
    if (badge) {
      badge.textContent = data.robot_state;
      badge.className = cls[data.robot_state] || "";
    }
  }

  if (data.intent_mode !== undefined) {
    _syncMirrorModeButtons(data.intent_mode);
  }
});


// ────────────────────────────────────────────────────────────────────────────
// Calibrazione stereo — parametri
// ────────────────────────────────────────────────────────────────────────────
const CALIB_KEYS = ["convPx", "vertPx", "rollDeg0", "rollDeg1", "zoom0", "zoom1", "focusPos0", "focusPos1"];
const CALIB_DEFAULTS = { convPx: 0, vertPx: 0, rollDeg0: 0, rollDeg1: 0, zoom0: 1.0, zoom1: 1.0, focusPos0: 1.0, focusPos1: 1.0 };
const CALIB_META = {
  convPx:   { min: -200, max: 200,  step: 1,     label: "Convergenza (px)",  fmt: (v) => Math.trunc(v) + " px" },
  vertPx:   { min: -200, max: 200,  step: 1,     label: "Offset verticale",  fmt: (v) => Math.trunc(v) + " px" },
  rollDeg0: { min: -5,   max: 5,    step: 0.1,   label: "Roll cam0 (°)",     fmt: (v) => v.toFixed(1) + "°"   },
  rollDeg1: { min: -5,   max: 5,    step: 0.1,   label: "Roll cam1 (°)",     fmt: (v) => v.toFixed(1) + "°"   },
  zoom0:    { min: 0.50, max: 2.00, step: 0.002, label: "Zoom cam0",         fmt: (v) => v.toFixed(3)         },
  zoom1:    { min: 0.50, max: 2.00, step: 0.002, label: "Zoom cam1",         fmt: (v) => v.toFixed(3)         },
  focusPos0:{ min: 0,    max: 10,   step: 0.01,  label: "Focus cam0",        fmt: (v) => v.toFixed(2)         },
  focusPos1:{ min: 0,    max: 10,   step: 0.01,  label: "Focus cam1",        fmt: (v) => v.toFixed(2)         },
};

const calib = { ...CALIB_DEFAULTS };

function _clamp(v, k) {
  const m = CALIB_META[k];
  return Math.max(m.min, Math.min(m.max, v));
}

function _updateSliderUI(k) {
  const slider = _el("calib-slider-" + k);
  const label  = _el("calib-val-" + k);
  if (slider) slider.value = calib[k];
  if (label)  label.textContent = CALIB_META[k].fmt(calib[k]);
}

function _updateAllSliders() {
  CALIB_KEYS.forEach(_updateSliderUI);
}

/** Invia {type:"vr_calib", ...valori} al server WS-TELEOP per broadcast al visore. */
function _sendCalib() {
  const payload = { ...calib };
  sendCommand("vr_calib", payload);
}

function _onSliderChange(k, rawVal) {
  const m = CALIB_META[k];
  let v = parseFloat(rawVal);
  if (Number.isNaN(v)) v = CALIB_DEFAULTS[k];
  if (k === "convPx" || k === "vertPx") v = Math.trunc(v);
  calib[k] = _clamp(v, k);
  _updateSliderUI(k);
  _sendCalib();
}

function _initCalibSliders() {
  CALIB_KEYS.forEach((k) => {
    const slider = _el("calib-slider-" + k);
    if (!slider) return;
    const m = CALIB_META[k];
    slider.min  = m.min;
    slider.max  = m.max;
    slider.step = m.step;
    slider.value = calib[k];
    slider.addEventListener("input", debounce(() => _onSliderChange(k, slider.value), 50));
  });
  _updateAllSliders();
}

/** Riceve aggiornamento calibrazione dal server (broadcast da un altro client, es. il visore). */
function _applyCalibFromServer(msg) {
  let changed = false;
  CALIB_KEYS.forEach((k) => {
    if (k in msg && typeof msg[k] === "number") {
      calib[k] = _clamp(msg[k], k);
      changed = true;
    }
  });
  if (changed) _updateAllSliders();
}

/** Reset a valori di default. */
function resetCalib() {
  CALIB_KEYS.forEach((k) => { calib[k] = CALIB_DEFAULTS[k]; });
  _updateAllSliders();
  _sendCalib();
}

// Riceve calibrazione broadcast dal server (inviata da un altro client, es. il visore)
registerVrCalibHandler(_applyCalibFromServer);

// ────────────────────────────────────────────────────────────────────────────
// Video low-latency: iframe HTTPS same-origin → embed WHEP (no mixed content). Richiede /api/video-config → webrtc.
// ────────────────────────────────────────────────────────────────────────────
const WHEP_EMBED_HTML = "/dashboard/whep_cam_embed.html";

function _whepDashboardEmbedUrl(camPath) {
  const u = new URL(WHEP_EMBED_HTML, window.location.origin);
  u.searchParams.set("path", camPath);
  return u.href;
}

let _videoStarted = false;

function startVideo() {
  if (_videoStarted) return;
  const btn = _el("vr-btn-start");
  if (btn) btn.disabled = true;
  const webrtcRow = _el("vr-webrtc-row");
  const descEl = _el("vr-video-desc");
  const errEl = _el("vr-video-err");

  const fail = (msg) => {
    if (errEl) {
      errEl.textContent = msg;
      errEl.style.display = "block";
    }
    console.error("[vr.js] startVideo:", msg);
    if (btn) btn.disabled = false;
  };

  fetch("/api/video-config")
    .then(async (r) => {
      let data = {};
      try {
        data = await r.json();
      } catch (_) {
        data = {};
      }
      if (!r.ok) {
        const detail = data.error || r.statusText || String(r.status);
        throw new Error(detail || "video-config request failed");
      }
      return data;
    })
    .then((cfg) => {
      if (!cfg || cfg.video_pipeline !== "webrtc") {
        const why = (cfg && cfg.error) || "server did not report video_pipeline: webrtc";
        throw new Error(why);
      }
      _videoStarted = true;
      if (errEl) {
        errEl.textContent = "";
        errEl.style.display = "none";
      }
      if (webrtcRow) webrtcRow.style.display = "flex";
      const iframe0 = _el("vr-webrtc-iframe0");
      const iframe1 = _el("vr-webrtc-iframe1");
      const status0 = _el("vr-cam-status-webrtc0");
      const status1 = _el("vr-cam-status-webrtc1");
      if (iframe0) {
        iframe0.src = _whepDashboardEmbedUrl("cam0");
        if (status0) status0.textContent = "WebRTC cam0 (WHEP)";
        iframe0.onerror = () => { if (status0) status0.textContent = "cam0 non disponibile"; };
      }
      if (iframe1) {
        iframe1.src = _whepDashboardEmbedUrl("cam1");
        if (status1) status1.textContent = "WebRTC cam1 (WHEP)";
        iframe1.onerror = () => { if (status1) status1.textContent = "cam1 non disponibile"; };
      }
      if (descEl) {
        descEl.textContent =
          "Anteprima WebRTC in bassa latenza via HTTPS same-origin e proxy WHEP (/api/webrtc-whep), come il visore VR.";
      }
    })
    .catch((e) => {
      fail(e && e.message ? e.message : String(e));
    });
}

// ────────────────────────────────────────────────────────────────────────────
// Azioni condivise (riusate da pulsanti originali + mirror design panel)
// ────────────────────────────────────────────────────────────────────────────
function _actionEnable() {
  sendCommand("uart", { cmd: "SAFE"   });
  sendCommand("uart", { cmd: "ENABLE" });
}

function _actionStop() {
  sendCommand("uart", { cmd: "STOP" });
}

function _actionVrPose() {
  sendCommand("uart", { cmd: "TELEOPPOSE" });
}

function _actionHome() {
  sendCommand("uart", { cmd: "HOME" });
}

function _actionPark() {
  sendCommand("uart", { cmd: "PARK" });
}

function _actionDemo() {
  sendCommand("uart", { cmd: "DEMO" });
}

// ────────────────────────────────────────────────────────────────────────────
// Init al caricamento DOM
// I moduli ES sono defer per default: DOMContentLoaded può già essere fired
// quando il modulo esegue. Stesso pattern usato in dashboard.js.
// ────────────────────────────────────────────────────────────────────────────
// ─── Zoom comune (pulsanti +/-/reset, indipendenti dal viewer XR) ─────────
const ZOOM_COMMON_MIN = 1.0;
const ZOOM_COMMON_MAX = 6.0;
let _currentZoomCommon = 1.0;

function _zoomCommonApplyLocal(z) {
  const v = Math.max(ZOOM_COMMON_MIN, Math.min(ZOOM_COMMON_MAX, Number(z) || 1.0));
  _currentZoomCommon = v;
  const cur = _el("zoom-common-val");
  if (cur) cur.textContent = v.toFixed(2) + "×";
  // Applica CSS zoom: in vr.html i preview sono dentro iframe (cross-document)
  // quindi scaliamo l'iframe stesso. Mantengo selettore inclusivo per altri
  // contesti che usano <video> diretto.
  document.querySelectorAll(".vr-iframe, .vr-cam-video, video.vr-video, .video-panel video")
    .forEach((el) => el.style.setProperty("--vr-zoom", String(v)));
}

function _zoomCommonStep() {
  const raw = Number(_el("zoom-common-step")?.value);
  return (Number.isFinite(raw) && raw > 0) ? raw : 0.2;
}

function _zoomCommonSend(action, value) {
  try { sendCommand("vr_zoom_command", { action: action, value: Number(value) || 0 }); } catch (_) {}
}

function _initZoomCommonControls() {
  const btnPlus  = _el("btn-zoom-plus");
  const btnMinus = _el("btn-zoom-minus");
  const btnReset = _el("btn-zoom-reset");
  if (btnPlus)  btnPlus.addEventListener("click",  () => { const s = _zoomCommonStep(); _zoomCommonApplyLocal(_currentZoomCommon + s); _zoomCommonSend("delta",  s); });
  if (btnMinus) btnMinus.addEventListener("click", () => { const s = _zoomCommonStep(); _zoomCommonApplyLocal(_currentZoomCommon - s); _zoomCommonSend("delta", -s); });
  if (btnReset) btnReset.addEventListener("click", () => { _zoomCommonApplyLocal(1.0);  _zoomCommonSend("reset", 1.0); });
  // Sincronizza con telemetria (vr_zoom0 dal viewer XR quando connesso)
  if (typeof registerTelemetryHandler === "function") {
    registerTelemetryHandler((m) => {
      if (Number.isFinite(m && m.vr_zoom0)) {
        const v = Number(m.vr_zoom0);
        if (Math.abs(v - _currentZoomCommon) > 0.005) _zoomCommonApplyLocal(v);
      }
    });
  }
  // Sincronizza con i comandi rilanciati dal backend (es. altra dashboard ha cliccato)
  if (typeof registerVrZoomCommandHandler === "function") {
    registerVrZoomCommandHandler((m) => {
      const a = String(m && m.action || "delta");
      const v = Number(m && m.value);
      if (a === "reset") _zoomCommonApplyLocal(1.0);
      else if (a === "set" && Number.isFinite(v)) _zoomCommonApplyLocal(v);
      else if (Number.isFinite(v)) _zoomCommonApplyLocal(_currentZoomCommon + v);
    });
  }
}

function _initDom() {
  _initCalibSliders();
  _initZoomCommonControls();

  const btnStart   = _el("vr-btn-start");
  const btnReset   = _el("vr-btn-reset-calib");
  const btnEnable  = _el("vr-btn-enable");
  const btnStop    = _el("vr-btn-stop");
  const btnVrPose  = _el("vr-btn-vrpose");
  const btnHome    = _el("vr-btn-home");
  const btnPark    = _el("vr-btn-park");
  const btnDemo    = _el("vr-btn-demo");
  const mirrorContainer = _el("vr-design-panel");
  if (mirrorContainer && typeof window.createVRControls === "function") {
    window.createVRControls(mirrorContainer);
  }
  const mirrorBtnStart = _el("btnStart");
  // Pulsanti modalità: display-only, attivabili solo dal visore VR.
  // Rimangono disabled; l'active class viene aggiornata via telemetria (intent_mode).

  if (btnStart)   btnStart.addEventListener("click",   startVideo);
  if (btnReset)   btnReset.addEventListener("click",   resetCalib);

  // Da STOPPED serve SAFE poi ENABLE (stessa sequenza della Home).
  if (btnEnable)  btnEnable.addEventListener("click",  _actionEnable);
  if (btnStop)    btnStop.addEventListener("click",    _actionStop);
  if (btnVrPose)  btnVrPose.addEventListener("click",  _actionVrPose);
  if (btnHome)    btnHome.addEventListener("click",    _actionHome);
  if (btnPark)    btnPark.addEventListener("click",    _actionPark);
  if (btnDemo)    btnDemo.addEventListener("click",    _actionDemo);

  // Mirror panel: solo Start è cliccabile; le modalità sono read-only (visore).
  if (mirrorBtnStart) mirrorBtnStart.addEventListener("click", startVideo);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", _initDom);
} else {
  _initDom();
}

// Avvia connessione WS (condivisa con j5_common.js — idempotente)
connectJ5Dashboard();

export { sendCommand, resetCalib, startVideo };
