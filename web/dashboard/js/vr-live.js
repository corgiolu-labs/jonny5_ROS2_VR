/**
 * vr-live.js — Pagina dashboard "VR Live"
 *
 * - Connessione WS via j5_common.connectJ5Dashboard (telemetria condivisa).
 * - Video cam0 inline (WHEP -> RTCPeerConnection), per accesso diretto a getStats().
 * - Latenza video calcolata dalle WebRTC stats (e2e / RTT / jitter buffer / decode / fps).
 * - Charts (Chart.js):
 *    1) Latenza video nel tempo
 *    2) Traiettoria EE: fk_live_x/y/z nel tempo
 *    3) Smoothing quintico: fk Z raw vs fit polinomio 5° grado (sliding window)
 */

import { connectJ5Dashboard, registerTelemetryHandler, registerOpenHandler, registerVrZoomCommandHandler, sendCommand } from "../../shared/js/j5_common.js";

// =============================================================================
// Helpers
// =============================================================================

const $ = (id) => document.getElementById(id);

function setText(id, val) {
  const el = $(id); if (el) el.textContent = val;
}

function fmtNum(v, d = 0) {
  if (!Number.isFinite(v)) return "--";
  return Number(v).toFixed(d);
}

function setBadge(id, value, level) {
  const el = $(id); if (!el) return;
  el.classList.remove("ok", "warn", "err");
  if (level) el.classList.add(level);
  const valEl = $(id + "-v"); if (valEl) valEl.textContent = value;
}

const LOG_MAX = 30;
const _logEl = $("event-log");
function addLog(msg) {
  if (!_logEl) return;
  const t = new Date();
  const ts = String(t.getHours()).padStart(2, "0") + ":" +
             String(t.getMinutes()).padStart(2, "0") + ":" +
             String(t.getSeconds()).padStart(2, "0");
  const line = document.createElement("div");
  line.innerHTML = '<span class="ev-time">' + ts + '</span>' + msg;
  _logEl.prepend(line);
  while (_logEl.children.length > LOG_MAX) _logEl.removeChild(_logEl.lastChild);
}

// =============================================================================
// Polyfit (least squares) — fit polinomio di grado 5 su array di punti
// Risolve A^T A c = A^T y con eliminazione di Gauss su matrice 6x7.
// Per visualizzazione: input ~30 campioni, deg=5 -> sistema 6x6, robusto.
// =============================================================================

function polyfit(xs, ys, deg) {
  const n = xs.length;
  if (n < deg + 1) return null;
  const m = deg + 1;
  // Matrice A (n x m): A[i][j] = xs[i]^j
  // Sistema normale: (A^T A) c = A^T y -> M (m x m) e b (m)
  const M = Array.from({ length: m }, () => new Float64Array(m));
  const b = new Float64Array(m);
  for (let i = 0; i < n; i++) {
    let xp = 1;
    for (let j = 0; j < m; j++) {
      b[j] += ys[i] * xp;
      let xq = xp;
      for (let k = j; k < m; k++) {
        M[j][k] += xp * Math.pow(xs[i], k);
      }
      xp *= xs[i];
    }
  }
  // Simmetrizza M
  for (let j = 0; j < m; j++) for (let k = 0; k < j; k++) M[j][k] = M[k][j];
  // Gauss elimination
  for (let i = 0; i < m; i++) {
    // pivot
    let max = Math.abs(M[i][i]); let pi = i;
    for (let r = i + 1; r < m; r++) if (Math.abs(M[r][i]) > max) { max = Math.abs(M[r][i]); pi = r; }
    if (max < 1e-12) return null;
    if (pi !== i) { [M[i], M[pi]] = [M[pi], M[i]]; [b[i], b[pi]] = [b[pi], b[i]]; }
    for (let r = i + 1; r < m; r++) {
      const f = M[r][i] / M[i][i];
      for (let k = i; k < m; k++) M[r][k] -= f * M[i][k];
      b[r] -= f * b[i];
    }
  }
  const c = new Float64Array(m);
  for (let i = m - 1; i >= 0; i--) {
    let s = b[i];
    for (let k = i + 1; k < m; k++) s -= M[i][k] * c[k];
    c[i] = s / M[i][i];
  }
  return c; // c[0] + c[1]*x + ... + c[5]*x^5
}

function polyEval(c, x) {
  let s = 0, xp = 1;
  for (let i = 0; i < c.length; i++) { s += c[i] * xp; xp *= x; }
  return s;
}

// =============================================================================
// WHEP cam0 -> <video> + RTCPeerConnection conservato per stats
// =============================================================================

const camVideo = $("cam-video");
const camStatusEl = $("cam-status");
const camErrEl = $("cam-error");
const camResEl = $("cam-res");

let camPC = null;
let camResRaf = null;

function camErr(msg) {
  if (camErrEl) { camErrEl.textContent = msg; camErrEl.classList.add("show"); }
  if (camStatusEl) { camStatusEl.textContent = "errore"; camStatusEl.className = "err"; }
}

function stopWHEP() {
  if (camPC) {
    try { camPC.close(); } catch (_) {}
    camPC = null;
  }
  if (camVideo) {
    try {
      const s = camVideo.srcObject;
      if (s && s.getTracks) s.getTracks().forEach(t => { try { t.stop(); } catch (_) {} });
    } catch (_) {}
    camVideo.srcObject = null;
  }
}

function startWHEP(camPath) {
  // Sicurezza: chiudi eventuale sessione precedente prima di aprirne una nuova.
  stopWHEP();
  const pc = new RTCPeerConnection({ iceServers: [] });
  pc.addTransceiver("video", { direction: "recvonly" });
  camPC = pc;

  let timeoutId;
  const streamPromise = new Promise((resolve, reject) => {
    timeoutId = setTimeout(() => reject(new Error("Timeout WHEP")), 15000);
    pc.ontrack = (ev) => {
      const stream = (ev.streams && ev.streams[0]) ? ev.streams[0]
                   : (ev.track && ev.track.kind === "video") ? new MediaStream([ev.track]) : null;
      if (stream) { clearTimeout(timeoutId); resolve(stream); }
    };
    pc.onconnectionstatechange = () => {
      if (pc.connectionState === "failed") { clearTimeout(timeoutId); reject(new Error("WebRTC failed")); }
    };
  });

  pc.createOffer()
    .then((offer) => pc.setLocalDescription(offer))
    .then(() => fetch(window.location.origin + "/api/webrtc-whep?path=" + encodeURIComponent(camPath), {
      method: "POST",
      headers: { "Content-Type": "application/sdp" },
      body: pc.localDescription.sdp,
    }))
    .then((resp) => {
      if (!resp.ok) return resp.text().then((t) => { throw new Error("WHEP " + resp.status + ": " + (t || "").slice(0, 120)); });
      return resp.text();
    })
    .then((answerSdp) => pc.setRemoteDescription({ type: "answer", sdp: answerSdp }))
    .then(() => streamPromise)
    .then((stream) => {
      camVideo.srcObject = stream;
      if (camErrEl) camErrEl.classList.remove("show");
      if (camStatusEl) { camStatusEl.textContent = "live"; camStatusEl.className = "ok"; }
      addLog("Cam0 connessa");
      // periodicamente leggi risoluzione effettiva
      const updateRes = () => {
        if (camVideo.videoWidth) {
          if (camResEl) camResEl.textContent = camVideo.videoWidth + "x" + camVideo.videoHeight;
        }
      };
      camVideo.addEventListener("loadedmetadata", updateRes);
      camVideo.addEventListener("resize", updateRes);
      return camVideo.play();
    })
    .catch((e) => {
      console.error("[vr-live] WHEP error:", e);
      camErr(e.message || String(e));
      addLog("Cam0 errore: " + (e.message || e));
    });
}

// =============================================================================
// WebRTC stats poller — riempi videoLatency state e badge
// =============================================================================

const videoLatency = {
  e2eMs: null, rttMs: null, jitterBufMs: null, decodeMs: null, fps: null,
};

let _prev = { decodeTotal: 0, framesDecoded: 0, jbDelay: 0, jbEmitted: 0, ts: 0 };

async function pollWebRTCStats() {
  if (!camPC) return;
  try {
    const stats = await camPC.getStats();
    let inboundVideo = null, remoteOutbound = null, candidatePair = null;
    stats.forEach((r) => {
      if (r.type === "inbound-rtp" && r.kind === "video") inboundVideo = r;
      else if (r.type === "remote-outbound-rtp" && r.kind === "video") remoteOutbound = r;
      else if (r.type === "candidate-pair" && r.nominated && r.state === "succeeded") candidatePair = r;
    });

    if (candidatePair && Number.isFinite(candidatePair.currentRoundTripTime)) {
      videoLatency.rttMs = candidatePair.currentRoundTripTime * 1000;
    }

    if (inboundVideo) {
      const tNow = inboundVideo.timestamp || performance.now();
      const fd = inboundVideo.framesDecoded || 0;
      const dt = inboundVideo.totalDecodeTime || 0;
      const jbd = inboundVideo.jitterBufferDelay || 0;
      const jbe = inboundVideo.jitterBufferEmittedCount || 0;

      // Decode time per frame (avg negli ultimi N frame)
      if (_prev.framesDecoded && fd > _prev.framesDecoded) {
        const dFrames = fd - _prev.framesDecoded;
        const dDecode = dt - _prev.decodeTotal;
        videoLatency.decodeMs = (dDecode / dFrames) * 1000;

        const dt_s = (tNow - _prev.ts) / 1000;
        if (dt_s > 0) videoLatency.fps = dFrames / dt_s;
      }

      // Jitter buffer delay corrente
      if (_prev.jbEmitted && jbe > _prev.jbEmitted) {
        const dDelay = jbd - _prev.jbDelay;
        const dEmit = jbe - _prev.jbEmitted;
        videoLatency.jitterBufMs = (dDelay / dEmit) * 1000;
      }

      _prev = { decodeTotal: dt, framesDecoded: fd, jbDelay: jbd, jbEmitted: jbe, ts: tNow };
    }

    // Stima e2e: jitter buffer + decode + RTT/2 (approssimazione classica)
    const jb = Number.isFinite(videoLatency.jitterBufMs) ? videoLatency.jitterBufMs : 0;
    const dec = Number.isFinite(videoLatency.decodeMs) ? videoLatency.decodeMs : 0;
    const rtt2 = Number.isFinite(videoLatency.rttMs) ? videoLatency.rttMs / 2 : 0;
    videoLatency.e2eMs = jb + dec + rtt2;
  } catch (e) {
    // ignore
  }
}

// =============================================================================
// Charts (Chart.js)
// =============================================================================

const COLORS = {
  red:   "#ff5161",
  green: "#4caf50",
  blue:  "#3d9dff",
  cyan:  "#12c2b2",
  amber: "#ff9800",
  pink:  "#e85aad",
  white: "#f6f8fb",
};

// Helper: formatta i tick come 1 decimale (no "288.36000000000005").
const _yTickFmt1 = (v) => (typeof v === "number") ? v.toFixed(1) : String(v);
const _xTickFmt0 = (v) => (typeof v === "number") ? v.toFixed(0) : String(v);

const CHART_OPTS_BASE = {
  responsive: true,
  maintainAspectRatio: false,
  animation: false,
  parsing: false,
  normalized: true,
  interaction: { mode: "nearest", intersect: false },
  plugins: {
    legend: { labels: { color: "#9db1cc", boxWidth: 14, font: { size: 11 } } },
    tooltip: { enabled: false },
  },
  scales: {
    x: {
      type: "linear",
      ticks: { color: "#9db1cc", maxTicksLimit: 5, font: { size: 10 }, callback: _xTickFmt0 },
      grid: { color: "rgba(255,255,255,0.05)" },
      title: { display: true, text: "t (s)", color: "#9db1cc", font: { size: 10 } },
    },
    y: {
      ticks: { color: "#9db1cc", font: { size: 10 }, callback: _yTickFmt1, maxTicksLimit: 6 },
      grid: { color: "rgba(255,255,255,0.05)" },
    },
  },
};

function makeLineDataset(label, color, opts = {}) {
  return {
    label,
    data: [],
    borderColor: color,
    backgroundColor: color + "33",
    borderWidth: opts.borderWidth || 1.5,
    pointRadius: 0,
    tension: opts.tension !== undefined ? opts.tension : 0.25,
    borderDash: opts.dash || undefined,
  };
}

const chLatency = new Chart($("ch-latency"), {
  type: "line",
  data: {
    datasets: [
      makeLineDataset("e2e",      COLORS.amber, { borderWidth: 2 }),
      makeLineDataset("RTT",      COLORS.blue),
      makeLineDataset("jitter buf", COLORS.cyan),
      makeLineDataset("decode",   COLORS.pink),
    ],
  },
  options: { ...CHART_OPTS_BASE, scales: { ...CHART_OPTS_BASE.scales, y: { ...CHART_OPTS_BASE.scales.y, title: { display: true, text: "ms", color: "#9db1cc", font: { size: 10 } } } } },
});

const chTrajectory = new Chart($("ch-trajectory"), {
  type: "line",
  data: {
    datasets: [
      makeLineDataset("X", COLORS.red),
      makeLineDataset("Y", COLORS.green),
      makeLineDataset("Z", COLORS.blue),
    ],
  },
  options: { ...CHART_OPTS_BASE, scales: { ...CHART_OPTS_BASE.scales, y: { ...CHART_OPTS_BASE.scales.y, title: { display: true, text: "mm", color: "#9db1cc", font: { size: 10 } } } } },
});

const chQuintic = new Chart($("ch-quintic"), {
  type: "line",
  data: {
    datasets: [
      makeLineDataset("Z grezzo", COLORS.white, { borderWidth: 1, tension: 0 }),
      makeLineDataset("Z quintic", COLORS.amber, { borderWidth: 2.5, tension: 0 }),
    ],
  },
  options: { ...CHART_OPTS_BASE, scales: { ...CHART_OPTS_BASE.scales, y: { ...CHART_OPTS_BASE.scales.y, title: { display: true, text: "Z (mm)", color: "#9db1cc", font: { size: 10 } } } } },
});

// =============================================================================
// Buffer dati
// =============================================================================

const WINDOW_SEC = 30;     // finestra visibile su tutti i grafici
const QUINTIC_FIT_SAMPLES = 24; // ultimi N campioni usati per il fit di grado 5
const T0 = performance.now() / 1000;

// Ogni buffer: array di {t, v}. t in secondi relativi a T0.
const buf = {
  e2e: [], rtt: [], jb: [], dec: [],
  fkx: [], fky: [], fkz: [],
};

function pushSample(arr, t, v) {
  if (!Number.isFinite(v)) return;
  arr.push({ x: t, y: v });
  // Evict samples più vecchi della finestra
  const cutoff = t - WINDOW_SEC;
  while (arr.length && arr[0].x < cutoff) arr.shift();
}

function nowSec() { return performance.now() / 1000 - T0; }

// Aggiorna tutti i chart usando i buffer correnti.
function refreshCharts() {
  // Latency chart
  chLatency.data.datasets[0].data = buf.e2e;
  chLatency.data.datasets[1].data = buf.rtt;
  chLatency.data.datasets[2].data = buf.jb;
  chLatency.data.datasets[3].data = buf.dec;

  // Trajectory chart
  chTrajectory.data.datasets[0].data = buf.fkx;
  chTrajectory.data.datasets[1].data = buf.fky;
  chTrajectory.data.datasets[2].data = buf.fkz;

  // Quintic: serie Z grezza + fit di grado 5 su ultimi QUINTIC_FIT_SAMPLES campioni.
  chQuintic.data.datasets[0].data = buf.fkz;
  if (buf.fkz.length >= 6) {
    const tail = buf.fkz.slice(-QUINTIC_FIT_SAMPLES);
    const xs = tail.map((p) => p.x);
    const ys = tail.map((p) => p.y);
    const yMin = Math.min(...ys);
    const yMax = Math.max(...ys);
    const yRange = yMax - yMin;
    // Skip fit se i dati sono praticamente costanti (robot fermo): polyfit
    // ill-conditioned -> spike numerici. Mostra una linea piatta sul valore medio.
    if (yRange < 0.5) {
      const yMean = ys.reduce((a, b) => a + b, 0) / ys.length;
      chQuintic.data.datasets[1].data = [
        { x: xs[0],                 y: yMean },
        { x: xs[xs.length - 1],     y: yMean },
      ];
    } else {
      // Centra l'asse x per migliore condizionamento numerico.
      const xMean = xs.reduce((a, b) => a + b, 0) / xs.length;
      const xsC = xs.map((x) => x - xMean);
      const c = polyfit(xsC, ys, Math.min(5, ys.length - 1));
      if (c) {
        // Plotta solo la regione INTERIORE (inset 8% per lato) per evitare
        // l'esplosione del polinomio ai bordi (Runge phenomenon).
        const xMin = xs[0], xMax = xs[xs.length - 1];
        const span = xMax - xMin;
        const xLo = xMin + span * 0.08;
        const xHi = xMax - span * 0.08;
        const N_PLOT = 60;
        // Clamp dei valori del fit a yMin/yMax ± 20% margine: scarta spike.
        const yMargin = yRange * 0.2;
        const yLo = yMin - yMargin;
        const yHi = yMax + yMargin;
        const fit = [];
        for (let i = 0; i < N_PLOT; i++) {
          const x = xLo + ((xHi - xLo) * i) / (N_PLOT - 1);
          const yEval = polyEval(c, x - xMean);
          if (Number.isFinite(yEval)) {
            fit.push({ x, y: Math.max(yLo, Math.min(yHi, yEval)) });
          }
        }
        chQuintic.data.datasets[1].data = fit;
      } else {
        chQuintic.data.datasets[1].data = [];
      }
    }
  } else {
    chQuintic.data.datasets[1].data = [];
  }

  chLatency.update("none");
  chTrajectory.update("none");
  chQuintic.update("none");
}

// =============================================================================
// 3D EE Tracker — proiezione isometrica della posizione end-effector + scia
// =============================================================================

const EE_TRAIL_MAX = 80;
const eeTrail = []; // {x, y, z}
const eeCanvas = $("ee-tracker-canvas");
const eeCtx = eeCanvas ? eeCanvas.getContext("2d") : null;

// Workspace approssimativo (mm). Convenzione robotica: Z up, piano XY = orizzontale.
// X in [0..400] (forward), Y in [-200..200] (laterale), Z in [0..400] (alto).
const EE_VIEW = {
  cx: 200, cy: 0, cz: 200,   // centro logico (mm) — middle of workspace
  scale: 0.32,                // mm -> px
};

function eeProject(x, y, z, w, h) {
  // Isometric con Z verticale (up). Piano XY orizzontale.
  //   X (forward) -> screen upper-right
  //   Y (lateral) -> screen upper-left
  //   Z (up)      -> screen up
  const X = x - EE_VIEW.cx;
  const Y = y - EE_VIEW.cy;
  const Z = z - EE_VIEW.cz;
  const px = w * 0.5  + (X - Y) * EE_VIEW.scale * 0.866;     // cos(30°)
  const py = h * 0.62 + (X + Y) * EE_VIEW.scale * 0.5        // sin(30°)
                       - Z * EE_VIEW.scale;
  return [px, py];
}

function drawEETracker() {
  if (!eeCtx || !eeCanvas) return;
  const w = eeCanvas.width, h = eeCanvas.height;
  eeCtx.clearRect(0, 0, w, h);

  // Origin assi (base robot)
  const [ox, oy] = eeProject(0, 0, 0, w, h);
  // Griglia di base = piano XY a Z=0 (floor): rettangolo iso 400x400 (X 0..400, Y -200..200)
  eeCtx.strokeStyle = "rgba(255,255,255,0.10)";
  eeCtx.lineWidth = 1;
  const gPts = [
    [0,   -200, 0],
    [400, -200, 0],
    [400,  200, 0],
    [0,    200, 0],
    [0,   -200, 0],
  ].map(p => eeProject(p[0], p[1], p[2], w, h));
  eeCtx.beginPath();
  eeCtx.moveTo(gPts[0][0], gPts[0][1]);
  for (let i = 1; i < gPts.length; i++) eeCtx.lineTo(gPts[i][0], gPts[i][1]);
  eeCtx.stroke();
  // Linee griglia interna ogni 100mm sul piano XY (Z=0)
  eeCtx.strokeStyle = "rgba(255,255,255,0.05)";
  // Linee parallele a Y, a X variabile (X = 100, 200, 300)
  for (let gx = 100; gx < 400; gx += 100) {
    const p1 = eeProject(gx, -200, 0, w, h), p2 = eeProject(gx, 200, 0, w, h);
    eeCtx.beginPath(); eeCtx.moveTo(p1[0], p1[1]); eeCtx.lineTo(p2[0], p2[1]); eeCtx.stroke();
  }
  // Linee parallele a X, a Y variabile (Y = -100, 0, 100)
  for (let gy = -100; gy <= 100; gy += 100) {
    const p1 = eeProject(0, gy, 0, w, h), p2 = eeProject(400, gy, 0, w, h);
    eeCtx.beginPath(); eeCtx.moveTo(p1[0], p1[1]); eeCtx.lineTo(p2[0], p2[1]); eeCtx.stroke();
  }
  // Assi colorati (X rosso forward, Y verde laterale, Z blu verticale)
  eeCtx.lineWidth = 1.5;
  eeCtx.font = "bold 10px monospace";
  let p;
  eeCtx.strokeStyle = COLORS.red;
  p = eeProject(150, 0, 0, w, h);
  eeCtx.beginPath(); eeCtx.moveTo(ox, oy); eeCtx.lineTo(p[0], p[1]); eeCtx.stroke();
  eeCtx.fillStyle = COLORS.red;
  eeCtx.fillText("X", p[0] + 3, p[1] + 8);
  eeCtx.strokeStyle = COLORS.green;
  p = eeProject(0, 150, 0, w, h);
  eeCtx.beginPath(); eeCtx.moveTo(ox, oy); eeCtx.lineTo(p[0], p[1]); eeCtx.stroke();
  eeCtx.fillStyle = COLORS.green;
  eeCtx.fillText("Y", p[0] - 12, p[1] + 8);
  eeCtx.strokeStyle = COLORS.blue;
  p = eeProject(0, 0, 150, w, h);
  eeCtx.beginPath(); eeCtx.moveTo(ox, oy); eeCtx.lineTo(p[0], p[1]); eeCtx.stroke();
  eeCtx.fillStyle = COLORS.blue;
  eeCtx.fillText("Z", p[0] - 4, p[1] - 4);

  // Scia EE
  if (eeTrail.length > 1) {
    for (let i = 1; i < eeTrail.length; i++) {
      const a = i / eeTrail.length;
      eeCtx.strokeStyle = `rgba(255, 200, 100, ${a * 0.85})`;
      eeCtx.lineWidth = 1.5;
      const [x0, y0] = eeProject(eeTrail[i-1].x, eeTrail[i-1].y, eeTrail[i-1].z, w, h);
      const [x1, y1] = eeProject(eeTrail[i].x,   eeTrail[i].y,   eeTrail[i].z,   w, h);
      eeCtx.beginPath();
      eeCtx.moveTo(x0, y0);
      eeCtx.lineTo(x1, y1);
      eeCtx.stroke();
    }
  }
  // Punto EE corrente
  if (eeTrail.length > 0) {
    const last = eeTrail[eeTrail.length - 1];
    const [px, py] = eeProject(last.x, last.y, last.z, w, h);
    eeCtx.fillStyle = "rgba(255, 200, 100, 1)";
    eeCtx.beginPath();
    eeCtx.arc(px, py, 5, 0, 2 * Math.PI);
    eeCtx.fill();
    eeCtx.lineWidth = 1.5;
    eeCtx.strokeStyle = "rgba(255, 255, 255, 0.95)";
    eeCtx.stroke();
  }
}

function pushEETrail(x, y, z) {
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;
  // Skip duplicate consecutivi (robot fermo): non sporchiamo la scia.
  const last = eeTrail[eeTrail.length - 1];
  if (last && Math.abs(last.x - x) < 0.05 && Math.abs(last.y - y) < 0.05 && Math.abs(last.z - z) < 0.05) return;
  eeTrail.push({ x, y, z });
  while (eeTrail.length > EE_TRAIL_MAX) eeTrail.shift();
}

// =============================================================================
// Telemetry handler — aggiorna pannelli + buffer
// =============================================================================

function modeName(mode) {
  switch (Number(mode)) {
    case 0: return "CALIB";
    case 1: return "POSE";
    case 2: return "MANUAL";
    case 3: return "HEAD";
    case 4: return "HYBRID";
    case 5: return "ASSIST";
    default: return "—";
  }
}

let lastLogState = null;
function onTelemetry(msg) {
  // ── EE (FK live)
  setText("fk-x", fmtNum(msg.fk_live_x_mm, 1));
  setText("fk-y", fmtNum(msg.fk_live_y_mm, 1));
  setText("fk-z", fmtNum(msg.fk_live_z_mm, 1));
  setText("fk-roll",  fmtNum(msg.fk_live_roll, 1));
  setText("fk-pitch", fmtNum(msg.fk_live_pitch, 1));
  setText("fk-yaw",   fmtNum(msg.fk_live_yaw, 1));

  // ── Joint state
  setText("j-B", fmtNum(msg.servo_deg_B, 0));
  setText("j-S", fmtNum(msg.servo_deg_S, 0));
  setText("j-G", fmtNum(msg.servo_deg_G, 0));
  setText("j-Y", fmtNum(msg.servo_deg_Y, 0));
  setText("j-P", fmtNum(msg.servo_deg_P, 0));
  setText("j-R", fmtNum(msg.servo_deg_R, 0));

  // ── Badge top
  const vrActive = !!msg.vr_active;
  setBadge("bd-vr", vrActive ? "ATTIVO" : "—", vrActive ? "ok" : "");
  const mn = modeName(msg.teleop_mode);
  setBadge("bd-mode", mn, mn === "HYBRID" || mn === "HEAD" || mn === "ASSIST" ? "ok" : "");
  const rs = msg.robot_state || "—";
  setBadge("bd-state", rs, rs === "IDLE" ? "ok" : (rs ? "warn" : ""));

  // ── Log eventi: change-detect su robot_state
  const ls = String(rs);
  if (lastLogState !== null && lastLogState !== ls) {
    addLog("Stato robot: " + lastLogState + " → " + ls);
  }
  lastLogState = ls;

  // ── Buffer per chart traiettoria
  const t = nowSec();
  pushSample(buf.fkx, t, msg.fk_live_x_mm);
  pushSample(buf.fky, t, msg.fk_live_y_mm);
  pushSample(buf.fkz, t, msg.fk_live_z_mm);

  // ── Scia 3D EE per il tracker isometrico
  pushEETrail(Number(msg.fk_live_x_mm), Number(msg.fk_live_y_mm), Number(msg.fk_live_z_mm));

  // ── Pulse "Modalita" badge quando l'operatore sta inviando intent attivo
  // (intent_age_ms < 500ms). Indica vita del sistema durante la demo.
  const modeBadge = $("bd-mode");
  if (modeBadge) {
    const intentAge = Number(msg.intent_age_ms);
    if (Number.isFinite(intentAge) && intentAge < 500) {
      modeBadge.classList.add("pulse");
    } else {
      modeBadge.classList.remove("pulse");
    }
  }

  // ── Zoom VR (cam0): aggiorna badge + overlay + CSS scale sul video.
  // Il viewer XR invia vr_zoom_state via WS, backend lo include come vr_zoom0/1.
  if (Number.isFinite(msg.vr_zoom0)) {
    const z = Math.max(1.0, Math.min(6.0, Number(msg.vr_zoom0)));
    const zStr = z.toFixed(2) + "×";
    setText("cam-zoom", zStr);
    const lvl = (Math.abs(z - 1.0) < 0.01) ? "" : "ok";
    setBadge("bd-zoom", zStr, lvl);
    if (camVideo) camVideo.style.setProperty("--vr-zoom", String(z));
  }
}

// =============================================================================
// Loop UI: aggiorna stats + badge latenza + chart ogni 250ms
// =============================================================================

async function tick() {
  await pollWebRTCStats();
  const t = nowSec();
  pushSample(buf.e2e, t, videoLatency.e2eMs);
  pushSample(buf.rtt, t, videoLatency.rttMs);
  pushSample(buf.jb,  t, videoLatency.jitterBufMs);
  pushSample(buf.dec, t, videoLatency.decodeMs);

  // Overlay video
  setText("ovr-e2e", Number.isFinite(videoLatency.e2eMs) ? Math.round(videoLatency.e2eMs) + " ms" : "-- ms");
  setText("ovr-rtt", Number.isFinite(videoLatency.rttMs) ? Math.round(videoLatency.rttMs) + " ms" : "-- ms");
  setText("ovr-jitter", Number.isFinite(videoLatency.jitterBufMs) ? Math.round(videoLatency.jitterBufMs) + " ms" : "-- ms");

  // Badge top
  if (Number.isFinite(videoLatency.e2eMs)) {
    const v = videoLatency.e2eMs;
    const lvl = v < 80 ? "ok" : v < 200 ? "warn" : "err";
    setBadge("bd-lat", Math.round(v) + " ms", lvl);
  }
  if (Number.isFinite(videoLatency.fps)) {
    setBadge("bd-fps", videoLatency.fps.toFixed(0), videoLatency.fps > 20 ? "ok" : "warn");
  }

  refreshCharts();
  drawEETracker();
}

// =============================================================================
// Bootstrap
// =============================================================================

document.addEventListener("DOMContentLoaded", () => {
  // WS
  setBadge("bd-ws", "connecting...", "warn");
  registerOpenHandler(() => {
    setBadge("bd-ws", "ONLINE", "ok");
    addLog("WebSocket aperto");
  });
  registerTelemetryHandler(onTelemetry);
  const _ws = connectJ5Dashboard();

  // Listener per "cameras_refocus_triggered" (broadcast dal viewer XR dopo
  // POST /api/refocus-cameras): MediaMTX si sta riavviando, quindi chiudi
  // la WHEP corrente, attendi ~3s, riapri cam0.
  if (_ws) {
    _ws.addEventListener("message", (ev) => {
      try {
        const m = JSON.parse(ev.data);
        if (m && m.type === "cameras_refocus_triggered") {
          addLog("Refocus camere: WHEP reconnect tra 3s");
          if (camStatusEl) { camStatusEl.textContent = "refocus..."; camStatusEl.className = "warn"; }
          stopWHEP();
          setTimeout(() => {
            try { startWHEP("cam0"); } catch (_) {}
          }, 3000);
        }
      } catch (_) { /* non-JSON or unrelated */ }
    });
  }

  // Update WS pill on close: j5_common gestisce pill #diag-ws ma noi qui
  // usiamo il proprio badge -> osserviamo onclose tramite event log fallback
  window.addEventListener("offline", () => setBadge("bd-ws", "OFFLINE", "err"));

  // WHEP cam0
  startWHEP("cam0");

  // Loop UI
  setInterval(tick, 250);

  // Inizializza controlli Zoom comune (pulsanti +/-/reset)
  _initZoomCommonControls();
  // Inizializza selettore profilo video MediaMTX
  _initVideoProfileSelector();

  addLog("Pagina VR Live inizializzata");
});

// === Zoom comune: pulsanti +/-/reset, indipendenti dal viewer XR ============
// La dashboard invia "vr_zoom_command" al backend. Se il viewer XR è
// connesso lo applica via changeZoom() e re-emette vr_zoom_state. Se non
// è connesso, le altre dashboard ricevono il broadcast del comando e
// aggiornano lo zoom CSS local-side.
const ZOOM_COMMON_MIN = 1.0;
const ZOOM_COMMON_MAX = 6.0;
let _currentZoomCommon = 1.0;

function _zoomCommonApplyLocal(z) {
  const v = Math.max(ZOOM_COMMON_MIN, Math.min(ZOOM_COMMON_MAX, Number(z) || 1.0));
  _currentZoomCommon = v;
  const cur = $("zoom-common-val");
  if (cur) cur.textContent = v.toFixed(2) + "×";
  const camV = $("cam-video");
  if (camV) camV.style.setProperty("--vr-zoom", String(v));
}

function _zoomCommonStep() {
  const raw = Number(($("zoom-common-step") || {}).value);
  return (Number.isFinite(raw) && raw > 0) ? raw : 0.2;
}

function _zoomCommonSend(action, value) {
  try { sendCommand("vr_zoom_command", { action: action, value: Number(value) || 0 }); } catch (_) {}
}

function _initZoomCommonControls() {
  const bp = $("btn-zoom-plus");
  const bm = $("btn-zoom-minus");
  const br = $("btn-zoom-reset");
  if (bp) bp.addEventListener("click", () => { const s = _zoomCommonStep(); _zoomCommonApplyLocal(_currentZoomCommon + s); _zoomCommonSend("delta",  s); });
  if (bm) bm.addEventListener("click", () => { const s = _zoomCommonStep(); _zoomCommonApplyLocal(_currentZoomCommon - s); _zoomCommonSend("delta", -s); });
  if (br) br.addEventListener("click", () => { _zoomCommonApplyLocal(1.0); _zoomCommonSend("reset", 1.0); });

  // Sync con telemetria (vr_zoom0 dal viewer XR quando connesso)
  try {
    registerTelemetryHandler((m) => {
      if (Number.isFinite(m && m.vr_zoom0)) {
        const v = Number(m.vr_zoom0);
        if (Math.abs(v - _currentZoomCommon) > 0.005) _zoomCommonApplyLocal(v);
      }
    });
  } catch (_) {}

  // Sync con i comandi rilanciati dal backend (altra dashboard ha cliccato)
  try {
    registerVrZoomCommandHandler((m) => {
      const a = String(m && m.action || "delta");
      const v = Number(m && m.value);
      if (a === "reset") _zoomCommonApplyLocal(1.0);
      else if (a === "set" && Number.isFinite(v)) _zoomCommonApplyLocal(v);
      else if (Number.isFinite(v)) _zoomCommonApplyLocal(_currentZoomCommon + v);
    });
  } catch (_) {}
}

// === Selettore profilo video MediaMTX ========================================
// La dashboard invia "set_video_profile". Il backend riscrive video_pipeline.yaml,
// riavvia jonny5-mediamtx e broadcast cameras_refocus_triggered (gia' gestito
// piu' sopra per riconnettere WHEP). vr-live espone anche il profilo MAXRES.
function _initVideoProfileSelector() {
  const sel = $("video-profile-select");
  const btn = $("btn-video-profile-apply");
  const status = $("video-profile-status");
  if (!sel || !btn) return;

  btn.addEventListener("click", () => {
    const profile = String(sel.value || "lowlatency");
    if (status) status.textContent = "applicazione " + profile + "...";
    btn.disabled = true;
    addLog("Profilo video richiesto: " + profile);
    try { sendCommand("set_video_profile", { profile: profile }); } catch (_) {}
    // La conferma operativa arriva come cameras_refocus_triggered (gia' gestito
    // sopra per la riconnessione WHEP). Re-abilito il pulsante dopo 7 s.
    setTimeout(() => {
      btn.disabled = false;
      if (status) status.textContent = "attivo: " + profile;
    }, 7000);
  });
}
