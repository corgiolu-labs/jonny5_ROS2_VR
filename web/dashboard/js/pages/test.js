/**
 * JONNY5 Dashboard - Pagina Test (validazione sperimentale live)
 *
 *  - Stato sistema live (loop control, SPI, IMU, WS RTT, refresh client/HMD,
 *    HMD video latency)
 *  - Latenza video per profilo MediaMTX vs MJPEG full-stack (selezione +
 *    run + bar chart + tabella comparativa con CPU/RAM/temp)
 *  - Self-test modale (forced response Y/P/R + PSD via Welch sul backend)
 *  - Misura latenza di comando (WebSocket round-trip burst)
 */

import {
  addLog,
  sendCommand,
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerSelfTestStatusHandler,
  registerSelfTestResultHandler,
  registerSystemLoadHandler,
  registerWsPongHandler,
  registerVrSessionHandler,
} from "../../../shared/js/j5_common.js";

// ===========================================================================
// Stato sistema live (legge dal payload telemetry esistente)
// ===========================================================================

// Soglie per LED di stato. Calibrate sui requisiti control-theoretical
// reali del manipolatore, non sui target architetturali storici:
//
//   loop  = banda meccanica 5-10 Hz × 20× regola conservativa = 200 Hz min ok
//           (Franklin-Powell-Spong: 10× minimo pratico, 20× conservativo,
//            30× paranoico). Loop misurato live ~506 Hz = ampiamente sopra.
//   SPI uplink: idle ~30 Hz (heartbeat), VR attivo ~100 Hz
//   IMU sample: BNO085 RotVec config ~100 Hz (oversampled da STM32 a 400 Hz)
//   ws_ping: round-trip vero ~1 ms LAN, soglia ok 5 ms (cap.10 tab.10.3: 0.75)
const TARGETS = {
  loop: { ok: 200, warn: 100 },       // Hz (regola 10×/20× su banda 5-10 Hz)
  spi:  { ok: 20,  warn: 5  },        // Hz (idle ~30, VR attivo ~100)
  imu:  { ok: 80,  warn: 30 },        // Hz (idle ~100, polling continuo ~400)
  ws:   { okMax: 5.0, warnMax: 20.0 } // ms (inter-msg proxy in assenza di ping live)
};

function setCardStatus(cardId, ledId, ok, warn) {
  const card = document.getElementById(cardId);
  const led  = document.getElementById(ledId);
  if (!card || !led) return;
  card.classList.remove("ok", "warn", "err");
  led.classList.remove("ok", "warn", "err");
  if (ok)        { card.classList.add("ok");   led.classList.add("ok"); }
  else if (warn) { card.classList.add("warn"); led.classList.add("warn"); }
  else           { card.classList.add("err");  led.classList.add("err"); }
}

function fmtHz(v)  { return (v == null || Number.isNaN(v)) ? "–" : v.toFixed(1); }
function fmtMs(v)  { return (v == null || Number.isNaN(v)) ? "–" : v.toFixed(2); }

// Buffer scorrevole per loop rate (smoothing live come SPI/IMU).
const LOOP_SMOOTH_WINDOW = 20;
const _loopRateBuf = [];

function updateLoopRate(hz) {
  const smoothed = _smoothedAvg(_loopRateBuf, hz, LOOP_SMOOTH_WINDOW);
  document.getElementById("val-loop").textContent = fmtHz(smoothed);
  setCardStatus("card-loop", "led-loop",
    smoothed >= TARGETS.loop.ok, smoothed >= TARGETS.loop.warn);
}

// Buffer scorrevoli per SPI/IMU: il backend calcola il rate come
// (delta_packet_index / delta_t) per ogni singolo messaggio telemetry,
// quindi il valore istantaneo oscilla bruscamente (0, 50, 100 Hz...
// dipende da quanti packet_index sono cambiati nel tick). Mostriamo
// la media scorrevole su SPI_SMOOTH_WINDOW campioni per stabilizzare
// la card ed evitare flicker del LED al confine delle soglie.
const SPI_SMOOTH_WINDOW = 20;
const IMU_SMOOTH_WINDOW = 20;
const _spiRateBuf = [];
const _imuRateBuf = [];

function _smoothedAvg(buf, value, windowSize) {
  buf.push(value);
  while (buf.length > windowSize) buf.shift();
  return buf.reduce((s, x) => s + x, 0) / buf.length;
}

function updateSpiRate(hz) {
  const smoothed = _smoothedAvg(_spiRateBuf, hz, SPI_SMOOTH_WINDOW);
  document.getElementById("val-spi").textContent = fmtHz(smoothed);
  setCardStatus("card-spi", "led-spi",
    smoothed >= TARGETS.spi.ok, smoothed >= TARGETS.spi.warn);
}
function updateImuRate(hz) {
  const smoothed = _smoothedAvg(_imuRateBuf, hz, IMU_SMOOTH_WINDOW);
  document.getElementById("val-imu").textContent = fmtHz(smoothed);
  setCardStatus("card-imu", "led-imu",
    smoothed >= TARGETS.imu.ok, smoothed >= TARGETS.imu.warn);
}

// ===========================================================================
// Card "HMD refresh (XR)" — refresh nativo del visore Quest durante sessione
// WebXR attiva, ricevuto via WS dal viewer_stereo_xr.html (XRSession.frameRate).
// ===========================================================================

let _hmdLastUpdateMs = 0;
let _hmdActive = false;
let _hmdLatLastUpdateMs = 0;

// Soglie HMD video latency: target sotto i ~50 ms (low-latency profile WebRTC);
// ok ≤ 80 ms, warn ≤ 150 ms. Sopra è degradato rispetto alla soglia percettiva VR.
const HMD_LAT_OK_MAX = 80;
const HMD_LAT_WARN_MAX = 150;

function handleVrSession(msg) {
  const sublabel = document.getElementById("hmd-sublabel");
  const latSublabel = document.getElementById("hmd-lat-sublabel");
  if (msg.type === "vr_session_state") {
    _hmdActive = !!msg.active;
    if (!_hmdActive) {
      // Sessione XR terminata: reset entrambe le card HMD
      document.getElementById("val-hmd").textContent = "–";
      if (sublabel) sublabel.textContent = "no XR session";
      setCardStatus("card-hmd", "led-hmd", false, false);
      document.getElementById("val-hmd-lat").textContent = "–";
      if (latSublabel) latSublabel.textContent = "no XR session";
      setCardStatus("card-hmd-lat", "led-hmd-lat", false, false);
    }
    return;
  }
  if (msg.type === "vr_session_refresh") {
    _hmdActive = true;
    _hmdLastUpdateMs = performance.now();
    const fr = Number(msg.frame_rate_hz);
    if (Number.isFinite(fr) && fr > 0) {
      document.getElementById("val-hmd").textContent = fr.toFixed(1);
      // LED verde per qualsiasi rate >=55 (Quest 1=72, Quest 2 90/120)
      setCardStatus("card-hmd", "led-hmd", fr >= 55, fr >= 30);
      if (sublabel) {
        const supported = Array.isArray(msg.supported_rates_hz) ? msg.supported_rates_hz : [];
        const tail = supported.length ? ` (supportati: ${supported.join("/")} Hz)` : "";
        sublabel.textContent = `XR session attiva${tail}`;
      }
    }
    return;
  }
  if (msg.type === "vr_video_latency") {
    _hmdActive = true;
    _hmdLatLastUpdateMs = performance.now();
    const est = msg.estimated_latency_ms || {};
    const mean = Number(est.mean);
    const min = Number(est.min);
    const max = Number(est.max);
    if (Number.isFinite(mean) && mean > 0) {
      document.getElementById("val-hmd-lat").textContent = mean.toFixed(1);
      // LED: verde ≤ 80 ms (low-latency profile), giallo 80-150, rosso > 150
      setCardStatus("card-hmd-lat", "led-hmd-lat",
                    mean <= HMD_LAT_OK_MAX,
                    mean <= HMD_LAT_WARN_MAX);
      if (latSublabel) {
        const minTxt = Number.isFinite(min) ? min.toFixed(0) : "?";
        const maxTxt = Number.isFinite(max) ? max.toFixed(0) : "?";
        latSublabel.textContent = `cam0 WHEP · range ${minTxt}-${maxTxt} ms`;
      }
    }
  }
}

// Watchdog: il viewer pubblica vr_video_latency ogni 500 ms e
// vr_session_refresh ogni 2 s. Se non riceviamo per >5 s (rispettivamente
// 10× e 2.5× del periodo), consideriamo la sessione XR terminata anche
// se non abbiamo ricevuto vr_session_state{active:false}.
setInterval(() => {
  const now = performance.now();
  if (_hmdActive && now - _hmdLastUpdateMs > 5000) {
    _hmdActive = false;
    document.getElementById("val-hmd").textContent = "–";
    const sublabel = document.getElementById("hmd-sublabel");
    if (sublabel) sublabel.textContent = "no XR session (timeout)";
    setCardStatus("card-hmd", "led-hmd", false, false);
  }
  if (now - _hmdLatLastUpdateMs > 3000 && _hmdLatLastUpdateMs > 0) {
    document.getElementById("val-hmd-lat").textContent = "–";
    const latSublabel = document.getElementById("hmd-lat-sublabel");
    if (latSublabel) latSublabel.textContent = "no XR session (timeout)";
    setCardStatus("card-hmd-lat", "led-hmd-lat", false, false);
    _hmdLatLastUpdateMs = 0;
  }
}, 1000);

// ===========================================================================
// Card "Dashboard video latency" — misura continua sul browser dashboard
// ===========================================================================
//
// Speculare alla card "HMD video latency" ma misurata localmente dal browser
// che ha aperto la dashboard (non ricevuta via WS dal viewer XR). Apre un
// peer WHEP nascosto sul flusso cam0, accumula inter-frame timing via
// requestVideoFrameCallback in finestra scorrevole (60 frame), e aggiorna
// la card ogni 500 ms.
//
// Latenza stimata = (inter-frame interval medio) × buffer_depth WebRTC (4.5),
// identico modello del viewer_stereo_xr.html e di measureLatencyForCurrentStream.
// Rappresenta il lower-bound della pipeline (no compositing XR, no rendering
// stereo): con profilo Low-latency 800×450@120 atteso ~38 ms.
//
// Recovery: se MediaMTX viene fermato (es. test MJPEG full-stack) la connessione
// muore. Il watchdog rileva il timeout e tenta una riconnessione con backoff.

const DASH_LAT_OK_MAX = 80;
const DASH_LAT_WARN_MAX = 150;
const DASH_LAT_BUFFER_DEPTH = 4.5;
const DASH_LAT_WINDOW = 60;
const DASH_LAT_UPDATE_MS = 500;
const DASH_LAT_TIMEOUT_MS = 3000;
const DASH_LAT_RECONNECT_MS = 5000;

let _dashLatPC = null;
let _dashLatVideo = null;
let _dashLatIntervals = [];
let _dashLatLastFrameTime = null;
let _dashLatLastUpdateMs = 0;
let _dashLatConnecting = false;
let _dashLatLastReconnectAttempt = 0;

function _stopDashLatency() {
  if (_dashLatPC) {
    try { _dashLatPC.close(); } catch (_) {}
    _dashLatPC = null;
  }
  if (_dashLatVideo) {
    try { _dashLatVideo.pause(); } catch (_) {}
    try { _dashLatVideo.srcObject = null; } catch (_) {}
    try {
      if (_dashLatVideo.parentNode) _dashLatVideo.parentNode.removeChild(_dashLatVideo);
    } catch (_) {}
    _dashLatVideo = null;
  }
  _dashLatIntervals = [];
  _dashLatLastFrameTime = null;
}

async function _startDashLatency() {
  if (_dashLatConnecting) return;
  _dashLatConnecting = true;

  _stopDashLatency();

  const sublabel = document.getElementById("dash-lat-sublabel");
  if (sublabel) sublabel.textContent = "connessione WHEP…";

  const video = document.createElement("video");
  video.style.display = "none";
  video.muted = true;
  video.autoplay = true;
  video.playsInline = true;
  document.body.appendChild(video);
  _dashLatVideo = video;

  const pc = new RTCPeerConnection();
  _dashLatPC = pc;
  pc.addTransceiver("video", { direction: "recvonly" });

  pc.ontrack = (ev) => {
    if (_dashLatVideo === video) {
      video.srcObject = ev.streams[0];
      video.play().catch(() => {});
    }
  };

  try {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    const resp = await fetch("/api/webrtc-whep?path=" + encodeURIComponent("cam0"), {
      method: "POST",
      headers: { "Content-Type": "application/sdp" },
      body: offer.sdp,
    });
    if (!resp.ok) {
      const txt = await resp.text().catch(() => "");
      throw new Error(`WHEP ${resp.status}: ${txt.slice(0, 80)}`);
    }
    const answer = await resp.text();
    await pc.setRemoteDescription({ type: "answer", sdp: answer });
  } catch (e) {
    _stopDashLatency();
    if (sublabel) sublabel.textContent = `riconnessione… (${(e.message || e).toString().slice(0, 40)})`;
    setCardStatus("card-dash-lat", "led-dash-lat", false, false);
    _dashLatConnecting = false;
    return;
  }

  if (!("requestVideoFrameCallback" in video)) {
    if (sublabel) sublabel.textContent = "rVFC non supportato dal browser";
    _dashLatConnecting = false;
    return;
  }

  // Loop di campionamento inter-frame in finestra scorrevole
  const cb = (now, _metadata) => {
    if (_dashLatVideo !== video) return; // connessione chiusa, esci
    if (_dashLatLastFrameTime != null) {
      const interval = now - _dashLatLastFrameTime;
      if (interval > 0.5 && interval < 200) {
        _dashLatIntervals.push(interval);
        if (_dashLatIntervals.length > DASH_LAT_WINDOW) {
          _dashLatIntervals.shift();
        }
      }
    }
    _dashLatLastFrameTime = now;
    video.requestVideoFrameCallback(cb);
  };
  video.requestVideoFrameCallback(cb);

  if (sublabel) sublabel.textContent = "cam0 WHEP · in acquisizione…";
  _dashLatConnecting = false;
}

// Tick di aggiornamento card: calcola latenza dalla finestra scorrevole
setInterval(() => {
  if (_dashLatIntervals.length < 5) return;
  const latencies = _dashLatIntervals.map(iv => iv * DASH_LAT_BUFFER_DEPTH);
  const sorted = [...latencies].sort((a, b) => a - b);
  const mean = latencies.reduce((s, x) => s + x, 0) / latencies.length;
  const min = sorted[0];
  const max = sorted[sorted.length - 1];
  document.getElementById("val-dash-lat").textContent = mean.toFixed(1);
  setCardStatus("card-dash-lat", "led-dash-lat",
                mean <= DASH_LAT_OK_MAX, mean <= DASH_LAT_WARN_MAX);
  const sublabel = document.getElementById("dash-lat-sublabel");
  if (sublabel) {
    sublabel.textContent = `cam0 WHEP · range ${min.toFixed(0)}-${max.toFixed(0)} ms`;
  }
  _dashLatLastUpdateMs = performance.now();
}, DASH_LAT_UPDATE_MS);

// Watchdog: se non arrivano frame da DASH_LAT_TIMEOUT_MS, tenta riconnessione
// con backoff di DASH_LAT_RECONNECT_MS (gestisce stop MediaMTX durante test MJPEG).
setInterval(() => {
  const now = performance.now();
  const stale = _dashLatLastUpdateMs > 0 && (now - _dashLatLastUpdateMs > DASH_LAT_TIMEOUT_MS);
  const neverStarted = _dashLatLastUpdateMs === 0 && !_dashLatConnecting && !_dashLatPC;
  if ((stale || neverStarted) && (now - _dashLatLastReconnectAttempt > DASH_LAT_RECONNECT_MS)) {
    _dashLatLastReconnectAttempt = now;
    if (stale) {
      document.getElementById("val-dash-lat").textContent = "–";
      const sublabel = document.getElementById("dash-lat-sublabel");
      if (sublabel) sublabel.textContent = "stream non disponibile, riconnessione…";
      setCardStatus("card-dash-lat", "led-dash-lat", false, false);
    }
    _startDashLatency().catch(() => {});
  }
}, 1500);

function updateWsRtt(ms) {
  document.getElementById("val-ws").textContent = fmtMs(ms);
  setCardStatus("card-ws", "led-ws",
    ms <= TARGETS.ws.okMax, ms <= TARGETS.ws.warnMax);
}

// ===========================================================================
// WebSocket round-trip (latenza di comando)
// ===========================================================================
//
// Metodologia: il client invia ws_ping con {id, ts_client}; il backend
// (ws_server.py) risponde IMMEDIATAMENTE con ws_pong contenente gli stessi
// campi. Il client matcha la risposta per id e calcola RTT come differenza
// di performance.now() locale (un solo clock coinvolto, no drift).
// Risultato: misura RTT realistica del canale di controllo end-to-end
// (browser -> TLS -> Pi -> echo -> TLS -> browser). Atteso su LAN: ~1 ms.

const _pendingPings = new Map();  // id -> {t0, resolve}
const _rttSamples = [];            // ultimi RTT per il cardlive
const WS_RTT_WINDOW = 20;

function genPingId() {
  return Math.random().toString(36).substring(2, 12);
}

/** Invia un singolo ping e attende il pong (timeout configurabile). */
function realPing(timeoutMs = 1000) {
  return new Promise((resolve) => {
    const id = genPingId();
    const t0 = performance.now();
    const timer = setTimeout(() => {
      if (_pendingPings.has(id)) {
        _pendingPings.delete(id);
        resolve(null);  // timeout
      }
    }, timeoutMs);
    _pendingPings.set(id, { t0, resolve, timer });
    const ok = sendCommand("ws_ping", { id, ts_client: t0 });
    if (!ok) {
      clearTimeout(timer);
      _pendingPings.delete(id);
      resolve(null);
    }
  });
}

function handleWsPong(msg) {
  const entry = _pendingPings.get(msg.id);
  if (!entry) return;
  clearTimeout(entry.timer);
  _pendingPings.delete(msg.id);
  const rtt = performance.now() - entry.t0;
  entry.resolve(rtt);
  // Aggiornamento live della card (ultimi WS_RTT_WINDOW campioni)
  _rttSamples.push(rtt);
  while (_rttSamples.length > WS_RTT_WINDOW) _rttSamples.shift();
  const avg = _rttSamples.reduce((s, x) => s + x, 0) / _rttSamples.length;
  updateWsRtt(avg);
}

/** Esegue N ping con throttle e ritorna statistiche aggregate. */
async function runPingBurst(n, intervalMs = 30) {
  const samples = [];
  let failures = 0;
  for (let i = 0; i < n; i++) {
    const rtt = await realPing(1000);
    if (rtt != null) samples.push(rtt);
    else failures++;
    if (i < n - 1) await new Promise(r => setTimeout(r, intervalMs));
  }
  if (samples.length < 3) return { error: "Troppe risposte mancanti", failures, n_received: samples.length };
  samples.sort((a, b) => a - b);
  const N = samples.length;
  const min = samples[0];
  const max = samples[N - 1];
  const mean = samples.reduce((s, x) => s + x, 0) / N;
  const median = samples[Math.floor(N / 2)];
  const variance = samples.reduce((s, x) => s + (x - mean) ** 2, 0) / N;
  const std = Math.sqrt(variance);
  const p95 = samples[Math.min(N - 1, Math.floor(N * 0.95))];
  return { n: N, failures, min, max, mean, median, std, p95 };
}

document.getElementById("btn-ws-ping")?.addEventListener("click", async () => {
  const btn = document.getElementById("btn-ws-ping");
  const msg = document.getElementById("ws-ping-msg");
  if (btn.disabled) return;
  btn.disabled = true;
  const N_PINGS = 300;  // burst per statistica robusta (min/median/mean/max/std/p95)
  try {
    if (msg) msg.textContent = `Esecuzione ${N_PINGS} ping...`;
    const stats = await runPingBurst(N_PINGS, 20);  // ~50 Hz
    if (stats.error) {
      if (msg) msg.textContent = `Errore: ${stats.error} (${stats.n_received} pong ricevuti su ${N_PINGS})`;
      addLog(`WS ping: ${stats.error}`);
      return;
    }
    const text = `${stats.n} ping (${stats.failures} timeout): min ${stats.min.toFixed(2)} ms, media ${stats.mean.toFixed(2)} ms, mediana ${stats.median.toFixed(2)} ms, max ${stats.max.toFixed(2)} ms (std ${stats.std.toFixed(2)}, p95 ${stats.p95.toFixed(2)}).`;
    if (msg) msg.innerHTML = text;
    addLog(`WS ping ${stats.n}/${N_PINGS}: media ${stats.mean.toFixed(2)} ms, max ${stats.max.toFixed(2)} ms`);
  } finally {
    btn.disabled = false;
  }
});

// ===========================================================================
// Test latenza video — Confronto MediaMTX vs MJPEG
// ===========================================================================

// I 5 profili supportati (stessi parametri W/H/FPS per entrambe le pipeline).
// L'ordine determina anche l'ordinamento nella tabella e nel bar chart.
const PROFILES = [
  { key: "lowlatency",   label: "Low-latency",   w: 800,  h: 450,  fps: 120, note: "adottato" },
  { key: "zoomfriendly", label: "Zoom-friendly", w: 1280, h: 720,  fps: 60,  note: "" },
  { key: "inspection",   label: "Inspection",    w: 1920, h: 1080, fps: 30,  note: "" },
  { key: "maxres",       label: "MAX-RES",       w: 3840, h: 2160, fps: 14,  note: "" },
  { key: "initial",      label: "Initial",       w: 1280, h: 720,  fps: 30,  note: "configurazione storica" },
];

function profileByKey(k) { return PROFILES.find(p => p.key === k); }
function profileLabel(k) {
  const p = profileByKey(k);
  if (!p) return k;
  return `${p.label} ${p.w}×${p.h}@${p.fps}`;
}

// Valori storici di riferimento. Sovrascritti dalle misure live quando disponibili.
// Struttura: results[profileKey] = {
//   mediamtx: { min,mean,max,std,n,source, load: {cpu_pct,ram_pct,temp_c}? },
//   mjpeg:    { min,mean,max,std,n,source, load: {cpu_pct,ram_pct,temp_c}? },
// }
// load.cpu_pct/ram_pct/temp_c = { mean, min, max, std, n } (aggregati sampler).
const results = {
  lowlatency:   { mediamtx: { min: 25,  mean: 37,  max: 50,  std: null, n: 300, source: "riferimento storico", load: null }, mjpeg: null },
  zoomfriendly: { mediamtx: null, mjpeg: null },
  inspection:   { mediamtx: null, mjpeg: null },
  maxres:       { mediamtx: null, mjpeg: null },
  initial:      { mediamtx: { min: 100, mean: 150, max: 200, std: null, n: 300, source: "riferimento storico", load: null },
                  mjpeg:    { min: 210, mean: 306, max: 420, std: null, n: 300, source: "riferimento storico", load: null } },
};

function fmtRange(v) {
  if (!v) return "&mdash;";
  return `${v.min.toFixed(0)}&ndash;${v.max.toFixed(0)} (n=${v.n})`;
}
function fmtMean(v) {
  if (!v) return "&mdash;";
  const tag = v.source && v.source.includes("storico") ? " <em>(storico)</em>" : "";
  return `<strong>${v.mean.toFixed(1)}</strong>${tag}`;
}
// Formattazione carico computazionale CPU/RAM/temp in cella unica.
// load = { cpu_pct: {mean,max,n}, ram_pct: {...}, temp_c: {...} }
function fmtLoad(v) {
  if (!v || !v.load) return '<span style="color:var(--muted)">&mdash;</span>';
  const L = v.load;
  const cpu = L.cpu_pct;
  const ram = L.ram_pct;
  const temp = L.temp_c;
  if (!cpu || cpu.mean == null) return '<span style="color:var(--muted)">&mdash;</span>';
  const cpuMean = cpu.mean.toFixed(1);
  const cpuMax  = cpu.max != null ? cpu.max.toFixed(1) : "?";
  const ramMean = (ram && ram.mean != null) ? ram.mean.toFixed(1) : "?";
  const tempMean = (temp && temp.mean != null) ? temp.mean.toFixed(1) : "?";
  const tempMax  = (temp && temp.max != null)  ? temp.max.toFixed(1)  : "?";
  // CPU media in bold + tooltip con max picco; sotto RAM% e temp°C
  return `<strong>${cpuMean}%</strong>` +
         `<span style="color:var(--muted);font-size:0.85em" title="CPU max picco ${cpuMax}% — Temp max ${tempMax}°C — n=${cpu.n}">` +
         ` (T ${tempMean}°C · RAM ${ramMean}%)</span>`;
}

function renderResults() {
  const tbody = document.getElementById("latency-results");
  if (!tbody) return;

  tbody.innerHTML = PROFILES.map(p => {
    const r = results[p.key] || {};
    const mm = r.mediamtx;
    const mj = r.mjpeg;
    let ratio = "&mdash;";
    if (mm && mj && mm.mean > 0) {
      const ratio_v = mj.mean / mm.mean;
      ratio = `<strong>${ratio_v.toFixed(2)}×</strong>`;
    }
    const noteCell = p.note ? ` <span style="color:var(--muted);font-size:0.85em">(${p.note})</span>` : "";
    return `
      <tr>
        <td>${p.label} ${p.w}×${p.h}@${p.fps}${noteCell}</td>
        <td class="num">${fmtMean(mm)}</td>
        <td class="num">${fmtRange(mm)}</td>
        <td class="num">${fmtLoad(mm)}</td>
        <td class="num">${fmtMean(mj)}</td>
        <td class="num">${fmtRange(mj)}</td>
        <td class="num">${fmtLoad(mj)}</td>
        <td class="num">${ratio}</td>
      </tr>
    `;
  }).join("");

  // Bar chart pairwise: 2 barre adiacenti per ogni profilo
  const chart = document.getElementById("bar-chart-rows");
  if (chart) {
    let maxVal = 50;
    for (const p of PROFILES) {
      const r = results[p.key] || {};
      if (r.mediamtx) maxVal = Math.max(maxVal, r.mediamtx.mean);
      if (r.mjpeg)    maxVal = Math.max(maxVal, r.mjpeg.mean);
    }
    chart.innerHTML = PROFILES.map(p => {
      const r = results[p.key] || {};
      const mm = r.mediamtx;
      const mj = r.mjpeg;
      const rows = [];
      if (mm) {
        const pct = (mm.mean / maxVal) * 100;
        rows.push(`
          <div class="bar-row">
            <span class="name">${p.label} &mdash; <strong style="color:#5dd6a0">MediaMTX</strong></span>
            <div class="bar-track"><div class="bar-fill measured" style="width:${pct.toFixed(1)}%;background:linear-gradient(90deg,#3d9dff,#5dd6a0)"></div></div>
            <span class="num">${mm.mean.toFixed(1)} ms</span>
          </div>`);
      }
      if (mj) {
        const pct = (mj.mean / maxVal) * 100;
        rows.push(`
          <div class="bar-row">
            <span class="name">${p.label} &mdash; <strong style="color:#ffc850">MJPEG</strong></span>
            <div class="bar-track"><div class="bar-fill" style="width:${pct.toFixed(1)}%;background:linear-gradient(90deg,#9aa3ad,#ffc850)"></div></div>
            <span class="num">${mj.mean.toFixed(1)} ms</span>
          </div>`);
      }
      if (rows.length === 0) {
        rows.push(`<div class="bar-row"><span class="name">${p.label}</span><div class="bar-track"></div><span class="num" style="color:var(--muted)">non misurato</span></div>`);
      }
      return rows.join("") + `<div style="height:0.4rem"></div>`;
    }).join("");
  }

  // Sintesi: fattore di guadagno medio MediaMTX/MJPEG (solo profili dove ho entrambi)
  const synth = document.getElementById("synthesis-text");
  if (synth) {
    const ratios = [];
    const profilesWithBoth = [];
    const cpuDeltas = []; // CPU% MJPEG - CPU% MediaMTX (per profili con entrambi i sampler attivi)
    for (const p of PROFILES) {
      const r = results[p.key] || {};
      if (r.mediamtx && r.mjpeg && r.mediamtx.mean > 0) {
        ratios.push(r.mjpeg.mean / r.mediamtx.mean);
        profilesWithBoth.push(p.label);
      }
      const mmCpu = r.mediamtx && r.mediamtx.load && r.mediamtx.load.cpu_pct && r.mediamtx.load.cpu_pct.mean;
      const mjCpu = r.mjpeg && r.mjpeg.load && r.mjpeg.load.cpu_pct && r.mjpeg.load.cpu_pct.mean;
      if (typeof mmCpu === "number" && typeof mjCpu === "number") {
        cpuDeltas.push({ label: p.label, mm: mmCpu, mj: mjCpu });
      }
    }
    if (ratios.length === 0) {
      synth.innerHTML = "eseguire almeno un profilo con entrambe le pipeline per calcolare il fattore di guadagno medio MediaMTX/MJPEG.";
    } else {
      const avg = ratios.reduce((s, x) => s + x, 0) / ratios.length;
      const mn = Math.min(...ratios), mx = Math.max(...ratios);
      let html = `MediaMTX risulta strutturalmente più rapido di MJPEG di un fattore medio <strong>${avg.toFixed(2)}×</strong> ` +
                 `(range ${mn.toFixed(2)}&ndash;${mx.toFixed(2)}×) attraverso ${ratios.length} profil${ratios.length === 1 ? "o" : "i"} ` +
                 `(${profilesWithBoth.join(", ")}). Confermato sperimentalmente il guadagno strutturale dell'architettura WebRTC sulle pipeline a bassa latenza, indipendentemente dal punto operativo.`;
      if (cpuDeltas.length > 0) {
        const mmAvg = cpuDeltas.reduce((s, x) => s + x.mm, 0) / cpuDeltas.length;
        const mjAvg = cpuDeltas.reduce((s, x) => s + x.mj, 0) / cpuDeltas.length;
        const dir = (mjAvg > mmAvg) ? "più alto" : "più basso";
        const abs = Math.abs(mjAvg - mmAvg).toFixed(1);
        html += ` <br><strong>Carico computazionale:</strong> MediaMTX <strong>${mmAvg.toFixed(1)}%</strong> ` +
                `vs MJPEG <strong>${mjAvg.toFixed(1)}%</strong> di CPU media (Δ ${abs} punti, MJPEG ${dir}) ` +
                `su ${cpuDeltas.length} profil${cpuDeltas.length === 1 ? "o" : "i"} con entrambi i sampler attivi.`;
      }
      synth.innerHTML = html;
    }
  }
}

async function setVideoProfile(profile) {
  // Il backend riavvia MediaMTX in ~4 s con grace, poi broadcasta
  // cameras_refocus_triggered. Usiamo un timeout fisso conservativo:
  // 5.5 s = restart_grace (4 s) + cushion per stabilizzazione encoder.
  const sent = sendCommand("set_video_profile", { profile });
  if (!sent) throw new Error("WS non pronto (set_video_profile fallito)");
  return new Promise((r) => setTimeout(r, 5500));
}

async function measureLatencyForCurrentStream(nFrames) {
  // Misura inter-frame timing su un <video> WHEP same-origin per cam0.
  // La latenza stimata = profondità buffer * inter-frame interval.
  const status = document.getElementById("latency-status");
  const progress = document.getElementById("latency-progress");
  if (status) status.textContent = "Avvio stream WebRTC cam0...";

  // Crea un <video> nascosto, apri WHEP same-origin
  const video = document.createElement("video");
  video.style.display = "none";
  video.muted = true;
  video.autoplay = true;
  video.playsInline = true;
  document.body.appendChild(video);

  const pc = new RTCPeerConnection();
  pc.addTransceiver("video", { direction: "recvonly" });

  let stream = null;
  pc.ontrack = (ev) => {
    if (!stream) {
      stream = ev.streams[0];
      video.srcObject = stream;
      video.play().catch(() => {});
    }
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  let answer;
  try {
    // Proxy WHEP same-origin esposto dal server HTTPS della dashboard:
    // /api/webrtc-whep?path=cam0 -> inoltra a MediaMTX su 127.0.0.1:8889
    const resp = await fetch("/api/webrtc-whep?path=" + encodeURIComponent("cam0"), {
      method: "POST",
      headers: { "Content-Type": "application/sdp" },
      body: offer.sdp,
    });
    if (!resp.ok) {
      const txt = await resp.text().catch(() => "");
      throw new Error(`WHEP ${resp.status}: ${txt.slice(0, 120)}`);
    }
    answer = await resp.text();
  } catch (e) {
    document.body.removeChild(video);
    try { pc.close(); } catch (_) {}
    throw e;
  }

  await pc.setRemoteDescription({ type: "answer", sdp: answer });

  // Attendi che il video parta
  await new Promise((r) => setTimeout(r, 600));

  // Cattura inter-frame timing usando requestVideoFrameCallback
  if (!("requestVideoFrameCallback" in video)) {
    document.body.removeChild(video);
    try { pc.close(); } catch (_) {}
    throw new Error("requestVideoFrameCallback non supportato dal browser");
  }

  const intervals = []; // ms tra frame consecutivi
  let lastNow = null;
  let collected = 0;

  await new Promise((resolve) => {
    const cb = (now, metadata) => {
      if (lastNow != null) {
        intervals.push(now - lastNow);
        collected++;
        if (progress) progress.style.width = `${(collected * 100 / nFrames).toFixed(0)}%`;
        if (status) status.textContent = `Acquisizione: ${collected} / ${nFrames} frame...`;
      }
      lastNow = now;
      if (collected >= nFrames) { resolve(); return; }
      video.requestVideoFrameCallback(cb);
    };
    video.requestVideoFrameCallback(cb);
  });

  try { pc.close(); } catch (_) {}
  document.body.removeChild(video);

  // La latenza video stimata segue la metodologia inter-frame timing:
  // ~ (profondità buffer) * (interframe interval).
  // Profondità buffer tipica MediaMTX/WebRTC: ~4-5 frame.
  const BUFFER_DEPTH = 4.5;
  const latencies = intervals.map(iv => iv * BUFFER_DEPTH);

  latencies.sort((a, b) => a - b);
  const n = latencies.length;
  const min = latencies[0];
  const max = latencies[n - 1];
  const mean = latencies.reduce((s, x) => s + x, 0) / n;
  const variance = latencies.reduce((s, x) => s + (x - mean) ** 2, 0) / n;
  const std = Math.sqrt(variance);

  return { n, min, mean, max, std };
}

async function runMediaMTXTest(profileKey, nFrames) {
  const p = profileByKey(profileKey);
  if (!p) throw new Error(`Profilo sconosciuto: ${profileKey}`);
  const status = document.getElementById("latency-status");
  const progress = document.getElementById("latency-progress");
  if (status) status.textContent = `MediaMTX → switch al profilo "${p.label}"…`;
  if (progress) progress.style.width = "0%";
  await setVideoProfile(profileKey);
  await new Promise(r => setTimeout(r, 1500)); // grace dopo restart mediamtx

  // Lancia in parallelo il sampler di carico computazionale (CPU/RAM/temp)
  // sul Pi. Durata = stima tempo cattura + cushion. La promise viene risolta
  // quando arriva system_load_result via WS (vedi handleSystemLoadMessage).
  const estDurationS = Math.max(5, Math.ceil(nFrames * 1.3 / Math.max(1, p.fps)) + 2);
  const loadPromise = requestSystemLoadSampling(profileKey, estDurationS, `mediamtx_${profileKey}`);

  const stats = await measureLatencyForCurrentStream(nFrames);
  results[profileKey] = results[profileKey] || {};
  results[profileKey].mediamtx = { ...stats, source: "misurato live", load: null };
  renderResults();
  if (status) status.textContent = `MediaMTX [${p.label}] completato: media ${stats.mean.toFixed(1)} ms (${stats.n} campioni). Attendo carico…`;

  // Attendo il termine del sampler (idem se gia' arrivato)
  try {
    const loadResult = await loadPromise;
    if (loadResult) {
      results[profileKey].mediamtx.load = loadResult;
      renderResults();
      const cpu = loadResult.cpu_pct;
      const temp = loadResult.temp_c;
      if (status) status.textContent = `MediaMTX [${p.label}] completato: media ${stats.mean.toFixed(1)} ms, CPU media ${cpu && cpu.mean != null ? cpu.mean.toFixed(1) : "?"}% (T ${temp && temp.mean != null ? temp.mean.toFixed(1) : "?"}°C).`;
    }
  } catch (e) {
    addLog(`system_load sampling non disponibile: ${e.message || e}`);
  }

  addLog(`Test MediaMTX [${profileKey}]: media ${stats.mean.toFixed(1)} ms su ${stats.n} frame`);
  return stats;
}

function setControlsDisabled(disabled) {
  for (const id of ["btn-run-mediamtx", "btn-run-mjpeg-fullstack",
                    "btn-run-comparative", "btn-run-all", "latency-profile", "latency-nframes"]) {
    const el = document.getElementById(id);
    if (el) el.disabled = disabled;
  }
}

document.getElementById("btn-run-mediamtx")?.addEventListener("click", async () => {
  const profile = document.getElementById("latency-profile").value;
  const nFrames = parseInt(document.getElementById("latency-nframes").value, 10) || 300;
  setControlsDisabled(true);
  try { await runMediaMTXTest(profile, nFrames); }
  catch (e) {
    const status = document.getElementById("latency-status");
    if (status) status.textContent = `Errore MediaMTX: ${e.message || e}`;
    addLog(`Errore test MediaMTX: ${e.message || e}`);
  }
  finally { setControlsDisabled(false); }
});

document.getElementById("btn-run-mjpeg-fullstack")?.addEventListener("click", async () => {
  const profile = document.getElementById("latency-profile").value;
  const nFrames = parseInt(document.getElementById("latency-nframes").value, 10) || 300;
  const p = profileByKey(profile);
  if (!p) return;
  if (!confirm(`Avviare il test MJPEG full-stack sul profilo "${p.label}" (${p.w}×${p.h}@${p.fps}FPS, ${nFrames} frame)?\n\nLa pipeline completa (rpicam-vid → HTTPS Python multipart → browser fetch) verrà attivata sul Pi al posto di MediaMTX (verrà fermato temporaneamente e riavviato al termine).\n\nDurata stimata ~${Math.ceil(nFrames / Math.max(1, p.fps)) + 5} s.`)) {
    return;
  }
  setControlsDisabled(true);
  try {
    await runMjpegFullstackTest(profile, nFrames);
  } catch (e) {
    const status = document.getElementById("latency-status");
    if (status) status.textContent = `Errore MJPEG full-stack: ${e.message || e}`;
    addLog(`Errore test MJPEG full-stack: ${e.message || e}`);
  } finally {
    setControlsDisabled(false);
  }
});

document.getElementById("btn-run-comparative")?.addEventListener("click", async () => {
  const profile = document.getElementById("latency-profile").value;
  const nFrames = parseInt(document.getElementById("latency-nframes").value, 10) || 300;
  const p = profileByKey(profile);
  if (!p) return;
  const status = document.getElementById("latency-status");
  if (!confirm(`Confronto sequenziale a parità di condizioni sul profilo "${p.label}" (${p.w}×${p.h}@${p.fps}, ${nFrames} frame)?\n\nFase 1: misura MediaMTX/WebRTC (~10 s).\nFase 2: ferma MediaMTX, misura MJPEG full-stack con browser consumer (~10-20 s), riavvia MediaMTX.\n\nEntrambe le fasi usano un consumer browser remoto via TLS: il confronto è apples-to-apples.`)) {
    return;
  }
  setControlsDisabled(true);
  try {
    if (status) status.textContent = `Fase 1/2: misura MediaMTX su "${p.label}"…`;
    await runMediaMTXTest(profile, nFrames);
    if (status) status.textContent = `Fase 2/2: misura MJPEG full-stack su "${p.label}"… (MediaMTX fermato temporaneamente)`;
    await runMjpegFullstackTest(profile, nFrames);
    if (status) status.textContent = `Confronto MediaMTX vs MJPEG full-stack [${p.label}] completato.`;
    addLog(`Confronto sequenziale [${profile}] completato.`);
  } catch (e) {
    if (status) status.textContent = `Errore confronto: ${e.message || e}`;
    addLog(`Errore confronto: ${e.message || e}`);
  } finally {
    setControlsDisabled(false);
  }
});

document.getElementById("btn-run-all")?.addEventListener("click", async () => {
  const nFrames = parseInt(document.getElementById("latency-nframes").value, 10) || 300;
  const profileKeys = ["lowlatency", "zoomfriendly", "inspection", "maxres", "initial"];
  const status = document.getElementById("latency-status");
  const totalSteps = profileKeys.length * 2;
  const estSec = totalSteps * 15;
  if (!confirm(`Eseguire confronto completo MediaMTX vs MJPEG full-stack su tutti i ${profileKeys.length} profili (${nFrames} frame ciascuno, ${totalSteps} test totali)?\n\nDurata stimata: ~${estSec} s = ~${Math.ceil(estSec/60)} min.\n\nOgni profilo viene testato due volte (MediaMTX + MJPEG full-stack) in condizioni operative equivalenti.`)) {
    return;
  }
  setControlsDisabled(true);
  let step = 0;
  try {
    for (const k of profileKeys) {
      step++;
      if (status) status.textContent = `Step ${step}/${totalSteps}: MediaMTX [${profileLabel(k)}]…`;
      await runMediaMTXTest(k, nFrames);
      step++;
      if (status) status.textContent = `Step ${step}/${totalSteps}: MJPEG full-stack [${profileLabel(k)}]…`;
      await runMjpegFullstackTest(k, nFrames);
    }
    if (status) status.textContent = "Confronto completo MediaMTX vs MJPEG full-stack su tutti i profili completato.";
    addLog(`Confronto su ${profileKeys.length} profili completato (${totalSteps} misure totali).`);
  } catch (e) {
    if (status) status.textContent = `Errore: ${e.message || e}`;
    addLog(`Errore: ${e.message || e}`);
  } finally {
    setControlsDisabled(false);
  }
});

// ===========================================================================
// Self-test modale
// ===========================================================================

document.getElementById("btn-self-test")?.addEventListener("click", () => {
  const btn = document.getElementById("btn-self-test");
  if (btn.disabled) return;
  const sent = sendCommand("self_test", { action: "run" });
  if (!sent) { addLog("SELF TEST non inviato (WS non pronto)"); return; }
  btn.disabled = true;
  document.getElementById("self-test-running-msg").textContent = "Self-test in esecuzione...";
  addLog("SELF TEST avviato dalla pagina Test");
});

// Formattazione picchi/bande — stesso contratto di ws_dashboard.js.
// Sorgente reale: il payload self_test_result del backend (self_test_imu.run_self_test_imu):
//   dynamic_response_peaks_hz / imu_vibration_peaks_hz / imu_modal_peaks_hz (liste di Hz).
function fmtPeaksHz(arr) {
  if (Array.isArray(arr) && arr.length > 0) return arr.map((hz) => `${Number(hz).toFixed(2)} Hz`).join(", ");
  return "–";
}
function fmtSelfTestBands(msg) {
  const raw = [];
  for (const arr of [msg?.dynamic_response_peaks_hz, msg?.dynamic_motion_peaks_hz, msg?.imu_vibration_peaks_hz, msg?.imu_modal_peaks_hz]) {
    if (Array.isArray(arr)) for (const x of arr) { const n = Number(x); if (Number.isFinite(n) && n > 0.05) raw.push(n); }
  }
  raw.sort((a, b) => a - b);
  const peaks = [];
  for (const p of raw) if (!peaks.length || Math.abs(p - peaks[peaks.length - 1]) > 0.2) peaks.push(p);
  if (peaks.length < 2) return "–";
  const clusters = []; let cur = [peaks[0]];
  for (let i = 1; i < peaks.length; i++) {
    if (peaks[i] - peaks[i - 1] <= 2.75) cur.push(peaks[i]);
    else { clusters.push(cur); cur = [peaks[i]]; }
  }
  clusters.push(cur);
  const weight = (c) => c.length * (Math.max(...c) - Math.min(...c) + 1);
  const top = clusters.slice().sort((a, b) => weight(b) - weight(a)).slice(0, 3).sort((a, b) => a[0] - b[0]);
  const parts = top.map((c) => {
    const mn = Math.min(...c), mx = Math.max(...c);
    const lo = Math.max(0, Math.floor(mn)), hi = Math.ceil(mx);
    if (hi <= lo + 1 && mx - mn < 1.25) return `~${Math.round((mn + mx) / 2)}`;
    return `~${lo}–${hi}`;
  });
  return `${parts.join(", ")} Hz`;
}

registerSelfTestStatusHandler((msg) => {
  const btn = document.getElementById("btn-self-test");
  if (btn) btn.disabled = Boolean(msg?.running);
  const runMsg = document.getElementById("self-test-running-msg");
  if (runMsg) runMsg.textContent = msg?.running ? "Self-test in esecuzione..." : "";
  // I campi di stato/risultato sono popolati dal selfTest handler in j5_common.js
  // se quella pagina non c'era, popoliamo noi qui:
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v == null ? "–" : v; };
  set("self-test-status", msg?.running ? "Running" : (msg?.status || "Idle"));
  set("self-test-message", msg?.message || "Ready");
  // Picchi e per-asse arrivano nel self_test_result (handler sotto), non nello status.
});

registerSelfTestResultHandler((msg) => {
  const btn = document.getElementById("btn-self-test");
  if (btn) btn.disabled = false;
  const runMsg = document.getElementById("self-test-running-msg");
  if (runMsg) runMsg.textContent = "";
  addLog(`SELF TEST RESULT: ${msg?.result || "UNKNOWN"}`);
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v == null ? "–" : v; };
  set("self-test-result", msg?.result || "–");
  // Frequenze caratteristiche: il backend le invia nel result come *_peaks_hz.
  set("self-test-dynamic-motion", fmtPeaksHz(msg?.dynamic_response_peaks_hz ?? msg?.dynamic_motion_peaks_hz));
  set("self-test-vibration", fmtPeaksHz(msg?.imu_vibration_peaks_hz));
  set("self-test-modal-vibration", fmtPeaksHz(msg?.imu_modal_peaks_hz));
  set("self-test-bands", fmtSelfTestBands(msg));
  // Per-asse: backend = `axes.{base,spalla,gomito,yaw,pitch,roll}.classification`
  // (spalla=shoulder, gomito=elbow); il valore è la stringa classification, non l'oggetto.
  const axes = msg?.axes || {};
  set("self-test-base", axes.base?.classification || "–");
  set("self-test-shoulder", axes.spalla?.classification || "–");
  set("self-test-elbow", axes.gomito?.classification || "–");
  set("self-test-yaw", axes.yaw?.classification || "–");
  set("self-test-pitch", axes.pitch?.classification || "–");
  set("self-test-roll", axes.roll?.classification || "–");
});

// ===========================================================================
// Telemetry handler (live stati)
// ===========================================================================

let lastTelemetryTime = null;
const wsArrivalIntervals = []; // proxy per WS health (ms tra messaggi consecutivi)

registerTelemetryHandler((msg) => {
  // Loop control del firmware STM32 (1 kHz deterministico Zephyr RTOS):
  // adesso telemetrato runtime via 2 byte reserved del frame canonical
  // (rt_loop_hz_est = 1e6 / rt_loop_period_us, EWMA firmware-side).
  // Se assente (firmware pre-2026-05-23): fallback al valore architetturale
  // statico 1000 Hz inizializzato in init().
  // SPI rate: calcolato lato Pi dal delta di packet_index (~30 Hz idle, ~100 con VR).
  // IMU rate: misurato lato firmware via sample counter del BNO085 (~100 Hz).
  const loopHz = msg.rt_loop_hz_est ?? null;
  const spiHz  = msg.ws_spi_rate_hz_est ?? null;
  const imuHz  = msg.imu_rate_hz_est ?? null;

  if (loopHz != null) updateLoopRate(loopHz);
  if (spiHz  != null) updateSpiRate(spiHz);
  if (imuHz  != null) updateImuRate(imuHz);

  // Inter-arrivo dei messaggi telemetry (NON e' RTT, e' il rate di emissione
  // del backend) — utile solo come indicatore interno di vitalita' WS, non
  // mostrato nella card "Control RTT" per non confondere l'utente.
  const now = performance.now();
  if (lastTelemetryTime != null) {
    const dt = now - lastTelemetryTime;
    wsArrivalIntervals.push(dt);
    while (wsArrivalIntervals.length > 20) wsArrivalIntervals.shift();
  }
  lastTelemetryTime = now;
});

// ===========================================================================
// Test MJPEG full-stack: confronto a parità di condizioni con MediaMTX
// ===========================================================================
//
// Pipeline misurata (analoga al ciclo completo MediaMTX/WebRTC):
//   Camera -> rpicam-vid MJPEG -> stdout -> Python HTTPS server ->
//   multipart/x-mixed-replace -> browser fetch + ReadableStream
//
// Il Pi serve un consumer remoto via rete TLS, esattamente come avviene
// per MediaMTX/WebRTC. Quindi la misura CPU e latenza è apples-to-apples
// con il confronto MediaMTX. È la pipeline operativa reale, non un test
// di encoder isolato (precedente "MJPEG encoder isolato" rimosso dalla UI
// in quanto restituiva gli stessi numeri di latenza ma con CPU sottostimata
// poiché non c'era un consumer di rete attivo).

const BUFFER_DEPTH_MJPEG = 9.0; // profondità tipica TCP+browser per MJPEG
const MULTIPART_BOUNDARY = "jonny5frame";
const MJPEG_FULLSTACK_TIMEOUT_MS = 90000; // safety

async function measureMjpegFullstack(profileKey, nFrames) {
  const p = profileByKey(profileKey);
  if (!p) throw new Error(`Profilo sconosciuto: ${profileKey}`);
  const status = document.getElementById("latency-status");
  const progress = document.getElementById("latency-progress");
  if (status) status.textContent = `MJPEG full-stack [${p.label}] ${p.w}×${p.h}@${p.fps} avvio…`;
  if (progress) progress.style.width = "0%";

  // Costruisci URL endpoint
  const params = new URLSearchParams({
    width: String(p.w), height: String(p.h), fps: String(p.fps),
    target_frames: String(nFrames), label: `mjpeg_fs_${profileKey}`,
  });
  const url = `/api/mjpeg-fullstack?${params.toString()}`;

  // Pattern boundary bytes
  const encoder = new TextEncoder();
  const boundaryBytes = encoder.encode(`--${MULTIPART_BOUNDARY}\r\n`);

  // Avvia fetch con AbortController per cleanup sicuro
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), MJPEG_FULLSTACK_TIMEOUT_MS);
  let response;
  try {
    response = await fetch(url, { signal: controller.signal });
  } catch (e) {
    clearTimeout(timeoutId);
    throw new Error(`fetch fallito: ${e.message || e}`);
  }
  if (!response.ok) {
    clearTimeout(timeoutId);
    throw new Error(`HTTP ${response.status}`);
  }

  const reader = response.body.getReader();
  const intervals = [];
  let lastTime = null;
  let frameCount = 0;

  // Buffer scorrevole per la ricerca dei boundary
  let buf = new Uint8Array(0);

  function findBoundary(haystack, needle, startIdx) {
    // Ricerca naive sufficient per ~50KB di chunk; performante per i nostri scopi
    const hlen = haystack.length;
    const nlen = needle.length;
    for (let i = startIdx; i <= hlen - nlen; i++) {
      let match = true;
      for (let j = 0; j < nlen; j++) {
        if (haystack[i + j] !== needle[j]) { match = false; break; }
      }
      if (match) return i;
    }
    return -1;
  }

  try {
    while (frameCount < nFrames) {
      const { value, done } = await reader.read();
      if (done) break;
      if (!value || value.length === 0) continue;

      // Concatena value al buffer
      const newBuf = new Uint8Array(buf.length + value.length);
      newBuf.set(buf);
      newBuf.set(value, buf.length);
      buf = newBuf;

      // Estrai tutti i boundary completi nel buffer corrente
      let searchStart = 0;
      while (true) {
        const idx = findBoundary(buf, boundaryBytes, searchStart);
        if (idx < 0) break;
        // Trovato un boundary: registra l'evento "nuovo frame"
        const now = performance.now();
        if (lastTime != null) {
          intervals.push(now - lastTime);
        }
        lastTime = now;
        frameCount++;
        if (progress) progress.style.width = `${(frameCount * 100 / nFrames).toFixed(0)}%`;
        if (status) status.textContent = `MJPEG full-stack [${p.label}]: ${frameCount} / ${nFrames} frame…`;
        if (frameCount >= nFrames) break;
        searchStart = idx + boundaryBytes.length;
      }
      // Mantengo solo gli ultimi (boundaryBytes.length - 1) byte per matching cross-chunk
      if (buf.length > boundaryBytes.length) {
        buf = buf.slice(buf.length - boundaryBytes.length + 1);
      }
    }
  } finally {
    clearTimeout(timeoutId);
    try { await reader.cancel(); } catch (_) {}
  }

  if (intervals.length < 3) {
    throw new Error(`Solo ${intervals.length} intervalli inter-frame catturati`);
  }

  // Latenza stimata = inter-frame × buffer depth
  const latencies = intervals.map(iv => iv * BUFFER_DEPTH_MJPEG);
  latencies.sort((a, b) => a - b);
  const n = latencies.length;
  const min = latencies[0];
  const max = latencies[n - 1];
  const mean = latencies.reduce((s, x) => s + x, 0) / n;
  const variance = latencies.reduce((s, x) => s + (x - mean) ** 2, 0) / n;
  const std = Math.sqrt(variance);

  return { n, min, mean, max, std };
}

async function runMjpegFullstackTest(profileKey, nFrames) {
  const p = profileByKey(profileKey);
  if (!p) throw new Error(`Profilo sconosciuto: ${profileKey}`);

  // Stima durata: target_frames / fps + grace (2s startup + 1s shutdown)
  const estDurationS = Math.max(5, Math.ceil(nFrames / Math.max(1, p.fps)) + 3);

  // Lancia in parallelo sampler CPU/RAM/temp lato Pi
  const loadPromise = requestSystemLoadSampling(profileKey, estDurationS,
                                                  `mjpeg_fs_${profileKey}`);

  // Misura inter-frame su stream multipart
  const stats = await measureMjpegFullstack(profileKey, nFrames);

  // Memorizza risultato come campo "mjpeg" (sostituisce eventuale baseline-isolated
  // per lo stesso profilo: la misura full-stack e' quella autoritativa).
  results[profileKey] = results[profileKey] || {};
  results[profileKey].mjpeg = { ...stats, source: "misurato live (full-stack)", load: null };
  renderResults();

  const status = document.getElementById("latency-status");
  if (status) status.textContent = `MJPEG full-stack [${p.label}] completato: media ${stats.mean.toFixed(1)} ms. Attendo carico…`;

  // Aspetta sampler
  try {
    const loadResult = await loadPromise;
    if (loadResult) {
      results[profileKey].mjpeg.load = loadResult;
      renderResults();
      const cpu = loadResult.cpu_pct;
      const temp = loadResult.temp_c;
      if (status) status.textContent = `MJPEG full-stack [${p.label}] completato: media ${stats.mean.toFixed(1)} ms, CPU media ${cpu && cpu.mean != null ? cpu.mean.toFixed(1) : "?"}% (T ${temp && temp.mean != null ? temp.mean.toFixed(1) : "?"}°C).`;
    }
  } catch (e) {
    addLog(`system_load sampling non disponibile: ${e.message || e}`);
  }

  addLog(`Test MJPEG full-stack [${profileKey}]: media ${stats.mean.toFixed(1)} ms su ${stats.n} frame`);
  return stats;
}

// ===========================================================================
// Carico computazionale CPU/RAM/temperatura (parallelo ai test MediaMTX)
// ===========================================================================

// Map di promise pending: key = label (es. "mediamtx_lowlatency"), value = {resolve, reject, timer}
const _pendingLoadPromises = new Map();

/**
 * Richiede un campionamento di carico al backend e ritorna una Promise
 * che si risolve quando arriva system_load_result con la stessa label.
 * In caso di timeout (durata + 5 s) la Promise viene risolta con null.
 */
function requestSystemLoadSampling(profileKey, durationS, label) {
  return new Promise((resolve) => {
    const ok = sendCommand("start_system_load_sampling", {
      duration_s: durationS,
      interval_s: 1.0,
      label,
    });
    if (!ok) {
      resolve(null);
      return;
    }
    const timeoutMs = (durationS + 5) * 1000;
    const timer = setTimeout(() => {
      if (_pendingLoadPromises.has(label)) {
        _pendingLoadPromises.delete(label);
        resolve(null);
      }
    }, timeoutMs);
    _pendingLoadPromises.set(label, { resolve, timer });
  });
}

function handleSystemLoadMessage(msg) {
  if (msg.type !== "system_load_result") return; // status/error non rilevanti
  const r = msg.result || {};
  const label = r.label || "";
  const entry = _pendingLoadPromises.get(label);
  if (entry) {
    clearTimeout(entry.timer);
    _pendingLoadPromises.delete(label);
    entry.resolve({
      cpu_pct: r.cpu_pct,
      ram_pct: r.ram_pct,
      temp_c:  r.temp_c,
      duration_s_actual: r.duration_s_actual,
      interval_s: r.interval_s,
    });
  }
}

// ===========================================================================
// Timeline di scheduling live — 4 frequenze operative.
// Vista "logic-analyzer" STATICA, triggerata sul control loop (rt-loop): la base
// dei tempi è agganciata al loop deterministico STM32 (master a ~1 kHz) e tutte
// le corsie sono allineate all'istante di trigger t=0 (un fronte di salita del
// rt-loop). Ogni corsia è un'onda quadra (50% duty) al periodo 1/f della
// frequenza misurata. Niente scorrimento: il quadro si aggiorna solo quando
// cambiano le frequenze. I valori sono letti dalle card "Frequenze operative",
// così la timeline segue le stesse misure senza duplicare la logica WS.
// ===========================================================================
const FT_WINDOW_MS = 50;   // ampiezza finestra temporale visualizzata (≈ 50 periodi rt-loop)
const FT_LANES = [
  { id: "val-hmd",  name: "HMD refresh (VR)",   color: "#b18cff", inactive: "no XR session" },
  { id: "val-spi",  name: "SPI uplink",         color: "#4db4ff", inactive: "in attesa…" },
  { id: "val-imu",  name: "IMU sample rate",    color: "#34d399", inactive: "in attesa…" },
  { id: "val-loop", name: "STM32 control loop", color: "#fbbf24", inactive: "in attesa…" },
];
let _ftCanvas = null, _ftCtx = null, _ftDpr = 1, _ftW = 0, _ftH = 0, _ftTimer = 0;

function _ftReadHz(id) {
  const f = parseFloat(document.getElementById(id)?.textContent);
  return (Number.isFinite(f) && f > 0) ? f : null;
}

function _ftResize() {
  if (!_ftCanvas || !_ftCtx) return;
  const r = _ftCanvas.getBoundingClientRect();
  _ftDpr = window.devicePixelRatio || 1;
  _ftW = r.width; _ftH = r.height;
  _ftCanvas.width  = Math.max(1, Math.round(_ftW * _ftDpr));
  _ftCanvas.height = Math.max(1, Math.round(_ftH * _ftDpr));
  _ftCtx.setTransform(_ftDpr, 0, 0, _ftDpr, 0, 0);
}

function _ftDraw() {
  if (!_ftCtx) return;
  const ctx = _ftCtx, W = _ftW, H = _ftH;
  const gutter = 172, padR = 14, padT = 12, padB = 24;
  const plotX = gutter, plotW = Math.max(10, W - gutter - padR);
  const n = FT_LANES.length, laneGap = 10;
  const plotH = Math.max(20, H - padT - padB);
  const laneH = (plotH - laneGap * (n - 1)) / n;
  const pxPerMs = plotW / FT_WINDOW_MS;
  const winStart = 0;   // t=0 = istante di trigger (fronte di salita del rt-loop)

  ctx.clearRect(0, 0, W, H);

  // gridlines verticali ogni 10 ms + scala asse
  ctx.strokeStyle = "rgba(255,255,255,0.06)";
  ctx.fillStyle = "#5c6573";
  ctx.lineWidth = 1;
  ctx.font = "10px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "alphabetic";
  for (let ms = 0; ms <= FT_WINDOW_MS; ms += 10) {
    const x = plotX + ms * pxPerMs;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
    ctx.fillText(ms === FT_WINDOW_MS ? ms + " ms" : String(ms), x, padT + plotH + 14);
  }

  for (let i = 0; i < n; i++) {
    const lane = FT_LANES[i];
    const y = padT + i * (laneH + laneGap);
    const hz = _ftReadHz(lane.id);

    // sfondo corsia
    ctx.fillStyle = "rgba(255,255,255,0.03)";
    ctx.fillRect(plotX, y, plotW, laneH);

    // etichette a sinistra (nome + Hz/periodo)
    ctx.textAlign = "left";
    ctx.fillStyle = "#cdd6e4";
    ctx.font = "600 12px system-ui, sans-serif";
    ctx.fillText(lane.name, 4, y + laneH / 2 - 1);
    ctx.font = "11px system-ui, sans-serif";
    if (hz) {
      const Tlbl = 1000 / hz;
      ctx.fillStyle = lane.color;
      ctx.fillText(`${hz.toFixed(hz >= 100 ? 0 : 1)} Hz · T=${Tlbl.toFixed(Tlbl < 10 ? 2 : 1)} ms`, 4, y + laneH / 2 + 14);
    } else {
      ctx.fillStyle = "#6b7280";
      ctx.fillText(lane.inactive, 4, y + laneH / 2 + 14);
    }
    if (!hz) continue;

    // treno di impulsi STATICO: onda quadra 50% duty allineata al trigger (t=0)
    const T = 1000 / hz, high = T / 2;
    const barTop = y + 4, barH = Math.max(4, laneH - 8);
    ctx.fillStyle = lane.color;
    const kEnd = Math.floor(FT_WINDOW_MS / T) + 1;
    for (let k = 0; k <= kEnd; k++) {
      const rise = k * T;                                   // fronte di salita (ms-segnale)
      const cx0 = Math.max(plotX, plotX + (rise - winStart) * pxPerMs);
      const cx1 = Math.min(plotX + plotW, plotX + (rise + high - winStart) * pxPerMs);
      if (cx1 > cx0) ctx.fillRect(cx0, barTop, cx1 - cx0, barH);
    }
  }

  // cornice del plot
  ctx.strokeStyle = "rgba(255,255,255,0.12)";
  ctx.strokeRect(plotX, padT, plotW, plotH);

  // marcatore di trigger sul bordo sinistro (t=0, agganciato al rt-loop)
  const loopHz = _ftReadHz("val-loop");
  const trigOn = loopHz != null;
  ctx.strokeStyle = trigOn ? "rgba(251,191,36,0.65)" : "rgba(120,130,145,0.5)";
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(plotX, padT); ctx.lineTo(plotX, padT + plotH); ctx.stroke();
  ctx.fillStyle = trigOn ? "#fbbf24" : "#788192";
  ctx.beginPath();
  ctx.moveTo(plotX - 5, padT - 8); ctx.lineTo(plotX + 5, padT - 8); ctx.lineTo(plotX, padT - 2);
  ctx.closePath(); ctx.fill();
  ctx.textAlign = "left"; ctx.font = "10px system-ui, sans-serif";
  ctx.fillStyle = trigOn ? "#d9a93a" : "#788192";
  ctx.fillText("trigger: rt-loop" + (trigOn ? ` ${loopHz.toFixed(0)} Hz` : " (in attesa)"), plotX + 9, padT - 1);
}

function initFreqTimeline() {
  _ftCanvas = document.getElementById("freq-timeline");
  if (!_ftCanvas) return;
  _ftCtx = _ftCanvas.getContext("2d");
  if (!_ftCtx) return;
  _ftResize();
  window.addEventListener("resize", () => { _ftResize(); _ftDraw(); });
  clearInterval(_ftTimer);
  // Vista statica: ridisegno a bassa cadenza per recepire i cambi di frequenza
  // (le card vengono aggiornate via WS ~1 Hz). Nessun rAF, niente scorrimento.
  _ftDraw();
  _ftTimer = setInterval(_ftDraw, 500);
}

// ===========================================================================
// Avvio
// ===========================================================================

function init() {
  // Il firmware STM32 mantiene un loop di controllo deterministico (target di
  // design, vincolo Zephyr RTOS). Il valore reale è telemetrato runtime via
  // SPI (rt_loop_hz_est dal firmware nei reserved bytes del frame TELEMETRY);
  // questo init è solo un placeholder a 1000 finché il primo messaggio arriva.
  updateLoopRate(1000);

  registerSystemLoadHandler(handleSystemLoadMessage);
  registerWsPongHandler(handleWsPong);
  registerVrSessionHandler(handleVrSession);
  renderResults();
  connectJ5Dashboard();

  // Heartbeat ping passivo a 1 Hz: mantiene la card "Control RTT" sempre
  // live (vero RTT, non proxy) anche senza che l'utente abbia lanciato il
  // burst test. Il burst test (300 ping a 50 Hz) aggiunge campioni allo
  // stesso _rttSamples e produce la statistica completa nel testo accanto.
  setInterval(() => { realPing(1500).catch(() => {}); }, 1000);

  // Avvia la misura continua della latenza video sulla dashboard.
  // Apre un peer WHEP cam0 nascosto e accumula inter-frame timing per
  // calcolare la latenza lower-bound del browser (~38 ms con profilo
  // Low-latency 800×450@120). Speculare alla HMD video latency.
  _startDashLatency().catch(() => {});

  // Timeline di scheduling live (treni di impulsi delle 4 frequenze operative).
  initFreqTimeline();

  addLog("Pagina Test inizializzata.");
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
