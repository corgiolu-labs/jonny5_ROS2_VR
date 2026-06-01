/**
 * NOTE [RPI-SAFE-REFACTOR-PHASE1]: modulo analizzato, no functional changes.
 *
 * j5_common.js — Modulo condiviso JONNY5-4.0
 *
 * Centralizza: connessione WS, routing messaggi, utility UI (pill, log,
 * quatToEuler) e sistema a callback registrabili per future pagine multi-pagina.
 *
 * Compatibilità: espone window.wsSend e window.addLog per compatibilità
 * con il codice esistente in dashboard.js (che li usa senza import).
 *
 * CORE: connectJ5Dashboard, sendCommand, register*Handler
 * UTILITY: addLog, setPill, quatToEuler
 */

import { j5_connect_ws } from "./ws_client.js";

// ---------------------------------------------------------------------------
// Stato interno
// ---------------------------------------------------------------------------
let _ws = null;
let _reconnectTimer = null;
let _reconnectAttempt = 0;
let _intentionalDisconnect = false;

// Callback registrabili per pagine specifiche
const _telemetryHandlers    = [];
const _ackHandlers          = [];
const _uartResponseHandlers = [];
const _imuEnabledHandlers   = [];
const _setposeDoneHandlers  = [];
const _settingsHandlers     = [];
const _ikResultHandlers     = [];
const _fkPoeResultHandlers  = [];
const _vrCalibHandlers      = [];
const _vrZoomCommandHandlers = [];
const _poeParamsHandlers    = [];
const _openHandlers         = [];
const _vrConfigAppliedHandlers = [];
const _selfTestStatusHandlers = [];
const _selfTestResultHandlers = [];
const _mjpegBaselineHandlers = [];
const _systemLoadHandlers   = [];
const _wsPongHandlers       = [];
const _vrSessionHandlers    = [];

// ---------------------------------------------------------------------------
// API registrazione callback
// ---------------------------------------------------------------------------

export function registerTelemetryHandler(fn) {
  if (typeof fn === "function") _telemetryHandlers.push(fn);
}

export function registerAckHandler(fn) {
  if (typeof fn === "function") _ackHandlers.push(fn);
}

export function registerUartResponseHandler(fn) {
  if (typeof fn === "function") _uartResponseHandlers.push(fn);
}

export function registerImuEnabledHandler(fn) {
  if (typeof fn === "function") _imuEnabledHandlers.push(fn);
}

export function registerSetposeDoneHandler(fn) {
  if (typeof fn === "function") _setposeDoneHandlers.push(fn);
}

export function registerSettingsHandler(fn) {
  if (typeof fn === "function") _settingsHandlers.push(fn);
}

export function registerIkResultHandler(fn) {
  if (typeof fn === "function") _ikResultHandlers.push(fn);
}

export function registerFkPoeResultHandler(fn) {
  if (typeof fn === "function") _fkPoeResultHandlers.push(fn);
}

export function registerVrZoomCommandHandler(fn) {
  if (typeof fn === "function") _vrZoomCommandHandlers.push(fn);
}

export function registerVrCalibHandler(fn) {
  if (typeof fn === "function") _vrCalibHandlers.push(fn);
}

export function registerPoeParamsHandler(fn) {
  if (typeof fn === "function") _poeParamsHandlers.push(fn);
}

export function registerOpenHandler(fn) {
  if (typeof fn === "function") _openHandlers.push(fn);
}

export function registerVrConfigAppliedHandler(fn) {
  if (typeof fn === "function") _vrConfigAppliedHandlers.push(fn);
}

export function registerSelfTestStatusHandler(fn) {
  if (typeof fn === "function") _selfTestStatusHandlers.push(fn);
}

export function registerSelfTestResultHandler(fn) {
  if (typeof fn === "function") _selfTestResultHandlers.push(fn);
}

// Handler unico per i 3 tipi mjpeg_baseline_* (status/result/error): usato
// dalla pagina Test per la misura live della pipeline MJPEG baseline storica.
export function registerMjpegBaselineHandler(fn) {
  if (typeof fn === "function") _mjpegBaselineHandlers.push(fn);
}

// Handler unico per i 3 tipi system_load_* (status/result/error): usato
// dalla pagina Test per il campionamento parallelo di CPU/RAM/temperatura
// durante i test MediaMTX (~1 Hz, /proc/stat + /proc/meminfo + thermal_zone0).
export function registerSystemLoadHandler(fn) {
  if (typeof fn === "function") _systemLoadHandlers.push(fn);
}

// Handler ws_pong (echo del backend per misura RTT reale dalla pagina Test).
// Il client invia ws_ping con {id, ts_client}; il backend risponde con
// ws_pong con gli stessi campi. RTT = performance.now() - ts_client.
export function registerWsPongHandler(fn) {
  if (typeof fn === "function") _wsPongHandlers.push(fn);
}

// Handler messaggi vr_session_refresh / vr_session_state. Pubblicati dal
// viewer XR (viewer_stereo_xr.html) ogni 2 s durante sessione attiva, con
// session.frameRate dell'HMD (Quest 1=72, Quest 2=90/120 Hz). Visualizzati
// nella card "HMD refresh" della pagina Test della dashboard.
export function registerVrSessionHandler(fn) {
  if (typeof fn === "function") _vrSessionHandlers.push(fn);
}

// ---------------------------------------------------------------------------
// Utility: quaternione → angoli Eulero in gradi
// (logica identica a ws_dashboard.js)
// ---------------------------------------------------------------------------

export function quatToEuler(w, x, y, z) {
  const t0 = +2.0 * (w * x + y * z);
  const t1 = +1.0 - 2.0 * (x * x + y * y);
  const roll = Math.atan2(t0, t1);

  let t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  const pitch = Math.asin(t2);

  const t3 = +2.0 * (w * z + x * y);
  const t4 = +1.0 - 2.0 * (y * y + z * z);
  const yaw = Math.atan2(t3, t4);

  return {
    roll:  (roll  * 180) / Math.PI,
    pitch: (pitch * 180) / Math.PI,
    yaw:   (yaw   * 180) / Math.PI,
  };
}

// ---------------------------------------------------------------------------
// Utility: imposta pill diagnostica
// (logica identica a ws_dashboard.js:setPill)
// ---------------------------------------------------------------------------

export function setPill(id, status) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = "diag-value state-pill state-" + status;
  el.textContent = status === "on" ? "ON" : status === "off" ? "OFF" : "–";
}

// ---------------------------------------------------------------------------
// Utility: event log
// (logica identica a dashboard.js:window.addLog, ma resa funzione nominale)
// ---------------------------------------------------------------------------

const _LOG_MAX = (typeof J5_CONFIG !== "undefined") ? J5_CONFIG.EVENT_LOG_MAX_ENTRIES : 50;

export function addLog(msg) {
  const log = document.getElementById("event-log");
  if (!log) return;
  const t = new Date().toLocaleTimeString();
  const entry = document.createElement("div");
  entry.textContent = `[${t}] ${msg}`;
  log.prepend(entry);
  // Circular buffer: remove oldest entries beyond limit
  while (log.children.length > _LOG_MAX) {
    log.removeChild(log.lastChild);
  }
}

// ---------------------------------------------------------------------------
// Utility: aggiorna pill diagnostiche da payload telemetria
// (logica identica a ws_dashboard.js:updateDiagnostics)
// ---------------------------------------------------------------------------

function updateDiagnostics(msg) {
  // SPI Bridge: ON se arrivano valori servo
  setPill("pill_spi", msg.servo_deg_B !== undefined ? "on" : "off");
  // STM32: ON se arrivano servo OPPURE IMU valida
  setPill("pill_stm32", (msg.servo_deg_B !== undefined || msg.imu_valid === true) ? "on" : "off");
  // IMU: valore esatto dal server (debounce fatto lato server)
  if (msg.imu_valid !== undefined) setPill("pill_imu", msg.imu_valid ? "on" : "off");
  // UART Bridge
  if (msg.uart_active !== undefined) setPill("pill_uart", msg.uart_active ? "on" : "off");
  // VR
  if (msg.vr_active !== undefined) setPill("pill_vr", msg.vr_active ? "on" : "off");
  // Pill "HYBRID" riusata come indicatore modalità VR attiva:
  // MANUAL / HEAD / HYBRID / HEAD ASSIST. Se nessuna attiva -> NO MODE (rosso).
  {
    const modeEl = document.getElementById("pill_hybrid");
    if (modeEl) {
      const mode = Number(msg.intent_mode);
      let text = "NO MODE";
      let cls = "state-off";
      if (mode === 1) {
        text = "POSE";
        cls = "state-on";
      } else if (mode === 2) {
        text = "MANUAL";
        cls = "state-on";
      } else if (mode === 3) {
        text = "HEAD";
        cls = "state-on";
      } else if (mode === 4) {
        text = "HYBRID";
        cls = "state-on";
      } else if (mode === 5) {
        text = "HEAD ASSIST";
        cls = "state-on";
      }
      modeEl.className = "diag-value state-pill " + cls;
      modeEl.textContent = text;
    }
  }
  // NOTE [HYBRID-H5]: visualizza mode teleop (0..5)
  const elemMode = document.getElementById("diag_teleop_mode");
  if (elemMode && typeof msg.teleop_mode !== "undefined") {
    elemMode.textContent = msg.teleop_mode;
  }
  // NOTE [HYBRID-H5]: intensità (solo lettura)
  const elemIntensity = document.getElementById("diag_intensity");
  if (elemIntensity && typeof msg.intensity !== "undefined") {
    elemIntensity.textContent = msg.intensity;
  }
}

// ---------------------------------------------------------------------------
// Connessione WebSocket
// ---------------------------------------------------------------------------

// Use same origin as the page (HTTPS server proxies /ws → WS server on port 8557).
// This avoids the need to accept a separate TLS certificate for port 8557 in Quest Browser.
function _getWsUrl() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/ws`;
}

/**
 * Connette il WebSocket JONNY5. Idempotente: se già connesso restituisce
 * l'istanza esistente senza aprire una seconda connessione.
 */
export function connectJ5Dashboard() {
  if (_ws && (_ws.readyState === WebSocket.OPEN || _ws.readyState === WebSocket.CONNECTING)) {
    return _ws;
  }

  if (_reconnectTimer) {
    clearTimeout(_reconnectTimer);
    _reconnectTimer = null;
  }

  const url = _getWsUrl();

  _ws = j5_connect_ws(
    url,
    // on_message
    (msg) => {
      if (msg.type === "telemetry") {
        // Aggiorna diagnostiche (comune a tutte le pagine che hanno le pill)
        updateDiagnostics(msg);
        // Notifica tutti i telemetry handler registrati (Home, future pagine)
        _telemetryHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] telemetryHandler error", e); } });

      } else if (msg.type === "ack") {
        _ackHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] ackHandler error", e); } });

      } else if (msg.type === "uart_response") {
        _uartResponseHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] uartResponseHandler error", e); } });

      } else if (msg.imu_enabled !== undefined) {
        _imuEnabledHandlers.forEach((fn) => { try { fn(msg.imu_enabled); } catch(e) { console.error("[J5] imuEnabledHandler error", e); } });
        // Compat con window._syncImuToggle esistente in dashboard.js
        if (typeof window._syncImuToggle === "function") {
          window._syncImuToggle(msg.imu_enabled);
        }

      } else if (msg.teleop_pose_ack) {
        _ackHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] ackHandler(pose) error", e); } });

      } else if (msg.type === "setpose_done") {
        _setposeDoneHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] setposeDoneHandler error", e); } });

      } else if (msg.type === "settings" || msg.type === "settings_saved" || msg.type === "offsets_applied"
                 || msg.type === "pwm_config" || msg.type === "pwm_config_applied") {
        _settingsHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] settingsHandler error", e); } });

      } else if (msg.type === "ik_result") {
        _ikResultHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] ikResultHandler error", e); } });

      } else if (msg.type === "fk_poe_result") {
        _fkPoeResultHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] fkPoeResultHandler error", e); } });

      } else if (msg.type === "poe_params" || msg.type === "poe_params_saved") {
        if (msg.type === "poe_params") {
          const S = msg.S;
          const M = msg.M;
          const persisted = msg.persisted === true;
          _poeParamsHandlers.forEach((fn) => {
            try { fn({ type: "poe_params", S, M, persisted }); } catch (e) { console.error("[J5] poeParamsHandler error", e); }
          });
        } else {
          _poeParamsHandlers.forEach((fn) => { try { fn(msg); } catch (e) { console.error("[J5] poeParamsHandler error", e); } });
        }

      } else if (msg.type === "vr_calib") {
        _vrCalibHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] vrCalibHandler error", e); } });

      } else if (msg.type === "vr_zoom_command") {
        _vrZoomCommandHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] vrZoomCommandHandler error", e); } });

      } else if (msg.type === "vr_config_applied") {
        _vrConfigAppliedHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] vrConfigAppliedHandler error", e); } });

      } else if (msg.type === "self_test_status") {
        _selfTestStatusHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] selfTestStatusHandler error", e); } });

      } else if (msg.type === "self_test_result") {
        _selfTestResultHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] selfTestResultHandler error", e); } });

      } else if (msg.type === "mjpeg_baseline_status" || msg.type === "mjpeg_baseline_result" || msg.type === "mjpeg_baseline_error") {
        _mjpegBaselineHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] mjpegBaselineHandler error", e); } });

      } else if (msg.type === "system_load_status" || msg.type === "system_load_result" || msg.type === "system_load_error") {
        _systemLoadHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] systemLoadHandler error", e); } });

      } else if (msg.type === "ws_pong") {
        _wsPongHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] wsPongHandler error", e); } });

      } else if (msg.type === "vr_session_refresh" || msg.type === "vr_session_state" || msg.type === "vr_video_latency") {
        _vrSessionHandlers.forEach((fn) => { try { fn(msg); } catch(e) { console.error("[J5] vrSessionHandler error", e); } });

      } else if (msg.type === "error") {
        addLog("ERRORE: " + (msg.message || JSON.stringify(msg)));
        // pill WS offline già gestita in on_close
      }
    },
    // on_open
    () => {
      _intentionalDisconnect = false;
      _reconnectAttempt = 0;
      const el = document.getElementById("diag-ws");
      if (el) {
        el.textContent = "Connesso";
        el.className = "diag-value state-pill state-on";
      }
      addLog("WebSocket aperto");
      _openHandlers.forEach((fn) => { try { fn(); } catch (e) { console.error("[J5] openHandler error", e); } });
    },
    // on_close
    () => {
      const el = document.getElementById("diag-ws");
      if (el) {
        el.textContent = "Disconnesso";
        el.className = "diag-value state-pill state-off";
      }
      addLog("WebSocket chiuso");
      _ws = null;
      if (_intentionalDisconnect) return;

      _reconnectAttempt += 1;
      const maxMs = (typeof J5_CONFIG !== "undefined") ? J5_CONFIG.WS_RECONNECT_MAX_MS : 3000;
      const baseMs = (typeof J5_CONFIG !== "undefined") ? J5_CONFIG.WS_RECONNECT_BASE_MS : 400;
      const delayMs = Math.min(maxMs, baseMs * Math.pow(1.5, _reconnectAttempt));
      if (el) {
        el.textContent = `Riconnessione ${Math.round(delayMs / 1000)}s`;
        el.className = "diag-value state-pill state-warn";
      }
      _reconnectTimer = setTimeout(() => {
        _reconnectTimer = null;
        connectJ5Dashboard();
      }, delayMs);
    }
  );

  return _ws;
}

// ---------------------------------------------------------------------------
// Invio comandi
// (replica identica di ws_dashboard.js:sendCommand), con helper safeSend
// ---------------------------------------------------------------------------

function safeSend(ws, payload) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return false;
  }
  ws.send(JSON.stringify(payload));
  return true;
}

if (typeof window !== "undefined") {
  window.addEventListener("beforeunload", () => {
    _intentionalDisconnect = true;
    if (_reconnectTimer) {
      clearTimeout(_reconnectTimer);
      _reconnectTimer = null;
    }
    try {
      if (_ws && _ws.readyState === WebSocket.OPEN) _ws.close();
    } catch (_) {
      // best effort during page teardown
    }
  });
}

export function sendCommand(type, payload = {}) {
  if (!_ws || _ws.readyState !== WebSocket.OPEN) {
    console.warn("[J5] WS non connesso, sendCommand ignorato:", type);
    return false;
  }
  return safeSend(_ws, { type, ...payload });
}

// ---------------------------------------------------------------------------
// Utility: imposta textContent di un elemento per ID (null-safe)
// ---------------------------------------------------------------------------

export function s(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

// ---------------------------------------------------------------------------
// Utility: flash visivo su un pulsante dopo un'azione
// ---------------------------------------------------------------------------

export function flashButton(btn, label, isError = false, timeoutMs = 1400) {
  if (!btn) return;
  const origBg    = btn.style.background;
  const origColor = btn.style.color;
  const origText  = btn.textContent.trim();
  btn.disabled = true;
  btn.style.background = isError ? "rgba(231,76,60,0.9)" : "rgba(46,204,113,0.95)";
  btn.style.color      = isError ? "#fff" : "#000";
  btn.textContent = label;
  setTimeout(() => {
    btn.style.background = origBg;
    btn.style.color      = origColor;
    btn.textContent      = origText;
    btn.disabled = false;
  }, timeoutMs);
}

// ---------------------------------------------------------------------------
// Utility: debounce generica
// ---------------------------------------------------------------------------

export function debounce(fn, ms) {
  let t;
  return (...args) => {
    clearTimeout(t);
    t = setTimeout(() => fn(...args), ms);
  };
}

// ---------------------------------------------------------------------------
// Utility: carica routing config dal Pi (GET /api/routing-config)
// ---------------------------------------------------------------------------

export async function loadRoutingConfig() {
  const res = await fetch("/api/routing-config");
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return res.json();
}

// ---------------------------------------------------------------------------
// Utility: salva routing config sul Pi (POST /api/routing-config)
// ---------------------------------------------------------------------------

export async function saveRoutingConfig(config) {
  const res = await fetch("/api/routing-config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(config),
  });
  const data = await res.json();
  if (!res.ok || !data.ok) throw new Error(data?.error || "Salvataggio routing_config fallito");
  return data;
}

// ---------------------------------------------------------------------------
// Alias globali per compatibilità con dashboard.js (usa window.wsSend/addLog)
// ---------------------------------------------------------------------------
if (typeof window !== "undefined") {
  window.wsSend  = sendCommand;
  window.addLog  = addLog;
}
