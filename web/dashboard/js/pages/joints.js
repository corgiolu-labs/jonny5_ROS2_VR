/**
 * joints.js — Pagina Joints JONNY5-4.0
 *
 * Visualizza in tempo reale gli angoli dei 6 servo e permette di
 * inviare una posa assoluta via comando UART SETPOSE o SETPOSE_T.
 *
 * Pipeline SETPOSE:
 *   Dashboard slider → sendCommand("uart", {cmd: "SETPOSE B S G Y P R vel PLANNER"})
 *   → ws_server.py → uart_manager.send_uart_command("SETPOSE ...")
 *   → STM32 uart_control.c → j5vr_go_setpose() → desired_positions[]
 *   → j5vr_setpose_tick() (RT loop 1 kHz) — applica direttamente senza velocity cap
 *
 * DESIGN SLIDER:
 *   Lo slider rappresenta il TARGET desiderato dall'utente, NON la posizione
 *   encoder/misurata del robot. La telemetria `servo_deg_*` aggiorna solo la
 *   barra live e il testo "attuale" usando il command state esportato dal
 *   firmware, non una misura assoluta del giunto; il pollice dello slider non
 *   viene mai spostato dalla telemetria.
 *   In questo modo lo slider resta stabile dopo ogni interazione.
 */

import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerSetposeDoneHandler,
  registerUartResponseHandler,
  registerSettingsHandler,
  registerOpenHandler,
  sendCommand,
  addLog,
  loadRoutingConfig,
} from "../../../shared/js/j5_common.js";

// ---------------------------------------------------------------------------
// Configurazione giunti
// key: campo nella telemetria SPI di compatibilitÃ ; i valori live rappresentano
// command state / target software esportato dal firmware.
// ---------------------------------------------------------------------------
const JOINT_LIMIT_DEFAULTS = {
  base:   { min: 10, max: 170 },
  spalla: { min: 10, max: 170 },
  gomito: { min: 10, max: 170 },
  yaw:    { min: 10, max: 170 },
  pitch:  { min: 60, max: 120 },
  roll:   { min: 60, max: 120 },
};

const JOINTS = [
  { key: "servo_deg_B", id: "joint-base",   label: "Base",   limitKey: "base" },
  { key: "servo_deg_S", id: "joint-spalla", label: "Spalla", limitKey: "spalla" },
  { key: "servo_deg_G", id: "joint-gomito", label: "Gomito", limitKey: "gomito" },
  { key: "servo_deg_Y", id: "joint-yaw",    label: "Yaw",    limitKey: "yaw" },
  { key: "servo_deg_P", id: "joint-pitch",  label: "Pitch",  limitKey: "pitch" },
  { key: "servo_deg_R", id: "joint-roll",   label: "Roll",   limitKey: "roll" },
];

// Limiti VIRTUALI applicati agli slider (90 = HOME). Partono dai default
// conservativi e vengono rimpiazzati dalla conversione fisico→virtuale una
// volta ricevuti routing_config (fisico) e settings (offsets+dirs).
//
// NOTA SPAZI ANGOLARI:
//   routing_config.limits → SPAZIO FISICO (quello che il firmware clampa)
//   slider / SETPOSE inviato dal browser → SPAZIO VIRTUALE
//   conversione inversa: virtuale = (fisico - offset) / dir + 90
// Gli slider devono essere nello spazio virtuale coerente col backend,
// altrimenti l'utente raggiunge valori che il backend rifiuta con warning.
let jointLimits = Object.fromEntries(
  Object.entries(JOINT_LIMIT_DEFAULTS).map(([k, v]) => [k, { ...v }]),
);

// Limiti FISICI autorevoli da routing_config.limits. null finché la fetch
// HTTP non completa; mantenuti separati da jointLimits per poter ricomputare
// lo spazio virtuale al cambio di offsets/dirs (settings).
let physicalLimits = null;

// ---------------------------------------------------------------------------
// Stato planner, velocità e modalità invio
// ---------------------------------------------------------------------------
let selectedPlanner = "RTR5"; // default — RTR5 attivo al caricamento
let poseMode        = "speed"; // "speed" | "time"

// Angoli live (dalla telemetria) — usati per stimare Δq nel calcolo derivato
const liveAngles = [90, 90, 90, 90, 90, 90];
// lastSentAngles: angoli dell'ultimo SETPOSE inviato — usati come punto di partenza
// per il calcolo dq, in sincronia con desired_positions[] del firmware.
// Aggiornato ad ogni invio di SETPOSE e confermato da SETPOSE_DONE.
const lastSentAngles = [90, 90, 90, 90, 90, 90];

// ---------------------------------------------------------------------------
// Posa precedente — salvata prima di ogni SETPOSE per consentire undo
// ---------------------------------------------------------------------------
let previousPose = null; // { vals: [b,s,g,y,p,r] } oppure null

function savePreviousPose(vals) {
  previousPose = { vals: [...vals] };
  const btn = document.getElementById("btn-undo-pose");
  if (btn) btn.disabled = false;
}

// ---------------------------------------------------------------------------
// Helper condiviso — costruisce il comando UART corretto in base a poseMode.
// Usato da tutti e tre i pulsanti: Applica, Centra tutto, Posa precedente.
// ---------------------------------------------------------------------------
function getJointLimit(limitKey) {
  const row = jointLimits[limitKey];
  if (
    row &&
    Number.isFinite(row.min) &&
    Number.isFinite(row.max) &&
    row.min >= 0 &&
    row.max <= 180 &&
    row.min < row.max
  ) {
    return row;
  }
  return JOINT_LIMIT_DEFAULTS[limitKey] ? { ...JOINT_LIMIT_DEFAULTS[limitKey] } : { min: 0, max: 180 };
}

function clampToJointLimit(limitKey, value) {
  const { min, max } = getJointLimit(limitKey);
  const n = Number(value);
  if (!Number.isFinite(n)) return Math.round((min + max) / 2);
  return Math.max(min, Math.min(max, Math.round(n)));
}

function clampJointValues(vals) {
  return JOINTS.map(({ limitKey }, i) => clampToJointLimit(limitKey, vals[i]));
}

async function loadPhysicalLimitsFromRoutingConfig() {
  try {
    const cfg = await loadRoutingConfig();
    const limits = cfg?.limits;
    if (!limits || typeof limits !== "object") {
      throw new Error("routing_config senza sezione limits");
    }
    const phys = {};
    let loaded = 0;
    for (const { limitKey } of JOINTS) {
      const row = limits[limitKey];
      if (!row) continue;
      const min = Number(row.min);
      const max = Number(row.max);
      if (Number.isFinite(min) && Number.isFinite(max) && min >= 0 && max <= 180 && min < max) {
        phys[limitKey] = { min, max };
        loaded += 1;
      }
    }
    if (loaded > 0) {
      physicalLimits = phys;
      addLog(`✓ Limiti fisici caricati da routing_config (${loaded}/${JOINTS.length})`);
      return;
    }
    throw new Error("nessun limite valido trovato");
  } catch (e) {
    console.warn("[Joints] fallback limiti UI", e);
    addLog(`⚠ routing_config non disponibile: slider usano fallback virtuale (${String(e)})`);
  }
}

// Converte i limiti fisici (routing_config) in limiti virtuali (slider) usando
// offsets e dirs di j5_settings. Se dir[i] = -1 la relazione lineare inverte
// il segno e min/max si scambiano, quindi calcoliamo entrambi i bound e poi
// riordiniamo.
function applyVirtualLimitsFromSettings(settings) {
  if (!physicalLimits) return;
  const offsets = settings?.offsets;
  const dirsIn  = settings?.dirs;
  if (!Array.isArray(offsets) || offsets.length !== 6) return;
  const dirs = Array.isArray(dirsIn) && dirsIn.length === 6
    ? dirsIn.map((d) => (Number(d) < 0 ? -1 : 1))
    : [1, 1, 1, 1, 1, 1];

  let updated = 0;
  JOINTS.forEach(({ limitKey }, i) => {
    const phys = physicalLimits[limitKey];
    if (!phys) return;
    const off = Number(offsets[i]);
    if (!Number.isFinite(off)) return;
    const dir = dirs[i];
    // virtuale = (fisico - off) / dir + 90
    const a = (phys.min - off) / dir + 90;
    const b = (phys.max - off) / dir + 90;
    let vmin = Math.round(Math.min(a, b));
    let vmax = Math.round(Math.max(a, b));
    // Clamp al range slider valido; il range di sicurezza firmware [5,175]
    // è già garantito dal clamp fisico downstream.
    vmin = Math.max(0, Math.min(180, vmin));
    vmax = Math.max(0, Math.min(180, vmax));
    if (vmin < vmax) {
      jointLimits[limitKey] = { min: vmin, max: vmax };
      updated += 1;
    }
  });
  if (updated > 0) {
    addLog(`✓ Range slider allineato ai limiti firmware (${updated}/${JOINTS.length})`);
    refreshSliderRanges();
    // Log diagnostico: permette di verificare in DevTools console che la fix
    // è quella nuova e quali range sono effettivamente stati applicati.
    try {
      const snap = {};
      JOINTS.forEach(({ limitKey, id }) => {
        const s = document.getElementById(id + "-slider");
        snap[limitKey] = {
          jointLimits: jointLimits[limitKey],
          slider_min: s ? s.min : null,
          slider_max: s ? s.max : null,
        };
      });
      console.log("[JOINTS-FIX v2] applyVirtualLimitsFromSettings →", snap);
    } catch (_) { /* non-fatal */ }
  }
}

// Propaga jointLimits aggiornati sugli slider esistenti (min/max/value e
// etichette min/max) senza ricostruire il DOM. Se lo slider era fuori dal
// nuovo range lo clampa e aggiorna anche il testo del target.
function refreshSliderRanges() {
  JOINTS.forEach(({ id, limitKey }) => {
    const { min, max } = getJointLimit(limitKey);
    const slider = document.getElementById(id + "-slider");
    if (slider) {
      slider.min = String(min);
      slider.max = String(max);
      const cur = parseInt(slider.value, 10);
      if (Number.isFinite(cur)) {
        const clamped = Math.max(min, Math.min(max, cur));
        if (clamped !== cur) {
          slider.value = String(clamped);
          const targetEl = document.getElementById(id + "-target");
          if (targetEl) targetEl.textContent = clamped + "°";
        }
      }
    }
    const card = slider?.closest(".joint-card");
    const minLabel = card?.querySelector(".joint-min");
    const maxLabel = card?.querySelector(".joint-max");
    if (minLabel) minLabel.textContent = `${min}°`;
    if (maxLabel) maxLabel.textContent = `${max}°`;
  });
}

function buildPoseCmd(vals) {
  const [b, s, g, y, p, r] = clampJointValues(vals);
  if (poseMode === "time") {
    const timeSlider = document.getElementById("time-slider");
    const T_s    = timeSlider ? parseFloat(timeSlider.value) : 1.0;
    const time_ms = Math.max(200, Math.min(3000, Math.round(T_s * 1000)));
    return `SETPOSE_T ${b} ${s} ${g} ${y} ${p} ${r} ${time_ms} ${selectedPlanner}`;
  } else {
    const speedSlider = document.getElementById("speed-slider");
    const vel = speedSlider ? Math.max(20, Math.min(120, parseInt(speedSlider.value, 10) || 60)) : 60;
    return `SETPOSE ${b} ${s} ${g} ${y} ${p} ${r} ${vel} ${selectedPlanner}`;
  }
}

// ---------------------------------------------------------------------------
// Calcolo quantità derivate (T stimato in mode=speed, vel stimata in mode=time)
// ---------------------------------------------------------------------------
function getCurrentSliderValues() {
  return JOINTS.map(({ id, limitKey }) => {
    const el = document.getElementById(id + "-slider");
    const clamped = clampToJointLimit(limitKey, el ? parseInt(el.value, 10) : 90);
    if (el && parseInt(el.value, 10) !== clamped) {
      el.value = String(clamped);
    }
    return clamped;
  });
}

function updateDerivedValues() {
  const vals = getCurrentSliderValues();
  let dq = 0;
  for (let i = 0; i < 6; i++) {
    // Usa lastSentAngles: stessa sorgente di desired_positions[] nel firmware.
    // liveAngles (SPI) è ritardata durante il moto e causerebbe stime imprecise.
    dq = Math.max(dq, Math.abs(vals[i] - lastSentAngles[i]));
  }

  const speedDerived = document.getElementById("speed-derived");
  if (speedDerived) speedDerived.textContent = "";
  const timeDerived = document.getElementById("time-derived");
  if (timeDerived) timeDerived.textContent = "";
}

// ---------------------------------------------------------------------------
// Colore barra in base all'angolo
// ---------------------------------------------------------------------------
function angleColor(deg) {
  const dist = Math.abs(deg - 90) / 90;
  if (dist < 0.4)  return "var(--success, #4caf50)";
  if (dist < 0.75) return "var(--warning, #ff9800)";
  return "var(--danger, #ff5161)";
}

// ---------------------------------------------------------------------------
// Aggiorna SOLO la barra live e il testo "attuale" ricevuto dalla telemetria.
// Lo slider NON viene mai toccato dalla telemetria: è un controllo utente.
// ---------------------------------------------------------------------------
function updateJointCard(id, val, limitKey) {
  const liveEl = document.getElementById(id + "-live");
  if (liveEl) liveEl.textContent = val + "°";

  const bar = document.getElementById(id + "-bar");
  if (bar) {
    const { min, max } = getJointLimit(limitKey);
    const span = Math.max(1, max - min);
    const clamped = Math.max(min, Math.min(max, val));
    bar.style.width = Math.round(((clamped - min) / span) * 100) + "%";
    bar.style.background = angleColor(val);
  }
}

// ---------------------------------------------------------------------------
// Telemetry handler — aggiorna solo la visualizzazione live, mai gli slider
// ---------------------------------------------------------------------------
function onTelemetry(data) {
  let anyUpdate = false;
  JOINTS.forEach(({ key, id, limitKey }, i) => {
    const val = data[key];
    if (val !== undefined) {
      liveAngles[i] = Math.round(val);
      updateJointCard(id, liveAngles[i], limitKey);
      anyUpdate = true;
    }
  });
  if (anyUpdate) updateDerivedValues();
}

// ---------------------------------------------------------------------------
// Costruzione DOM delle card giunti
// ---------------------------------------------------------------------------
function buildUI() {
  const container = document.getElementById("joints-container");
  if (!container) return;
  container.innerHTML = "";

  JOINTS.forEach(({ id, label, limitKey }) => {
    const { min, max } = getJointLimit(limitKey);
    const startValue = clampToJointLimit(limitKey, 90);
    const card = document.createElement("div");
    card.className = "joint-card";
    card.innerHTML = `
      <div class="joint-header">
        <span class="joint-label">${label}</span>
        <span class="joint-value" id="${id}-target">${startValue}°</span>
      </div>
      <div class="joint-bar-bg">
        <div class="joint-bar" id="${id}-bar" style="width:50%;background:var(--success)"></div>
      </div>
      <div class="joint-slider-row">
        <span class="joint-min">${min}°</span>
        <input
          type="range"
          id="${id}-slider"
          class="joint-slider"
          min="${min}"
          max="${max}"
          value="${startValue}"
          step="1"
        />
        <span class="joint-max">${max}°</span>
      </div>
      <div class="joint-live-row">
        <span class="joint-live-label">Attuale:</span>
        <span class="joint-live-val" id="${id}-live">–°</span>
      </div>
    `;
    container.appendChild(card);

    const slider    = document.getElementById(id + "-slider");
    const targetEl  = document.getElementById(id + "-target");

    // Lo slider aggiorna il display del target e ricalcola i valori derivati
    slider.addEventListener("input", () => {
      const v = clampToJointLimit(limitKey, parseInt(slider.value, 10));
      slider.value = String(v);
      if (targetEl) targetEl.textContent = v + "°";
      updateDerivedValues();
    });
  });

}

// ---------------------------------------------------------------------------
// Inizializzazione toggle modalità Velocità / Tempo
// ---------------------------------------------------------------------------
function initModeToggle() {
  document.querySelectorAll('input[name="pose-mode"]').forEach(r => {
    r.addEventListener("change", e => {
      poseMode = e.target.value;
      const speedEl = document.getElementById("speed-mode");
      const timeEl  = document.getElementById("time-mode");
      if (speedEl) speedEl.style.display = poseMode === "speed" ? "" : "none";
      if (timeEl)  timeEl.style.display  = poseMode === "time"  ? "" : "none";
      updateDerivedValues();
    });
  });
}

// ---------------------------------------------------------------------------
// Inizializzazione slider velocità
function initSpeedSlider() {
  const speedSlider = document.getElementById("speed-slider");
  const speedValue  = document.getElementById("speed-value");
  if (!speedSlider || !speedValue) return;

  const updateSpeed = () => {
    const vel = parseInt(speedSlider.value, 10);
    speedValue.textContent = `${vel} °/s`;
    updateDerivedValues();
  };
  updateSpeed(); // valore iniziale
  speedSlider.addEventListener("input", updateSpeed);
}

// ---------------------------------------------------------------------------
// Inizializzazione slider tempo
// ---------------------------------------------------------------------------
function initTimeSlider() {
  const timeSlider = document.getElementById("time-slider");
  const timeValue  = document.getElementById("time-value");
  if (!timeSlider || !timeValue) return;

  const update = () => {
    timeValue.textContent = `${parseFloat(timeSlider.value).toFixed(1)} s`;
    updateDerivedValues();
  };
  update();
  timeSlider.addEventListener("input", update);
}

// ---------------------------------------------------------------------------
// Inizializzazione pulsanti planner (toggle group mutuamente esclusivo)
// ---------------------------------------------------------------------------
function initPlannerButtons() {
  document.querySelectorAll(".planner-btn").forEach(btn => {
    btn.addEventListener("click", () => {
      // Rimuovi active da tutti
      document.querySelectorAll(".planner-btn").forEach(b => b.classList.remove("active"));
      // Attiva il selezionato
      btn.classList.add("active");
      // Il dataset.planner contiene il valore esatto per il firmware (RTR3/RTR5/BB/BCB)
      selectedPlanner = btn.dataset.planner || "RTR5";
      addLog(`Profilo selezionato: ${selectedPlanner}`);
    });
  });
}

// ---------------------------------------------------------------------------
// Pulsante "Applica pose" — costruisce e invia SETPOSE o SETPOSE_T
// ---------------------------------------------------------------------------
function initApplyPoseButton() {
  const applyBtn = document.getElementById("apply-pose-btn");
  if (!applyBtn) return;

  applyBtn.addEventListener("click", () => {
    const vals = getCurrentSliderValues();
    savePreviousPose(vals);
    const cmd = buildPoseCmd(vals);
    console.log("CMD INVIATO (Applica):", cmd);
    if (!sendCommand("uart", { cmd })) {
      addLog("✗ SETPOSE non inviato (WS non connesso)");
      return;
    }
    vals.forEach((v, i) => { lastSentAngles[i] = v; });
    updateDerivedValues();
    addLog(`… SETPOSE inviato (pending conferma): ${cmd}`);
  });
}

// ---------------------------------------------------------------------------
// Pulsante globale "Centra tutto" — invia SETPOSE/SETPOSE_T 90x6 con parametri correnti
// ---------------------------------------------------------------------------
function initCenterAllButton() {
  const btn = document.getElementById("btn-center-all");
  if (!btn) return;

  btn.addEventListener("click", () => {
    // Salva la posa corrente degli slider PRIMA di azzerarli (per undo)
    const currentVals = JOINTS.map(({ id }) => {
      const sl = document.getElementById(id + "-slider");
      return sl ? parseInt(sl.value, 10) : 90;
    });
    savePreviousPose(currentVals);

    // Porta tutti gli slider a 90° e aggiorna il display del target
    JOINTS.forEach(({ id, limitKey }) => {
      const sl = document.getElementById(id + "-slider");
      const tg = document.getElementById(id + "-target");
      const center = clampToJointLimit(limitKey, 90);
      if (sl) sl.value = center;
      if (tg) tg.textContent = center + "°";
    });

    // Usa buildPoseCmd con 90° fissi — rispetta poseMode corrente
    const centerVals = JOINTS.map(({ limitKey }) => clampToJointLimit(limitKey, 90));
    const cmd = buildPoseCmd(centerVals);
    console.log("CMD INVIATO (Centra tutto):", cmd);
    if (!sendCommand("uart", { cmd })) {
      addLog("✗ Centra tutto non inviato (WS non connesso)");
      return;
    }
    centerVals.forEach((v, i) => { lastSentAngles[i] = v; });
    updateDerivedValues();
    addLog(`… Centra tutto inviato (pending conferma): ${cmd}`);
  });
}

// ---------------------------------------------------------------------------
// Pulsante "Posa precedente" — ripristina l'ultima posa inviata (undo)
// ---------------------------------------------------------------------------
function initUndoPoseButton() {
  const btn = document.getElementById("btn-undo-pose");
  if (!btn) return;

  btn.disabled = true; // disabilitato finché non esiste una posa precedente

  btn.addEventListener("click", () => {
    if (!previousPose) return;
    const { vals } = previousPose;

    // Ripristina solo gli slider dei giunti (angoli precedenti)
    JOINTS.forEach(({ id, limitKey }, i) => {
      const sl = document.getElementById(id + "-slider");
      const tg = document.getElementById(id + "-target");
      const clamped = clampToJointLimit(limitKey, vals[i]);
      if (sl) sl.value = clamped;
      if (tg) tg.textContent = clamped + "°";
    });

    // Usa poseMode, velocità e profilo CORRENTEMENTE selezionati (non quelli salvati)
    // così si può testare la stessa posa con parametri diversi
    const cmd = buildPoseCmd(vals);
    console.log("CMD INVIATO (Posa precedente):", cmd);
    if (!sendCommand("uart", { cmd })) {
      addLog("✗ Posa precedente non inviata (WS non connesso)");
      return;
    }
    vals.forEach((v, i) => { lastSentAngles[i] = v; });
    updateDerivedValues();
    addLog(`… Posa precedente inviata (pending conferma): ${cmd}`);

    // Non azzera previousPose: permette di ripetere la stessa posa più volte
    // cambiando ogni volta modalità/velocità/profilo per confrontare la reattività
  });
}

// ---------------------------------------------------------------------------
// Handler SETPOSE_DONE — popola il box telemetria
// ---------------------------------------------------------------------------
function initSetposeDoneHandler() {
  registerSetposeDoneHandler((msg) => {
    const box = document.getElementById("setpose-telemetry-box");
    if (!box) return;

    const timeEl = document.getElementById("tel-time");
    const velEl  = document.getElementById("tel-vel");
    const accEl  = document.getElementById("tel-acc");

    const timeSec = (msg.time_ms / 1000).toFixed(2);
    if (timeEl) timeEl.textContent = `${timeSec} s`;
    if (velEl)  velEl.textContent  = `${msg.vel_max.toFixed(1)} °/s`;
    if (accEl)  accEl.textContent  = `${msg.acc_max.toFixed(0)} °/s²`;

    box.classList.add("visible");
    addLog(`✓ SETPOSE completato: ${timeSec}s | vel_max=${msg.vel_max.toFixed(1)}°/s | acc_max=${msg.acc_max.toFixed(0)}°/s²`);
    // Sincronizza lastSentAngles con la posizione live (robot fermo a fine SETPOSE)
    liveAngles.forEach((v, i) => { lastSentAngles[i] = v; });
    updateDerivedValues();
  });
}

// ---------------------------------------------------------------------------
// Banner informativo sulle limitazioni attuali
// ---------------------------------------------------------------------------
function showMoveNote() {
  const note = document.getElementById("joints-move-note");
  if (note) note.style.display = "none";
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
async function init() {
  console.log("[JOINTS-FIX v2] joints.js loaded — virtual slider limits derived from routing_config + settings");
  await loadPhysicalLimitsFromRoutingConfig();
  buildUI();
  initModeToggle();
  initSpeedSlider();
  initTimeSlider();
  initPlannerButtons();
  initApplyPoseButton();
  initUndoPoseButton();
  initCenterAllButton();
  initSetposeDoneHandler();
  // Settings (offsets, dirs) necessari per convertire i limiti fisici in
  // limiti virtuali coerenti con lo slider. Ci interessa SOLO la risposta
  // type="settings" (full state); "settings_saved"/"offsets_applied" non
  // trasportano entrambi offsets+dirs, quindi per quei broadcast richiediamo
  // un refresh esplicito.
  registerSettingsHandler((msg) => {
    if (!msg) return;
    if (msg.type === "settings") {
      applyVirtualLimitsFromSettings(msg);
    } else if (msg.type === "settings_saved" || msg.type === "offsets_applied") {
      sendCommand("get_settings");
    }
  });
  registerUartResponseHandler((msg) => {
    if (msg?.type === "uart_response" && msg?.warning) {
      addLog(`⚠ ${msg.warning}`);
      alert(msg.warning);
    }
  });
  showMoveNote();
  registerTelemetryHandler(onTelemetry);
  registerOpenHandler(() => {
    sendCommand("get_settings");
  });
  connectJ5Dashboard();
  addLog("Pagina Joints caricata — SETPOSE attivo");
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", () => { void init(); });
} else {
  void init();
}
