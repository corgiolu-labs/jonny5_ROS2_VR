/**
 * imu_vr.js — Logica pagina Settings VR JONNY5
 *
 * Aree gestite:
 *   1. Mapping visore -> polso robot
 *   2. Mapping controller dx/sx -> braccio robot
 *   3. Velocita operative persistite via routing_config + SET_VR_PARAMS
 */

import {
  registerOpenHandler,
  registerTelemetryHandler,
  registerVrConfigAppliedHandler,
  quatToEuler,
  sendCommand,
  s,
  flashButton,
  loadRoutingConfig,
  saveRoutingConfig,
} from "../../shared/js/j5_common.js";

function setStatus(id, msg, type = "") {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = msg;
  el.className = "send-status" + (type ? " " + type : "");
}

function setGlobalStatus(msg, cls = "") {
  const el = document.getElementById("global-status");
  if (!el) return;
  el.textContent = msg;
  el.className = "status-bar" + (cls ? " " + cls : "");
}

/** Valori mostrati negli slider (file / default) — non implica firmware aggiornato */
function setVrUiLoadedStatus(text, kind = "") {
  const el = document.getElementById("vr-ui-loaded-status");
  if (!el) return;
  el.textContent = text;
  el.className = "vr-sync-line" + (kind ? " " + kind : "");
}

/** Stato applicazione reale sullo STM32 (via apply_saved_vr_config / ack server) */
function setVrRobotApplyStatus(ok) {
  const el = document.getElementById("vr-robot-apply-status");
  if (!el) return;
  if (ok === null || ok === undefined) {
    el.textContent = "Robot: in attesa — WebSocket / backend non pronti o applicazione in corso…";
    el.className = "vr-sync-line waiting";
    return;
  }
  if (ok) {
    el.textContent = "Robot: parametri VR/IMU applicati (SET_VR_PARAMS accettato da STM32)";
    el.className = "vr-sync-line ok";
  } else {
    el.textContent = "Robot: applicazione non riuscita (UART/ACK/file routing_config assente o vuoto)";
    el.className = "vr-sync-line err";
  }
}

let _vrPendingResolve = null;

function waitVrConfigApplied(timeoutMs = 10000) {
  return new Promise((resolve) => {
    const t = setTimeout(() => {
      if (_vrPendingResolve) {
        _vrPendingResolve = null;
      }
      resolve(false);
    }, timeoutMs);
    _vrPendingResolve = (v) => {
      clearTimeout(t);
      _vrPendingResolve = null;
      resolve(v);
    };
  });
}

function onVrConfigAppliedMessage(msg) {
  if (_vrPendingResolve) {
    _vrPendingResolve(!!msg.ok);
  }
  setVrRobotApplyStatus(msg.ok);
}

// ─── Sezione 1: aggiornamento tabella matching live ──────────────────────────

// ── Patch bay routing ────────────────────────────────────────────────────────
//
// Stato patch bay: per ogni servo robot (riga), quale src visore (colonna) è collegato
// src: 1=visore PITCH, 0=visore YAW, 2=visore ROLL
// sign: +1 concordi (verde), -1 invertiti (rosso)
// enable per servo: _pbEn[servo] = true/false (toggle ON/OFF nella riga)

// Configurazione di default: corrispondenza FISICA reale asse-per-asse.
// Il firmware NON incorpora permutazioni del polso: ogni cella indica quale
// asse del visore guida davvero il servo, così la tabella robot<->visore non
// e' "cosmetica" (diagonale) ma rispecchia il movimento reale:
// ROLL(robot)  <- YAW(visore)   [sign -1: ROLL in opposizione a YAW visore]
// PITCH(robot) <- ROLL(visore)
// YAW(robot)   <- PITCH(visore)
let _pbStateDefault = {
  roll:  { src: 0, sign: -1 },  // ROLL  ← YAW visore (invertito)
  pitch: { src: 2, sign: 1 },   // PITCH ← ROLL visore
  yaw:   { src: 1, sign: 1 },   // YAW   ← PITCH visore
};
let _pbEnDefault = { roll: true, pitch: true, yaw: true };

const _pbState = {
  roll:  { ..._pbStateDefault.roll  },
  pitch: { ..._pbStateDefault.pitch },
  yaw:   { ..._pbStateDefault.yaw   },
};
// Enable per servo robot (riga): true = servo attivo
const _pbEn = { ..._pbEnDefault };
let _limitsCache = {
  base: { min: 10, max: 170 },
  spalla: { min: 10, max: 170 },
  gomito: { min: 10, max: 170 },
  yaw: { min: 10, max: 170 },
  pitch: { min: 60, max: 120 },
  roll: { min: 60, max: 120 },
};
const OPERATIONAL_TUNE_IDS = [
  "tg-vel-base",
  "tg-vel-spalla",
  "tg-vel-gomito",
  "tg-vel-yaw",
  "tg-vel-pitch",
  "tg-vel-roll",
  "tg-vel-yaw-head",
  "tg-vel-pitch-head",
  "tg-vel-roll-head",
  "tg-vel-base-head",
  "tg-vel-spalla-head",
  "tg-vel-gomito-head",
];

const _controllerMappingsTemplate = () => ({
  right: {
    state: {
      base: { src: 0, sign: 1 },
      spalla: { src: 1, sign: 1 },
      gomito: { src: 2, sign: 1 },
    },
    en: { base: true, spalla: true, gomito: true },
  },
  left: {
    state: {
      base: { src: 0, sign: 1 },
      spalla: { src: 1, sign: 1 },
      gomito: { src: 2, sign: 1 },
    },
    en: { base: true, spalla: true, gomito: true },
  },
});
/** Persistenza mapping controller (nessuna UI dedicata): ultimo file o default. */
let _controllerMappingsPersisted = _controllerMappingsTemplate();

let _armControlDefaults = {
  preferredController: "right",
  enabledControllers: { right: true, left: true },
};
let _armControlState = { ..._armControlDefaults, enabledControllers: { ..._armControlDefaults.enabledControllers } };

let _headAssistDefaults = {
  enabled: true,
  yaw: { warnDeg: 22, critDeg: 9 },
  pitch: { warnDeg: 20, critDeg: 8 },
  roll: { warnDeg: 10, critDeg: 4 },
  assistEnable: { yaw: true, pitch: true, roll: false },
  signYaw: 1,
  signPitch: 1,
  signRoll: 1,
  gainBase: 0.48,
  gainSpalla: 0.36,
  gainGomito: 0.30,
  gainRollArm: 0.1,
  critGainMul: 1.75,
  pitchSplit: { spalla: 0.45, gomito: 0.55 },
  rollSplit: { spalla: 0.5, gomito: 0.5 },
  assistAlpha: 0.72,
  freeFollowAlpha: 0.16,
  maxStepDegPerTick: 4.0,
  reliefDeadband: 0.015,
};

// DLS (position-based ASSIST) defaults — see head_assist_dls.py.
// assistMode="rate" keeps the historical path; "dls" activates the new one.
// Values below were validated on the live robot (tuning campaign
// 2026-04-21, see ai/reports/DLS_ASSIST_TUNING_SUMMARY_*.md).
// manipThresh=1e-3 is load-bearing: lowering it reintroduces the
// damping-unlock pitch_down excursion at gainM>=0.12.
let _assistModeDefault = "rate";
let _assistDlsDefaults = {
  gainM: 0.15,
  lambdaMax: 0.12,
  manipThresh: 1e-3,
  maxDqDegPerTick: 2.0,
  maxDxMmPerTick: 15.0,
  nullSpaceGain: 0.20,
};
const _assistModeFactory = _assistModeDefault;
const _assistDlsFactory = JSON.parse(JSON.stringify(_assistDlsDefaults));
let _headAssistFactoryDefaults = JSON.parse(JSON.stringify(_headAssistDefaults));

// Etichette brevi per ogni src (per testo nelle celle patch bay visore)
const SRC_LABEL = { 0: "YAW", 1: "PITCH", 2: "ROLL" };
const SRC_ICON  = { 0: "Y", 1: "P", 2: "R" };

function _applyHeadAssistDefaultsToUi() {
  const d = _headAssistDefaults;
  const setCh = (id, v) => {
    const el = document.getElementById(id);
    if (el) el.checked = !!v;
  };
  const setRg = (id, v, disp) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.value = v;
    if (disp) {
      const dec = (el.step || "1").includes(".") ? el.step.split(".")[1].length : 0;
      s(disp, Number(v).toFixed(dec));
    }
  };
  const setSel = (id, v) => {
    const el = document.getElementById(id);
    if (el) el.value = String(v);
  };

  setCh("ha-enabled", d.enabled);
  setCh("ha-en-yaw", d.assistEnable.yaw);
  setCh("ha-en-pitch", d.assistEnable.pitch);
  setRg("ha-yaw-warn", d.yaw.warnDeg, "tv-ha-yaw-warn");
  setRg("ha-pitch-warn", d.pitch.warnDeg, "tv-ha-pitch-warn");
  setRg("ha-gain-base", d.gainBase, "tv-ha-gain-base");
  setRg("ha-gain-spalla", d.gainSpalla, "tv-ha-gain-spalla");
  setRg("ha-gain-gomito", d.gainGomito, "tv-ha-gain-gomito");
  setRg("ha-assist-alpha", d.assistAlpha, "tv-ha-assist-alpha");
  setRg("ha-max-step", d.maxStepDegPerTick, "tv-ha-max-step");
  setSel("ha-sign-yaw", d.signYaw);
  setSel("ha-sign-pitch", d.signPitch);

  // DLS block
  const dls = _assistDlsDefaults;
  setSel("ha-mode", _assistModeDefault);
  setRg("hdls-gain-m",       dls.gainM,             "tv-hdls-gain-m");
  setRg("hdls-null-gain",    dls.nullSpaceGain,     "tv-hdls-null-gain");
  setRg("hdls-lambda-max",   dls.lambdaMax,         "tv-hdls-lambda-max");
  setRg("hdls-manip-thresh", Math.round(dls.manipThresh * 1e4), "tv-hdls-manip-thresh");
  setRg("hdls-max-dq",       dls.maxDqDegPerTick,   "tv-hdls-max-dq");
  setRg("hdls-max-dx",       dls.maxDxMmPerTick,    "tv-hdls-max-dx");
}

function _pbServoenable(servo) {
  return _pbEn[servo] ? 1 : 0;
}

function _pbRenderCell(servo, src) {
  const btn = document.getElementById(`pb-${servo}-${src}`);
  if (!btn) return;
  const st     = _pbState[servo];
  const active = (st.src === src);
  const inv    = active && st.sign === -1;
  const servoOff = !_pbEn[servo];

  btn.className = "pb-btn" +
    (active ? (inv ? " pb-active pb-inv" : " pb-active") : "") +
    (servoOff ? " pb-disabled" : "");

  if (active) {
    btn.innerHTML = `
      <span class="pb-icon">${SRC_ICON[src]}</span>
      <span class="pb-label" style="color:${inv ? "#ff8080" : "#2ecc71"}">${inv ? "✕ INVERTITO" : "✓ CONCORDI"}</span>
      <span class="pb-sign">${inv ? "clic → concordi" : "clic → inverti"}</span>`;
  } else {
    btn.innerHTML = `
      <span class="pb-icon" style="opacity:.3">${SRC_ICON[src]}</span>
      <span class="pb-label" style="opacity:.35">${SRC_LABEL[src]}</span>
      <span class="pb-sign" style="opacity:.3">clic per collegare</span>`;
  }
}

function _pbRenderAll() {
  ["roll", "pitch", "yaw"].forEach(servo => {
    [0, 1, 2].forEach(src => _pbRenderCell(servo, src));
  });
  // Sincronizza hidden inputs per getRouting()
  const setHidden = (id, val) => { const el = document.getElementById(id); if (el) el.value = val; };
  setHidden("rt-src-roll",  _pbState.roll.src);
  setHidden("rt-src-pitch", _pbState.pitch.src);
  setHidden("rt-src-yaw",   _pbState.yaw.src);
  setHidden("rt-en-roll",   _pbServoenable("roll"));
  setHidden("rt-en-pitch",  _pbServoenable("pitch"));
  setHidden("rt-en-yaw",    _pbServoenable("yaw"));
}

/* Legge dalla UI lo stato routing completo */
function getRouting() {
  return {
    srcRoll:  _pbState.roll.src,
    srcPitch: _pbState.pitch.src,
    srcYaw:   _pbState.yaw.src,
    enRoll:   _pbServoenable("roll"),
    enPitch:  _pbServoenable("pitch"),
    enYaw:    _pbServoenable("yaw"),
  };
}

/*
 * _signForSrc(srcIdx): restituisce il segno da applicare al SENSORE srcIdx.
 * Il firmware applica sign_yaw/pitch/roll all'errore del SENSORE (indice 0/1/2),
 * non al servo di destinazione. Quindi cerchiamo quale servo ha srcIdx come sorgente
 * attiva e ne leggiamo il sign. Se nessuno usa quella sorgente, default -1.
 */
function _signForSrc(srcIdx) {
  for (const servo of ["roll", "pitch", "yaw"]) {
    if (_pbState[servo].src === srcIdx) return _pbState[servo].sign;
  }
  return -1;
}

/*
 * Helper legacy di apply diretto.
 * Il flow principale della UI usa persistenza su routing_config.json e poi
 * apply_saved_vr_config dal backend; questo builder locale resta solo per
 * compatibilità/debug e usa gli stessi default UI correnti.
 */
function sendFullVrParams(statusEl) {
  const params = [
    2.0, 3.0, 1.0,
    0.05, 0.35, 3.0,
    60, 60, 35,
    1.0, 1.0, 0.10,
    1.0,
  ];
  // Sensibilità (amplificazione movimento testa): 13° float
  // sign_yaw / sign_pitch / sign_roll
  const signSrc0 = _signForSrc(0);
  const signSrc1 = _signForSrc(1);
  const signSrc2 = _signForSrc(2);
  params.push(signSrc0, signSrc1, signSrc2);
  // src_roll, src_pitch, src_yaw, en_roll, en_pitch, en_yaw
  const r = getRouting();
  params.push(r.srcRoll, r.srcPitch, r.srcYaw, r.enRoll, r.enPitch, r.enYaw);
  // Velocita' opzionali per BASE/SPALLA/GOMITO (0 = usa vel globale)
  const velBase = parseInt(document.getElementById("tg-vel-base")?.value ?? "0", 10) || 0;
  const velSpalla = parseInt(document.getElementById("tg-vel-spalla")?.value ?? "0", 10) || 0;
  const velGomito = parseInt(document.getElementById("tg-vel-gomito")?.value ?? "0", 10) || 0;
  params.push(velBase, velSpalla, velGomito);
  // Velocita' separata polso Manual VR (0 = usa Vel max PITCH/ROLL)
  const velYaw = parseInt(document.getElementById("tg-vel-yaw")?.value ?? "0", 10) || 0;
  const velPitch = parseInt(document.getElementById("tg-vel-pitch")?.value ?? "0", 10) || 0;
  const velRoll = parseInt(document.getElementById("tg-vel-roll")?.value ?? "0", 10) || 0;
  params.push(velYaw, velPitch, velRoll);
  // Velocita' separata polso HEAD/HYBRID (0 = usa Vel max globale)
  const velYawHead = parseInt(document.getElementById("tg-vel-yaw-head")?.value ?? "0", 10) || 0;
  const velPitchHead = parseInt(document.getElementById("tg-vel-pitch-head")?.value ?? "0", 10) || 0;
  const velRollHead = parseInt(document.getElementById("tg-vel-roll-head")?.value ?? "0", 10) || 0;
  // Velocita' separata braccio HEAD/HYBRID (0 = usa cap manuale/globale)
  const velBaseHead = parseInt(document.getElementById("tg-vel-base-head")?.value ?? "0", 10) || 0;
  const velSpallaHead = parseInt(document.getElementById("tg-vel-spalla-head")?.value ?? "0", 10) || 0;
  const velGomitoHead = parseInt(document.getElementById("tg-vel-gomito-head")?.value ?? "0", 10) || 0;
  params.push(velYawHead, velPitchHead, velRollHead, velBaseHead, velSpallaHead, velGomitoHead);
  const cmd = `SET_VR_PARAMS ${params.join(" ")}`;
  const sent = sendCommand("uart", { cmd });
  if (!sent) {
    if (statusEl) statusEl.textContent = "Errore: WebSocket non connesso — comando non inviato.";
    setGlobalStatus("WS non pronto — SET_VR_PARAMS non inviato", "err");
    return false;
  }
  const srcName = (idx) => (idx === 0 ? "YAW" : idx === 1 ? "PITCH" : "ROLL");
  const msg = `Routing robot: ROLL←${srcName(r.srcRoll)} PITCH←${srcName(r.srcPitch)} YAW←${srcName(r.srcYaw)} | en:R${r.enRoll}P${r.enPitch}Y${r.enYaw}`;
  if (statusEl) { statusEl.textContent = msg + " (in attesa ack UART…)"; }
  setGlobalStatus("SET_VR_PARAMS inviato — attendere ack firmware", "ok");
  return true;
}

// Click su cella patch bay
document.querySelectorAll(".pb-btn[data-servo]").forEach(btn => {
  btn.addEventListener("click", () => {
    const servo = btn.dataset.servo;
    const src   = parseInt(btn.dataset.src);
    const st    = _pbState[servo];
    if (st.src === src) {
      // già collegato: toggle segno
      st.sign *= -1;
    } else {
      // nuova connessione: disconnette la precedente, collega questa concordi
      st.src  = src;
      st.sign = 1;
    }
    _pbRenderAll();
    const statusEl = document.getElementById("routing-status");
    if (statusEl) statusEl.textContent = "Routing aggiornato (non ancora salvato).";
  });
});

// Toggle enable per servo (riga)
["roll", "pitch", "yaw"].forEach(servo => {
  document.getElementById(`rt-en-${servo}`)?.addEventListener("change", e => {
    _pbEn[servo] = e.target.checked;
    _pbRenderAll();
    const statusEl = document.getElementById("routing-status");
    if (statusEl) statusEl.textContent = "Abilitazione aggiornata (non ancora salvata).";
  });
});

document.getElementById("btn-reset-routing")?.addEventListener("click", () => {
  // Ripristina i default correnti (backend autorevole se già caricati).
  _resetPatchBayToDefaults();
  const statusEl = document.getElementById("routing-status");
  if (statusEl) statusEl.textContent = "Default routing ripristinati (non ancora salvati).";
});

// ─── Salva / carica configurazione routing su Pi ─────────────────────────────

function _buildFullPageConfigObject() {
  const cfg = {
    pbState: {
      roll:  { src: _pbState.roll.src,  sign: _pbState.roll.sign  },
      pitch: { src: _pbState.pitch.src, sign: _pbState.pitch.sign },
      yaw:   { src: _pbState.yaw.src,   sign: _pbState.yaw.sign   },
    },
    pbEn: { roll: _pbEn.roll, pitch: _pbEn.pitch, yaw: _pbEn.yaw },
    limits: {},
    tuning: {},
    controllerMappings: JSON.parse(JSON.stringify(_controllerMappingsPersisted)),
    armControl: {
      preferredController: _armControlState.preferredController === "left" ? "left" : "right",
      enabledControllers: {
        right: !!_armControlState.enabledControllers.right,
        left: !!_armControlState.enabledControllers.left,
      },
    },
    headAssist: {
      enabled: !!document.getElementById("ha-enabled")?.checked,
      yaw: {
        warnDeg: parseFloat(document.getElementById("ha-yaw-warn")?.value ?? "22"),
        critDeg: _headAssistDefaults.yaw.critDeg,
      },
      pitch: {
        warnDeg: parseFloat(document.getElementById("ha-pitch-warn")?.value ?? "20"),
        critDeg: _headAssistDefaults.pitch.critDeg,
      },
      roll: { ..._headAssistDefaults.roll },
      assistEnable: {
        yaw: !!document.getElementById("ha-en-yaw")?.checked,
        pitch: !!document.getElementById("ha-en-pitch")?.checked,
        roll: _headAssistDefaults.assistEnable.roll,
      },
      signYaw: parseInt(document.getElementById("ha-sign-yaw")?.value ?? "1", 10),
      signPitch: parseInt(document.getElementById("ha-sign-pitch")?.value ?? "1", 10),
      signRoll: _headAssistDefaults.signRoll,
      gainBase: parseFloat(document.getElementById("ha-gain-base")?.value ?? "0.48"),
      gainSpalla: parseFloat(document.getElementById("ha-gain-spalla")?.value ?? "0.36"),
      gainGomito: parseFloat(document.getElementById("ha-gain-gomito")?.value ?? "0.30"),
      gainRollArm: _headAssistDefaults.gainRollArm,
      critGainMul: _headAssistDefaults.critGainMul,
      pitchSplit: { ..._headAssistDefaults.pitchSplit },
      rollSplit: { ..._headAssistDefaults.rollSplit },
      assistAlpha: parseFloat(document.getElementById("ha-assist-alpha")?.value ?? "0.72"),
      freeFollowAlpha: _headAssistDefaults.freeFollowAlpha,
      maxStepDegPerTick: parseFloat(document.getElementById("ha-max-step")?.value ?? "4.0"),
      reliefDeadband: _headAssistDefaults.reliefDeadband,
    },
    assistMode: (document.getElementById("ha-mode")?.value === "dls") ? "dls" : "rate",
    assistDls: {
      gainM:           parseFloat(document.getElementById("hdls-gain-m")?.value       ?? "0.15"),
      lambdaMax:       parseFloat(document.getElementById("hdls-lambda-max")?.value   ?? "0.12"),
      manipThresh:     parseFloat(document.getElementById("hdls-manip-thresh")?.value ?? "10") * 1e-4,
      maxDqDegPerTick: parseFloat(document.getElementById("hdls-max-dq")?.value       ?? "2.0"),
      maxDxMmPerTick:  parseFloat(document.getElementById("hdls-max-dx")?.value       ?? "15"),
      nullSpaceGain:   parseFloat(document.getElementById("hdls-null-gain")?.value    ?? "0.20"),
    },
    savedAt: new Date().toISOString(),
  };
  const joints = ["base", "spalla", "gomito", "yaw", "pitch", "roll"];
  joints.forEach(j => {
    const minEl = document.getElementById(`lim-${j}-min`);
    const maxEl = document.getElementById(`lim-${j}-max`);
    cfg.limits[j] = {
      min: parseInt(minEl?.value ?? _limitsCache[j]?.min ?? 10, 10),
      max: parseInt(maxEl?.value ?? _limitsCache[j]?.max ?? 170, 10),
    };
  });
  OPERATIONAL_TUNE_IDS.forEach(id => {
    cfg.tuning[id] = parseFloat(document.getElementById(id)?.value ?? TUNE_DEFAULTS[id]);
  });
  return cfg;
}

function _pbApplyConfigObject(cfg) {
  if (!cfg || !cfg.pbState || !cfg.pbEn) return false;
  // Rispetta SEMPRE il mapping salvato dall'utente.
  const pbStateCfg = cfg.pbState;

  // Routing (card 1)
  ["roll", "pitch", "yaw"].forEach(srv => {
    if (pbStateCfg[srv]) {
      _pbState[srv].src  = pbStateCfg[srv].src  ?? _pbState[srv].src;
      _pbState[srv].sign = pbStateCfg[srv].sign ?? _pbState[srv].sign;
    }
    if (cfg.pbEn[srv] !== undefined) {
      _pbEn[srv] = !!cfg.pbEn[srv];
      const el = document.getElementById(`rt-en-${srv}`);
      if (el) el.checked = _pbEn[srv];
    }
  });
  _pbRenderAll();

  // Limiti per-giunto (card 3)
  if (cfg.limits) {
    _limitsCache = { ..._limitsCache, ...cfg.limits };
    const joints = ["base", "spalla", "gomito", "yaw", "pitch", "roll"];
    joints.forEach(j => {
      if (!cfg.limits[j]) return;
      const minEl = document.getElementById(`lim-${j}-min`);
      const maxEl = document.getElementById(`lim-${j}-max`);
      if (minEl && cfg.limits[j].min !== undefined) minEl.value = cfg.limits[j].min;
      if (maxEl && cfg.limits[j].max !== undefined) maxEl.value = cfg.limits[j].max;
    });
  }

  // Parametri tuning (card 4)
  if (cfg.tuning) {
    OPERATIONAL_TUNE_IDS.forEach(id => {
      if (cfg.tuning[id] === undefined) return;
      const sl = document.getElementById(id);
      if (!sl) return;
      sl.value = cfg.tuning[id];
      const dispId = TUNE_MAP[id];
      if (dispId) {
        const decimals = (sl.step || "1").includes(".") ? (sl.step.split(".")[1].length) : 0;
        s(dispId, parseFloat(cfg.tuning[id]).toFixed(decimals));
      }
    });
  }

  if (cfg.controllerMappings && typeof cfg.controllerMappings === "object") {
    _controllerMappingsPersisted = JSON.parse(JSON.stringify(cfg.controllerMappings));
  }

  if (cfg.armControl && typeof cfg.armControl === "object") {
    const preferred = cfg.armControl.preferredController === "left" ? "left" : "right";
    const enabled = cfg.armControl.enabledControllers || {};
    _armControlState = {
      preferredController: preferred,
      enabledControllers: {
        right: enabled.right !== undefined ? !!enabled.right : _armControlDefaults.enabledControllers.right,
        left: enabled.left !== undefined ? !!enabled.left : _armControlDefaults.enabledControllers.left,
      },
    };
  }

  let _touchedHeadAssistUi = false;
  if (cfg.headAssist && typeof cfg.headAssist === "object") {
    const h = cfg.headAssist;
    _headAssistDefaults = {
      ..._headAssistDefaults,
      ...h,
      yaw: { ..._headAssistDefaults.yaw, ...(h.yaw || {}) },
      pitch: { ..._headAssistDefaults.pitch, ...(h.pitch || {}) },
      roll: { ..._headAssistDefaults.roll, ...(h.roll || {}) },
      assistEnable: { ..._headAssistDefaults.assistEnable, ...(h.assistEnable || {}) },
      pitchSplit: { ..._headAssistDefaults.pitchSplit, ...(h.pitchSplit || {}) },
      rollSplit: { ..._headAssistDefaults.rollSplit, ...(h.rollSplit || {}) },
    };
    _touchedHeadAssistUi = true;
  } else if (cfg.ikMode && typeof cfg.ikMode === "object") {
    _headAssistDefaults = {
      ..._headAssistDefaults,
      enabled: cfg.ikMode.enabled !== undefined ? !!cfg.ikMode.enabled : _headAssistDefaults.enabled,
    };
    _touchedHeadAssistUi = true;
  }

  // assistMode / assistDls are top-level keys, independent from headAssist.
  // Read them unconditionally so the DLS UI syncs even if the loaded config
  // has no headAssist block.
  if (typeof cfg.assistMode === "string") {
    _assistModeDefault = (cfg.assistMode === "dls") ? "dls" : "rate";
    _touchedHeadAssistUi = true;
  }
  if (cfg.assistDls && typeof cfg.assistDls === "object") {
    _assistDlsDefaults = { ..._assistDlsDefaults, ...cfg.assistDls };
    _touchedHeadAssistUi = true;
  }
  if (_touchedHeadAssistUi) {
    _applyHeadAssistDefaultsToUi();
  }

  return true;
}

/* Carica la configurazione salvata dal Pi all'avvio della pagina */
async function _loadSavedRoutingConfig() {
  try {
    const cfg = await loadRoutingConfig();
    if (_pbApplyConfigObject(cfg)) {
      const statusEl = document.getElementById("routing-status");
      const ts = cfg.savedAt ? ` (salvata: ${new Date(cfg.savedAt).toLocaleString("it-IT")})` : "";
      if (statusEl) statusEl.textContent = `Config caricata${ts}`;
      setStatus("params-status", `Configurazione pagina caricata da file${ts}`, "ok");
      setVrUiLoadedStatus(`UI: valori caricati da file${ts}`, "ok");
      return true;
    }
  } catch (_) {
    // Errore di rete: usa i default, non bloccare la pagina
  }
  return false;
}

function _readAndValidateJointLimits() {
  const minProbe = document.getElementById("lim-base-min");
  const maxProbe = document.getElementById("lim-base-max");
  if (!minProbe || !maxProbe) {
    return { ok: true, vals: _limitsCache };
  }
  const joints = ["base", "spalla", "gomito", "yaw", "pitch", "roll"];
  const vals = {};
  for (const j of joints) {
    const minEl = document.getElementById(`lim-${j}-min`);
    const maxEl = document.getElementById(`lim-${j}-max`);
    const minV = parseInt(minEl?.value, 10);
    const maxV = parseInt(maxEl?.value, 10);
    if (isNaN(minV) || isNaN(maxV) || minV >= maxV || minV < 0 || maxV > 180) {
      return { ok: false, joint: j.toUpperCase() };
    }
    vals[j] = { min: minV, max: maxV };
  }
  return { ok: true, vals };
}

/**
 * Pipeline unica: salva la UI su routing_config.json poi il server invia SET_VR_PARAMS + limiti.
 * L’esito reale arriva come messaggio vr_config_applied (no successi finti se WS assente).
 */
async function applyAllParamsToFirmware() {
  const lim = _readAndValidateJointLimits();
  if (!lim.ok) {
    const msg = `Errore: limiti non validi per ${lim.joint}`;
    setStatus("params-status", msg, "err");
    setGlobalStatus(msg, "err");
    return false;
  }

  try {
    await persistFullPageConfig();
  } catch (err) {
    const msg = `Salvataggio su Pi fallito: ${err.message}`;
    setStatus("params-status", msg, "err");
    setGlobalStatus(msg, "err");
    return false;
  }

  setVrUiLoadedStatus("UI: allineata al file salvato sul Raspberry", "ok");
  setStatus("params-status", "File salvato — applicazione firmware via server…");

  if (!sendCommand("apply_saved_vr_config", {})) {
    setVrRobotApplyStatus(false);
    const msg = "WebSocket non pronto — impossibile richiedere l’applicazione al robot";
    setStatus("params-status", msg, "err");
    setGlobalStatus(msg, "err");
    return false;
  }

  setVrRobotApplyStatus(null);
  const ok = await waitVrConfigApplied(12000);
  if (ok) {
    setStatus("params-status", "Configurazione pagina applicata sullo STM32 (OK server)", "ok");
    setGlobalStatus("Configurazione applicata sul firmware", "ok");
  } else {
    setStatus("params-status", "Applicazione fallita o timeout — controllare UART / log Pi", "err");
    setGlobalStatus("Apply firmware non confermato", "err");
  }
  return ok;
}

/* Persist only: salva il file sul Raspberry, senza implicare apply firmware. */
async function persistFullPageConfig() {
  await saveRoutingConfig(_buildFullPageConfigObject());
}

async function saveConfigFromButton(btn, statusId, successLabel) {
  try {
    await persistFullPageConfig();
    const stamp = new Date().toLocaleString("it-IT");
    setStatus(statusId, `${successLabel} salvata — ${stamp}`, "ok");
    setGlobalStatus(`${successLabel} salvata su JSON`, "ok");
    flashButton(btn, "✓ Salvata!");
    return true;
  } catch (err) {
    setStatus(statusId, `Errore salvataggio: ${err.message}`, "err");
    setGlobalStatus(`Errore durante il salvataggio di ${successLabel.toLowerCase()}`, "err");
    flashButton(btn, "✗ Errore", true);
    return false;
  }
}

function onTelemetry(data) {
  // Angoli IMU robot (da quaternione Madgwick)
  let robotYaw = null, robotPitch = null, robotRoll = null;
  if (data.imu_q_w !== undefined) {
    const e = quatToEuler(data.imu_q_w, data.imu_q_x, data.imu_q_y, data.imu_q_z);
    robotYaw   = e.yaw;
    robotPitch = e.pitch;
    robotRoll  = e.roll;
  }

  // Angoli visore (dal telemetry già calcolato lato server)
  const vrPitch = data.vr_pitch ?? null;
  const vrRoll  = data.vr_roll  ?? null;
  const vrYaw   = data.vr_yaw   ?? null;

  // Valori live header colonne patch bay (angoli visore)
  if (vrPitch !== null) s("live-vr-pitch", vrPitch.toFixed(1)+"°");
  if (vrRoll  !== null) s("live-vr-roll",  vrRoll.toFixed(1)+"°");
  if (vrYaw   !== null) s("live-vr-yaw",   vrYaw.toFixed(1)+"°");

  if (data.ctrl_right_pitch !== undefined) s("live-right-pitch", `${Number(data.ctrl_right_pitch).toFixed(1)}°`);
  if (data.ctrl_right_roll !== undefined) s("live-right-roll", `${Number(data.ctrl_right_roll).toFixed(1)}°`);
  if (data.ctrl_right_yaw !== undefined) s("live-right-yaw", `${Number(data.ctrl_right_yaw).toFixed(1)}°`);
  if (data.ctrl_left_pitch !== undefined) s("live-left-pitch", `${Number(data.ctrl_left_pitch).toFixed(1)}°`);
  if (data.ctrl_left_roll !== undefined) s("live-left-roll", `${Number(data.ctrl_left_roll).toFixed(1)}°`);
  if (data.ctrl_left_yaw !== undefined) s("live-left-yaw", `${Number(data.ctrl_left_yaw).toFixed(1)}°`);

  // Valori IMU robot per riga (dopo eventuale offset zero).
  const imuRobot = { roll: robotRoll, pitch: robotPitch, yaw: robotYaw };
  ["roll","pitch","yaw"].forEach(servo => {
    const ang = imuRobot[servo];
    const en  = _pbEn[servo];
    if (ang != null) {
      s(`rt-live-${servo}`, en ? `${ang.toFixed(1)}°` : "OFF");
    }
  });

}

registerTelemetryHandler(onTelemetry);

registerVrConfigAppliedHandler(onVrConfigAppliedMessage);
registerOpenHandler(() => {
  setVrRobotApplyStatus(null);
  if (!sendCommand("apply_saved_vr_config", {})) {
    setVrRobotApplyStatus(false);
    setGlobalStatus("WS connesso ma invio apply_saved_vr_config fallito", "err");
  }
});

// ─── Sezione 2: slider Roll/Pitch/Yaw manuale ───────────────────────────────

// ─── Sezione 3: limiti per-giunto ───────────────────────────────────────────

// ─── Sezione 4: tuning closed-loop ──────────────────────────────────────────

// Fallback locale: il backend esporta gli stessi default tramite /api/vr-config-defaults.
// Se l'endpoint non è disponibile, la UI continua a funzionare con questi valori.
let TUNE_DEFAULTS = {
  "tg-maxstep":     60,    // g_head_max_step = 0.060f/ciclo → 60°/s
  "tg-velmax":      60,    // current_max_velocity_deg_per_sec = 60°/s
  "tg-veldigital":  35,    // joint_max_vel_deg_s[PITCH/ROLL] = 35°/s
  "tg-lpf-pitch":   1.00,  // bypass LPF: pitch segue il path diretto come gli altri giunti
  "tg-lpf-roll":    1.00,  // bypass LPF: roll segue il path diretto come gli altri giunti
  "tg-joy-dz":      0.10,  // g_head_joy_dz = 0.10f
  "tg-vel-base":    0,     // joint_max_vel_deg_s[BASE]   (0 = usa globale)
  "tg-vel-spalla":  0,     // joint_max_vel_deg_s[SPALLA] (0 = usa globale)
  "tg-vel-gomito":  0,     // joint_max_vel_deg_s[GOMITO] (0 = usa globale)
  "tg-vel-yaw":     0,
  "tg-vel-pitch":   0,
  "tg-vel-roll":    0,
  "tg-vel-yaw-head":   0,
  "tg-vel-pitch-head": 0,
  "tg-vel-roll-head":  0,
  "tg-vel-base-head":  0,
  "tg-vel-spalla-head": 0,
  "tg-vel-gomito-head": 0,
  "tg-sensitivity": 1.0,   // amplificazione: >1 = piccolo movimento testa → grande robot
};

// Mappa slider-id → display-id
const TUNE_MAP = {
  "tg-yaw":         "tv-yaw",
  "tg-pitch":       "tv-pitch",
  "tg-roll":        "tv-roll",
  "tg-alpha-small": "tv-alpha-small",
  "tg-alpha-large": "tv-alpha-large",
  "tg-deadzone":    "tv-deadzone",
  "tg-maxstep":     "tv-maxstep",
  "tg-velmax":      "tv-velmax",
  "tg-veldigital":  "tv-veldigital",
  "tg-lpf-pitch":   "tv-lpf-pitch",
  "tg-lpf-roll":    "tv-lpf-roll",
  "tg-joy-dz":      "tv-joy-dz",
  "tg-vel-base":    "tv-vel-base",
  "tg-vel-spalla":  "tv-vel-spalla",
  "tg-vel-gomito":  "tv-vel-gomito",
  "tg-vel-yaw":     "tv-vel-yaw",
  "tg-vel-pitch":   "tv-vel-pitch",
  "tg-vel-roll":    "tv-vel-roll",
  "tg-vel-yaw-head":   "tv-vel-yaw-head",
  "tg-vel-pitch-head": "tv-vel-pitch-head",
  "tg-vel-roll-head":  "tv-vel-roll-head",
  "tg-vel-base-head":  "tv-vel-base-head",
  "tg-vel-spalla-head": "tv-vel-spalla-head",
  "tg-vel-gomito-head": "tv-vel-gomito-head",
  "tg-sensitivity":   "tv-sensitivity",
};

const HA_TUNE_MAP = {
  "ha-yaw-warn":    "tv-ha-yaw-warn",
  "ha-pitch-warn":  "tv-ha-pitch-warn",
  "ha-gain-base":   "tv-ha-gain-base",
  "ha-gain-spalla": "tv-ha-gain-spalla",
  "ha-gain-gomito": "tv-ha-gain-gomito",
  "ha-assist-alpha":"tv-ha-assist-alpha",
  "ha-max-step":    "tv-ha-max-step",
  "hdls-gain-m":       "tv-hdls-gain-m",
  "hdls-null-gain":    "tv-hdls-null-gain",
  "hdls-lambda-max":   "tv-hdls-lambda-max",
  "hdls-manip-thresh": "tv-hdls-manip-thresh",
  "hdls-max-dq":       "tv-hdls-max-dq",
  "hdls-max-dx":       "tv-hdls-max-dx",
};

function _applyTuneDefaultsToUi() {
  OPERATIONAL_TUNE_IDS.forEach((slId) => {
    const def = TUNE_DEFAULTS[slId];
    const sl = document.getElementById(slId);
    if (!sl) return;
    sl.value = def;
    const dispId = TUNE_MAP[slId];
    if (dispId) {
      const step = sl.step || "1";
      const decimals = step.includes(".") ? step.split(".")[1].length : 0;
      s(dispId, parseFloat(def).toFixed(decimals));
    }
  });
}

function _resetPatchBayToDefaults() {
  ["roll", "pitch", "yaw"].forEach((srv) => {
    _pbState[srv] = { ..._pbStateDefault[srv] };
    _pbEn[srv] = _pbEnDefault[srv];
    const el = document.getElementById(`rt-en-${srv}`);
    if (el) el.checked = _pbEnDefault[srv];
  });
  _pbRenderAll();
}

function _applyBackendVrDefaults(defaults) {
  if (!defaults || typeof defaults !== "object") return false;
  if (defaults.tuning && typeof defaults.tuning === "object") {
    TUNE_DEFAULTS = { ...TUNE_DEFAULTS, ...defaults.tuning };
  }
  if (defaults.pbState && typeof defaults.pbState === "object") {
    _pbStateDefault = {
      roll: { ..._pbStateDefault.roll, ...(defaults.pbState.roll || {}) },
      pitch: { ..._pbStateDefault.pitch, ...(defaults.pbState.pitch || {}) },
      yaw: { ..._pbStateDefault.yaw, ...(defaults.pbState.yaw || {}) },
    };
  }
  if (defaults.pbEn && typeof defaults.pbEn === "object") {
    _pbEnDefault = {
      roll: defaults.pbEn.roll ?? _pbEnDefault.roll,
      pitch: defaults.pbEn.pitch ?? _pbEnDefault.pitch,
      yaw: defaults.pbEn.yaw ?? _pbEnDefault.yaw,
    };
  }
  if (defaults.controllerMappings && typeof defaults.controllerMappings === "object") {
    _controllerMappingsPersisted = JSON.parse(JSON.stringify(defaults.controllerMappings));
  }
  if (defaults.armControl && typeof defaults.armControl === "object") {
    _armControlDefaults = {
      preferredController: defaults.armControl.preferredController === "left" ? "left" : "right",
      enabledControllers: {
        right: defaults.armControl.enabledControllers?.right ?? _armControlDefaults.enabledControllers.right,
        left: defaults.armControl.enabledControllers?.left ?? _armControlDefaults.enabledControllers.left,
      },
    };
    _armControlState = {
      preferredController: _armControlDefaults.preferredController,
      enabledControllers: { ..._armControlDefaults.enabledControllers },
    };
  }
  if (defaults.headAssist && typeof defaults.headAssist === "object") {
    const h = defaults.headAssist;
    _headAssistDefaults = {
      ..._headAssistDefaults,
      ...h,
      yaw: { ..._headAssistDefaults.yaw, ...(h.yaw || {}) },
      pitch: { ..._headAssistDefaults.pitch, ...(h.pitch || {}) },
      roll: { ..._headAssistDefaults.roll, ...(h.roll || {}) },
      assistEnable: { ..._headAssistDefaults.assistEnable, ...(h.assistEnable || {}) },
      pitchSplit: { ..._headAssistDefaults.pitchSplit, ...(h.pitchSplit || {}) },
      rollSplit: { ..._headAssistDefaults.rollSplit, ...(h.rollSplit || {}) },
    };
    _headAssistFactoryDefaults = JSON.parse(JSON.stringify(_headAssistDefaults));
  }
  return true;
}

async function _loadBackendVrDefaults() {
  try {
    const res = await fetch("/api/vr-config-defaults");
    if (!res.ok) return false;
    const defaults = await res.json();
    return _applyBackendVrDefaults(defaults);
  } catch (_) {
    return false;
  }
}

function getTuneVal(sliderId) {
  return parseFloat(document.getElementById(sliderId)?.value ?? TUNE_DEFAULTS[sliderId]);
}

// Bind live display per ogni slider tuning
Object.entries(TUNE_MAP).forEach(([slId, dispId]) => {
  const sl = document.getElementById(slId);
  if (!sl) return;
  sl.addEventListener("input", () => {
    s(dispId, parseFloat(sl.value).toFixed(sl.step.includes(".") ? sl.step.split(".")[1].length : 0));
  });
});

Object.entries(HA_TUNE_MAP).forEach(([slId, dispId]) => {
  const sl = document.getElementById(slId);
  if (!sl) return;
  sl.addEventListener("input", () => {
    const decimals = sl.step.includes(".") ? sl.step.split(".")[1].length : 0;
    s(dispId, parseFloat(sl.value).toFixed(decimals));
  });
});

document.getElementById("btn-reset-params")?.addEventListener("click", () => {
  OPERATIONAL_TUNE_IDS.forEach((id) => {
    const sl = document.getElementById(id);
    if (!sl) return;
    const def = TUNE_DEFAULTS[id];
    sl.value = def;
    const dispId = TUNE_MAP[id];
    if (dispId) {
      const step = sl.step || "1";
      const decimals = step.includes(".") ? step.split(".")[1].length : 0;
      s(dispId, parseFloat(def).toFixed(decimals));
    }
  });
  setStatus("params-status", "Default velocita ripristinati (non ancora salvati)");
  setGlobalStatus("Default ripristinati. Salva configurazione per persistere sul Raspberry oppure Applica per inviare al firmware.", "");
});


document.getElementById("btn-reset-head-assist")?.addEventListener("click", () => {
  _headAssistDefaults = JSON.parse(JSON.stringify(_headAssistFactoryDefaults));
  _assistModeDefault = _assistModeFactory;
  _assistDlsDefaults = JSON.parse(JSON.stringify(_assistDlsFactory));
  _applyHeadAssistDefaultsToUi();
  setStatus("head-assist-status", "Default HEAD ASSIST ripristinati (non ancora salvati)");
  setGlobalStatus("Default HEAD ASSIST ripristinati. Salva configurazione per persistere sul Raspberry.", "");
});

async function onApplyAllParamsClick(e) {
  const btn = e.currentTarget;
  btn.disabled = true;
  const ok = await applyAllParamsToFirmware();
  flashButton(btn, ok ? "✓ Applicato!" : "✗ Fallito", !ok);
}
document.querySelectorAll(".apply-all-params").forEach((el) => el.addEventListener("click", onApplyAllParamsClick));

document.querySelectorAll(".save-config-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    const statusId = btn.dataset.statusId;
    const successLabel = btn.dataset.successLabel || "Configurazione";
    saveConfigFromButton(btn, statusId, successLabel);
  });
});


// Carica prima i default autorevoli backend, poi l'eventuale config persistita.
// La connessione WS è avviata da dashboard.js (dopo questo modulo).
(async () => {
  await _loadBackendVrDefaults();
  _applyHeadAssistDefaultsToUi();
  const loaded = await _loadSavedRoutingConfig();
  if (!loaded) {
    _resetPatchBayToDefaults();
    _applyTuneDefaultsToUi();
    _applyHeadAssistDefaultsToUi();
    _controllerMappingsPersisted = _controllerMappingsTemplate();
    setStatus("params-status", "Default backend pronti da salvare.");
    setVrUiLoadedStatus("UI: valori predefiniti backend (nessun routing_config.json sul Pi)", "");
  }
})();
