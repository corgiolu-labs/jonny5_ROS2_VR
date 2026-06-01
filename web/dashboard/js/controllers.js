/* controllers.js — Editor visivo della mappatura runtime controller VR.
 *
 * Layout:
 *  - Tab per modalità in alto (STEREO/POSE/MANUAL/HEAD/HYBRID/ASSIST).
 *  - Selettore tipo evento (press/held/release/click) e modificatore (bothGrips/...).
 *  - Due card "Controller SX/DX" con 5 slot pulsanti ciascuna; dropdown per
 *    assegnare l'azione al binding (mode, hand.button.edge[|modifier]).
 *  - Salvataggio: POST /api/controller-mappings + WS broadcast live-reload.
 */

import { connectJ5Dashboard } from "../../shared/js/j5_common.js";

// ---------------------------------------------------------------------------
// Costanti
// ---------------------------------------------------------------------------

const MODES = ["STEREO", "POSE", "MANUAL", "HEAD", "HYBRID", "ASSIST"];

// Whitelist actions disponibili (devono combaciare con quelle registrate dal viewer).
const ACTIONS = [
  "noop",
  "setAutofocusBoth",
  "setManualFocusMode",
  "toggleHudVisible",
  "toggleLatencyOverlay",
  "toggleLinkMode",
  "zoom_in_continuous",
  "zoom_out_continuous",
  "toggleCrosshair",
  "refocusCamerasHw",
  "toggleAutoConv",
];

// Stick analogici: descrizione (sola lettura) per modalità.
// Mappature verificate da firmware (j5vr_actuation.c, j5vr_head.c, rt_loop.c)
// e dal viewer JS (viewer_stereo_xr.html). NON modificabile da qui.
const STICK_INFO = {
  STEREO: {
    "left.x":  "SCALE o ROLL cam0 (toggle con click thumbstick SX)",
    "left.y":  "Focus cam0",
    "right.x": "SCALE o ROLL cam1 (toggle con click thumbstick DX)",
    "right.y": "Focus cam1",
  },
  POSE: {
    "left.x":  "(non usato — pose via TELEOPPOSE one-shot)",
    "left.y":  "(non usato)",
    "right.x": "(non usato)",
    "right.y": "(non usato)",
  },
  MANUAL: {
    "left.x":  "→ YAW polso (firmware)",
    "left.y":  "→ PITCH polso (firmware)",
    "right.x": "→ SPALLA (firmware)",
    "right.y": "→ GOMITO (firmware)",
  },
  HEAD: {
    "left.x":  "(firmware non legge — polso da IMU visore)",
    "left.y":  "(firmware non legge — polso da IMU visore)",
    "right.x": "(firmware non legge — polso da IMU visore)",
    "right.y": "(firmware non legge — polso da IMU visore)",
  },
  HYBRID: {
    "left.x":  "(firmware non legge — disponibile per azioni web)",
    "left.y":  "(firmware non legge — disponibile per azioni web)",
    "right.x": "→ SPALLA (firmware) | + zoom cam quando !grip",
    "right.y": "→ GOMITO (firmware)",
  },
  ASSIST: {
    "left.x":  "→ solver DLS sul Pi → braccio (B/S/G)",
    "left.y":  "→ solver DLS sul Pi → braccio (B/S/G)",
    "right.x": "→ solver DLS sul Pi (polso da IMU visore)",
    "right.y": "→ solver DLS sul Pi (polso da IMU visore)",
  },
};

// Cosa fa OGNI bottone a livello hardcoded (firmware o viewer JS) per modalità.
// Valore: { src: "fw"|"viewer", desc: "..." } oppure null se nulla di hardcoded.
//   src="fw"     -> comportamento implementato in firmware STM32
//   src="viewer" -> comportamento hardcoded nel viewer (NON in dispatch system)
// Verificato da firmware (j5vr_actuation.c, j5vr_head.c, assist_v2_raw.c, rt_loop.c)
// e viewer (viewer_stereo_xr.html).
const BUTTON_HARDCODED_INFO = {
  STEREO: {
    "left.X":       null,
    "left.Y":       null,
    "left.trigger": { src: "viewer", desc: "convergenza − (convPx, calib stereo)" },
    "left.grip":    { src: "viewer", desc: "vertPx0 cam SX (con stick Y deflesso)" },
    "left.thumb":   { src: "viewer", desc: "toggle SCALE/ROLL stick SX" },
    "right.A":      null,
    "right.B":      null,
    "right.trigger":{ src: "viewer", desc: "convergenza + (convPx, calib stereo)" },
    "right.grip":   { src: "viewer", desc: "vertPx1 cam DX (con stick Y deflesso)" },
    "right.thumb":  null,
  },
  POSE: {
    "left.X": null, "left.Y": null,
    "left.trigger": null, "left.grip": null, "left.thumb": null,
    "right.A": null, "right.B": null,
    "right.trigger": null, "right.grip": null, "right.thumb": null,
  },
  MANUAL: {
    "left.X":       { src: "fw", desc: "ROLL polso (+)" },
    "left.Y":       { src: "fw", desc: "ROLL polso (−)" },
    "left.trigger": { src: "fw", desc: "BASE rotation (−)" },
    "left.grip":    { src: "fw", desc: "deadman (richiesto + grip dx)" },
    "left.thumb":   null,
    "right.A":      { src: "fw", desc: "max velocity ridotta (vel A)" },
    "right.B":      { src: "fw", desc: "max velocity piena (vel B)" },
    "right.trigger":{ src: "fw", desc: "BASE rotation (+)" },
    "right.grip":   { src: "fw", desc: "deadman (richiesto + grip sx)" },
    "right.thumb":  null,
  },
  HEAD: {
    "left.X":       null,
    "left.Y":       null,
    "left.trigger": null,
    "left.grip":    { src: "fw", desc: "deadman (polso da IMU visore)" },
    "left.thumb":   null,
    "right.A":      { src: "fw", desc: "max velocity ridotta" },
    "right.B":      { src: "fw", desc: "max velocity piena" },
    "right.trigger":null,
    "right.grip":   { src: "fw", desc: "deadman" },
    "right.thumb":  null,
  },
  HYBRID: {
    "left.X":       null,
    "left.Y":       null,
    "left.trigger": { src: "fw", desc: "BASE rotation (−)" },
    "left.grip":    { src: "fw", desc: "deadman" },
    "left.thumb":   null,
    "right.A":      { src: "fw", desc: "max velocity ridotta" },
    "right.B":      { src: "fw", desc: "max velocity piena" },
    "right.trigger":{ src: "fw", desc: "BASE rotation (+)" },
    "right.grip":   { src: "fw", desc: "deadman" },
    "right.thumb":  null,
  },
  ASSIST: {
    "left.X":       { src: "fw", desc: "ROLL polso (via solver DLS)" },
    "left.Y":       { src: "fw", desc: "ROLL polso (via solver DLS)" },
    "left.trigger": { src: "fw", desc: "input solver DLS" },
    "left.grip":    { src: "fw", desc: "deadman + attiva solver" },
    "left.thumb":   null,
    "right.A":      { src: "fw", desc: "max velocity ridotta" },
    "right.B":      { src: "fw", desc: "max velocity piena" },
    "right.trigger":{ src: "fw", desc: "input solver DLS" },
    "right.grip":   { src: "fw", desc: "deadman + attiva solver" },
    "right.thumb":  null,
  },
};

// Slot pulsanti per controller. L'ordine determina la disposizione visiva.
const LEFT_SLOTS = [
  { key: "thumb", label: "Thumb (click)", glyph: "●", glyphCls: "thumb" },
  { key: "X",     label: "X (sup.)",      glyph: "X", glyphCls: "face" },
  { key: "Y",     label: "Y (inf.)",      glyph: "Y", glyphCls: "face" },
  { key: "trigger", label: "Trigger",     glyph: "TRIG", glyphCls: "trig" },
  { key: "grip",    label: "Grip",        glyph: "GRIP", glyphCls: "grip" },
];
const RIGHT_SLOTS = [
  { key: "thumb", label: "Thumb (click)", glyph: "●", glyphCls: "thumb" },
  { key: "A",     label: "A (sup.)",      glyph: "A", glyphCls: "face" },
  { key: "B",     label: "B (inf.)",      glyph: "B", glyphCls: "face" },
  { key: "trigger", label: "Trigger",     glyph: "TRIG", glyphCls: "trig" },
  { key: "grip",    label: "Grip",        glyph: "GRIP", glyphCls: "grip" },
];

// ---------------------------------------------------------------------------
// Stato
// ---------------------------------------------------------------------------

const state = {
  config: null,         // { version, modes: { MODE: { eventKey: action } } }
  initialJSON: null,    // snapshot per dirty detection
  ws: null,
  ui: {
    mode: "HYBRID",     // tab attivo
    edge: "press",      // press/held/release/click
    modifier: "",       // ""/bothGrips/noGrips/leftGrip/rightGrip
  },
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

const $ = (id) => document.getElementById(id);

function setStatus(text, level) {
  const el = $("status");
  if (!el) return;
  el.textContent = text;
  el.classList.remove("ok", "warn", "err");
  if (level) el.classList.add(level);
}

function isDirty() {
  if (!state.config || !state.initialJSON) return false;
  return JSON.stringify(state.config) !== state.initialJSON;
}

function refreshDirtyUI() {
  const tb = $("toolbar");
  if (!tb) return;
  if (isDirty()) tb.classList.add("dirty"); else tb.classList.remove("dirty");
}

function eventKey(hand, button) {
  let key = `${hand}.${button}.${state.ui.edge}`;
  if (state.ui.modifier) key += `|${state.ui.modifier}`;
  return key;
}

function getActionForSlot(hand, button) {
  if (!state.config || !state.config.modes) return "noop";
  const mmap = state.config.modes[state.ui.mode] || {};
  return mmap[eventKey(hand, button)] || "noop";
}

function setActionForSlot(hand, button, action) {
  if (!state.config) return;
  if (!state.config.modes) state.config.modes = {};
  if (!state.config.modes[state.ui.mode]) state.config.modes[state.ui.mode] = {};
  const mmap = state.config.modes[state.ui.mode];
  const key = eventKey(hand, button);
  if (!action || action === "noop") {
    delete mmap[key];
  } else {
    mmap[key] = action;
  }
  refreshDirtyUI();
  setStatus("Modifica pendente", "warn");
}

// ---------------------------------------------------------------------------
// Render
// ---------------------------------------------------------------------------

function renderTabs() {
  const wrap = $("mode-tabs");
  if (!wrap) return;
  wrap.innerHTML = "";
  for (const m of MODES) {
    const b = document.createElement("button");
    b.dataset.mode = m;
    b.textContent = m;
    if (m === state.ui.mode) b.classList.add("active");
    b.addEventListener("click", () => {
      state.ui.mode = m;
      renderTabs();
      renderControllers();
    });
    wrap.appendChild(b);
  }
}

function renderControllers() {
  renderControllerCard("ctrl-left",  "left",  LEFT_SLOTS);
  renderControllerCard("ctrl-right", "right", RIGHT_SLOTS);
  renderStickInfo();
  updateEvtPreview();
}

function renderStickInfo() {
  const info = STICK_INFO[state.ui.mode] || {};
  const pairs = [
    ["left-stick-x-desc",  info["left.x"]],
    ["left-stick-y-desc",  info["left.y"]],
    ["right-stick-x-desc", info["right.x"]],
    ["right-stick-y-desc", info["right.y"]],
  ];
  for (const [id, txt] of pairs) {
    const el = $(id);
    if (!el) continue;
    el.textContent = txt || "—";
    el.classList.toggle("muted", !txt || /^\(non usato\)/i.test(txt));
  }
}

function renderControllerCard(containerId, hand, slots) {
  const wrap = $(containerId);
  if (!wrap) return;
  wrap.innerHTML = "";
  const fwTable = BUTTON_HARDCODED_INFO[state.ui.mode] || {};
  for (const s of slots) {
    const slot = document.createElement("div");
    slot.className = "btn-slot";
    const action = getActionForSlot(hand, s.key);
    if (action !== "noop") slot.classList.add("set"); else slot.classList.add("unset");

    // Etichetta + glyph del pulsante
    const lhs = document.createElement("div");
    lhs.className = "btn-icon";
    lhs.innerHTML = `<span class="glyph ${s.glyphCls}">${s.glyph}</span><span class="name">${s.label}</span>`;
    slot.appendChild(lhs);

    // Colonna destra: dropdown action + annotazione firmware sotto
    const rhs = document.createElement("div");
    rhs.className = "slot-rhs";

    const sel = document.createElement("select");
    for (const a of ACTIONS) {
      const o = document.createElement("option");
      o.value = a; o.textContent = a;
      if (a === action) o.selected = true;
      sel.appendChild(o);
    }
    if (action && !ACTIONS.includes(action)) {
      const opt = document.createElement("option");
      opt.value = action; opt.textContent = action + " (legacy)";
      opt.selected = true;
      sel.insertBefore(opt, sel.firstChild);
    }
    sel.addEventListener("change", () => {
      setActionForSlot(hand, s.key, sel.value);
      slot.classList.remove("set", "unset");
      if (sel.value !== "noop") slot.classList.add("set"); else slot.classList.add("unset");
    });
    rhs.appendChild(sel);

    // Annotazione hardcoded (firmware o viewer JS — sola lettura).
    const hcInfo = fwTable[`${hand}.${s.key}`];
    if (hcInfo && hcInfo.desc) {
      const fwEl = document.createElement("div");
      fwEl.className = "fw-note src-" + hcInfo.src;
      fwEl.innerHTML = `<span class="fw-tag">${hcInfo.src}</span><span class="fw-desc">${hcInfo.desc}</span>`;
      rhs.appendChild(fwEl);
    }

    // Altri binding per questo bottone in QUESTA modalità ma sotto edge/modifier
    // diversi dal filtro corrente — visibili sempre per evitare di "nasconderli".
    const others = collectOtherBindingsForButton(hand, s.key);
    if (others.length > 0) {
      const oWrap = document.createElement("div");
      oWrap.className = "other-bindings";
      const lbl = document.createElement("span");
      lbl.className = "ob-label"; lbl.textContent = "altre:";
      oWrap.appendChild(lbl);
      for (const o of others) {
        const chip = document.createElement("span");
        chip.className = "ob-chip";
        chip.title = o.fullKey;
        chip.textContent = `${o.ctxLabel} → ${o.action}`;
        oWrap.appendChild(chip);
      }
      rhs.appendChild(oWrap);
    }

    slot.appendChild(rhs);
    wrap.appendChild(slot);
  }
}

// Restituisce tutti i binding presenti per ${hand}.${button}.* nella modalità
// corrente che NON corrispondono al filtro edge/modifier attualmente attivo.
function collectOtherBindingsForButton(hand, button) {
  const out = [];
  if (!state.config || !state.config.modes) return out;
  const mmap = state.config.modes[state.ui.mode];
  if (!mmap) return out;
  const prefix = `${hand}.${button}.`;
  const currentKey = eventKey(hand, button);
  for (const [k, v] of Object.entries(mmap)) {
    if (!k.startsWith(prefix)) continue;
    if (k === currentKey) continue;
    if (!v || v === "noop") continue;
    // Estrai edge|modifier dalla chiave: es. "right.A.held|bothGrips" -> "held|bothGrips"
    const tail = k.substring(prefix.length);
    out.push({ fullKey: k, ctxLabel: tail, action: v });
  }
  return out;
}

function updateEvtPreview() {
  const el = $("evt-preview");
  if (!el) return;
  // Esempio: usa il primo slot del SX
  const sample = LEFT_SLOTS[1].key; // "X"
  el.textContent = eventKey("left", sample);
}

// ---------------------------------------------------------------------------
// Save / Load
// ---------------------------------------------------------------------------

async function loadConfig() {
  setStatus("Caricamento...", "warn");
  const cfg = await window.controllerRemap.loadFromServer();
  if (!cfg) {
    setStatus("Errore caricamento", "err");
    state.config = { version: 1, modes: {} };
  } else {
    state.config = cfg;
    setStatus("Caricato", "ok");
  }
  // Garantisci tutte le modalità anche se vuote
  if (!state.config.modes) state.config.modes = {};
  for (const m of MODES) {
    if (!state.config.modes[m]) state.config.modes[m] = {};
  }
  state.initialJSON = JSON.stringify(state.config);
  renderTabs();
  renderControllers();
  refreshDirtyUI();
}

async function saveConfig() {
  if (!state.config) return;
  setStatus("Salvataggio...", "warn");
  try {
    await window.controllerRemap.saveToServer(state.config);
    state.initialJSON = JSON.stringify(state.config);
    refreshDirtyUI();
    // Broadcast WS live-reload
    try {
      if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({
          type: "controller_mappings_updated",
          config: state.config,
        }));
      }
    } catch (_) {}
    setStatus("Salvato e applicato", "ok");
  } catch (e) {
    setStatus("Errore: " + (e.message || e), "err");
  }
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

document.addEventListener("DOMContentLoaded", () => {
  state.ws = connectJ5Dashboard();

  // Tab modalità: rendering iniziale
  renderTabs();

  // Edge radio
  document.querySelectorAll("#edge-radio input[type=radio]").forEach((r) => {
    r.addEventListener("change", () => {
      if (r.checked) {
        state.ui.edge = r.value;
        renderControllers();
      }
    });
  });

  // Modifier select
  $("mod-select").addEventListener("change", (ev) => {
    state.ui.modifier = ev.target.value;
    renderControllers();
  });

  // Toolbar
  $("btn-reload").addEventListener("click", loadConfig);
  $("btn-save").addEventListener("click", saveConfig);

  // Carica config iniziale
  loadConfig();

  // Live-update da altri client (refresh UI se l'utente non sta editando)
  if (window.controllerRemap) {
    window.controllerRemap.onConfigChange(() => {
      if (!isDirty()) {
        state.config = window.controllerRemap.getConfig();
        if (!state.config.modes) state.config.modes = {};
        for (const m of MODES) {
          if (!state.config.modes[m]) state.config.modes[m] = {};
        }
        state.initialJSON = JSON.stringify(state.config);
        renderTabs();
        renderControllers();
        setStatus("Aggiornato da altro client", "ok");
      }
    });
  }
});
