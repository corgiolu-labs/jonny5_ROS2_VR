/**
 * settings.js — Pagina Settings JONNY5-4.0
 *
 * Gestisce la visualizzazione e modifica di:
 *   - Offset meccanici (servo_offset_deg, sola lettura)
 *   - Pose predefinite HOME / PARK / VR (modificabili + applicabili via SETPOSE)
 *   - Parametri sistema: vel_max, profilo di moto (salvati in settings.json sul Pi)
 *
 * Flusso dati:
 *   Apertura pagina → get_settings → ws_server.py → settings.json → popola UI
 *   Bottone Applica → uart {cmd: "SETPOSE B S G Y P R vel PROFILE"} → STM32
 *   Bottone Salva   → save_settings → ws_server.py → settings.json → feedback
 */

import {
  connectJ5Dashboard,
  registerSettingsHandler,
  registerPoeParamsHandler,
  registerUartResponseHandler,
  sendCommand,
  addLog,
  loadRoutingConfig,
  saveRoutingConfig,
} from "../../../shared/js/j5_common.js";

// ---------------------------------------------------------------------------
// Stato locale
// ---------------------------------------------------------------------------
const JOINTS = ["B", "S", "G", "Y", "P", "R"];
const PROFILES = ["RTR3", "RTR5", "BB", "BCB"];
const DEMO_STEP_DEFAULT = { angles: [90, 90, 90, 90, 90, 90], vel: 40, profile: "RTR5" };

// --- POE: source of truth = Raspberry (j5_poe_params.json); localStorage = mirror / fallback ---
const POE_STORAGE_KEY = "j5_poe_params";
const POE_MIGRATE_FLAG = "j5_poe_migrate_done";
const POE_DEFAULT = {
  S: [
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -0.094, 0, 0],
    [0, 1, 0, -0.154, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -0.311, 0, 0],
    [1, 0, 0, 0, 0.311, 0],
  ],
  M: [
    [1, 0, 0, 0.060],
    [0, 1, 0, 0.000],
    [0, 0, 1, 0.311],
    [0, 0, 0, 1.000],
  ],
};

function _clonePoeDefault() {
  return JSON.parse(JSON.stringify(POE_DEFAULT));
}

function _poeCfgEquals(a, b) {
  if (!a?.S || !b?.S || !a?.M || !b?.M) return false;
  try {
    return JSON.stringify(a.S) === JSON.stringify(b.S) && JSON.stringify(a.M) === JSON.stringify(b.M);
  } catch (_) {
    return false;
  }
}

/** Solo browser: usato prima del WS e come fallback offline. */
function loadPoeFromLocalStorageOnly() {
  try {
    const raw = localStorage.getItem(POE_STORAGE_KEY);
    if (!raw) return _clonePoeDefault();
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed?.S) && parsed.S.length === 6 && Array.isArray(parsed?.M) && parsed.M.length === 4) {
      return parsed;
    }
  } catch (_) { /* ignore */ }
  return _clonePoeDefault();
}

function savePoeSettingsLocalMirror(cfg) {
  localStorage.setItem(POE_STORAGE_KEY, JSON.stringify({ S: cfg.S, M: cfg.M }));
}

/**
 * Primo boot senza file su Pi: se il browser ha POE non default, invia set_poe_params una volta per sessione.
 * Ritorna la config da mostrare (LS custom o messaggio server).
 */
function resolvePoeCfgFromServerMessage(msg) {
  let cfg = { S: msg.S, M: msg.M };
  if (msg.persisted === false && sessionStorage.getItem(POE_MIGRATE_FLAG) !== "1") {
    const ls = loadPoeFromLocalStorageOnly();
    if (!_poeCfgEquals(ls, _clonePoeDefault())) {
      sendCommand("set_poe_params", { S: ls.S, M: ls.M });
      sessionStorage.setItem(POE_MIGRATE_FLAG, "1");
      addLog("POE: migrazione parametri browser → Raspberry");
      cfg = { S: ls.S, M: ls.M };
    } else {
      sessionStorage.setItem(POE_MIGRATE_FLAG, "1");
    }
  }
  return cfg;
}

/** Backend / JSON: v in metri, traslazioni M in metri. UI Settings: mm. */
function _poeMmFromMeters(m) {
  return Number(m) * 1000;
}
function _poeMetersFromMm(mm) {
  return (Number(mm) || 0) / 1000;
}
function _poeMUiFromStorage(val, row, col) {
  if (col === 3 && row <= 2) return _poeMmFromMeters(val);
  return Number(val);
}
function _poeMStorageFromUi(val, row, col) {
  if (col === 3 && row <= 2) return _poeMetersFromMm(val);
  const x = parseFloat(val);
  return Number.isFinite(x) ? x : 0;
}

function buildPoeTablesSettings(cfg) {
  const sTbody = document.getElementById("settings-poe-s-tbody");
  const mTbody = document.getElementById("settings-poe-m-tbody");
  if (!sTbody || !mTbody) return;
  sTbody.innerHTML = "";
  const S = cfg.S || POE_DEFAULT.S;
  for (let i = 0; i < 6; i++) {
    const row = Array.isArray(S[i]) && S[i].length >= 6 ? S[i] : POE_DEFAULT.S[i];
    const tr = document.createElement("tr");
    tr.dataset.sIdx = String(i);
    const jointTd = document.createElement("td");
    jointTd.className = "col-link";
    jointTd.textContent = `S${i + 1}`;
    tr.appendChild(jointTd);
    for (let j = 0; j < 6; j++) {
      const td = document.createElement("td");
      const inp = document.createElement("input");
      inp.type = "number";
      inp.step = "any";
      inp.dataset.sj = String(j);
      const raw = Number(row[j]);
      inp.value = j >= 3 ? _poeMmFromMeters(raw) : raw;
      td.appendChild(inp);
      tr.appendChild(td);
    }
    sTbody.appendChild(tr);
  }
  mTbody.innerHTML = "";
  const M = cfg.M || POE_DEFAULT.M;
  for (let r = 0; r < 4; r++) {
    const tr = document.createElement("tr");
    tr.dataset.mIdx = String(r);
    const mrow = Array.isArray(M[r]) && M[r].length >= 4 ? M[r] : POE_DEFAULT.M[r];
    for (let c = 0; c < 4; c++) {
      const td = document.createElement("td");
      const inp = document.createElement("input");
      inp.type = "number";
      inp.step = "any";
      inp.dataset.mc = String(c);
      inp.value = _poeMUiFromStorage(mrow[c], r, c);
      td.appendChild(inp);
      tr.appendChild(td);
    }
    mTbody.appendChild(tr);
  }
}

function collectPoeFromSettings() {
  const sTbody = document.getElementById("settings-poe-s-tbody");
  const mTbody = document.getElementById("settings-poe-m-tbody");
  if (!sTbody || !mTbody) return null;
  const sRows = sTbody.querySelectorAll("tr[data-s-idx]");
  if (sRows.length !== 6) return null;
  const S = [];
  for (const tr of sRows) {
    const row = [];
    for (let j = 0; j < 6; j++) {
      const inp = tr.querySelector(`input[data-sj="${j}"]`);
      const v = parseFloat(inp?.value);
      const num = Number.isFinite(v) ? v : 0;
      row.push(j >= 3 ? _poeMetersFromMm(num) : num);
    }
    S.push(row);
  }
  const mRows = mTbody.querySelectorAll("tr[data-m-idx]");
  if (mRows.length !== 4) return null;
  const M = [];
  for (const tr of mRows) {
    const r = parseInt(tr.dataset.mIdx, 10);
    const row = [];
    for (let c = 0; c < 4; c++) {
      const inp = tr.querySelector(`input[data-mc="${c}"]`);
      const v = parseFloat(inp?.value);
      const num = Number.isFinite(v) ? v : 0;
      row.push(_poeMStorageFromUi(num, r, c));
    }
    M.push(row);
  }
  return { S, M };
}

function initRobotPoeSection() {
  registerPoeParamsHandler((msg) => {
    if (msg.type === "poe_params" && Array.isArray(msg.S) && msg.S.length === 6 && Array.isArray(msg.M) && msg.M.length === 4) {
      const cfg = resolvePoeCfgFromServerMessage(msg);
      savePoeSettingsLocalMirror(cfg);
      buildPoeTablesSettings(cfg);
      addLog("POE sincronizzato dal Raspberry");
    } else if (msg.type === "poe_params_saved") {
      showFeedback("fb-settings-poe", msg.ok, msg.ok ? "Salvato sul Raspberry" : "Salvataggio POE fallito");
      if (msg.ok) {
        addLog("POE salvato su j5_poe_params.json");
        sendCommand("get_poe_params", {});
      } else {
        addLog("ERRORE: salvataggio POE sul Raspberry fallito");
      }
    }
  });

  buildPoeTablesSettings(loadPoeFromLocalStorageOnly());

  document.getElementById("btn-settings-save-poe")?.addEventListener("click", () => {
    const collected = collectPoeFromSettings();
    if (!collected) {
      showFeedback("fb-settings-poe", false, "Tabella POE incompleta");
      return;
    }
    sendCommand("set_poe_params", { S: collected.S, M: collected.M });
    showFeedback("fb-settings-poe", true, "Invio in corso…");
    addLog("Richiesta salvataggio POE sul Raspberry");
  });

  document.getElementById("btn-settings-reset-poe")?.addEventListener("click", () => {
    if (!confirm("Ripristinare i parametri POE ai valori predefiniti sul Raspberry?")) return;
    const d = _clonePoeDefault();
    sendCommand("set_poe_params", { S: d.S, M: d.M });
    showFeedback("fb-settings-poe", true, "Reset inviato…");
    addLog("Richiesta reset POE (default) sul Raspberry");
  });
}

// Snapshot degli ultimi settings ricevuti dal server
let _current = {
  offsets:    [90, 90, 90, 90, 90, 90],
  dirs:       [1, 1, 1, 1, 1, 1],
  home:       [90, 90, 90, 90, 90, 90],
  park:       [90, 90, 90, 90, 90, 90],
  vr:         [90, 90, 90, 90, 90, 90],
  vel_max:    80,
  profile:    "RTR5",
  ws_port:    8557,
  demo_steps: [DEMO_STEP_DEFAULT],
};

// ---------------------------------------------------------------------------
// Costruzione griglia 6 input angoli
// ---------------------------------------------------------------------------
function buildAngleGrid(containerId, prefix, disabled = false) {
  const el = document.getElementById(containerId);
  if (!el) return;
  el.innerHTML = "";
  JOINTS.forEach((label, i) => {
    const cell = document.createElement("div");
    cell.className = "angle-cell";

    const lbl = document.createElement("label");
    lbl.textContent = label;
    lbl.setAttribute("for", `${prefix}-${i}`);

    const inp = document.createElement("input");
    inp.type = "number";
    inp.id = `${prefix}-${i}`;
    inp.min = 0;
    inp.max = 180;
    inp.step = 1;
    inp.value = 90;  // default 90 = HOME meccanica
    if (disabled) inp.disabled = true;

    cell.appendChild(lbl);
    cell.appendChild(inp);
    el.appendChild(cell);
  });
}

function buildDirGrid(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;
  el.innerHTML = "";
  JOINTS.forEach((label, i) => {
    const cell = document.createElement("div");
    cell.className = "angle-cell";

    const lbl = document.createElement("label");
    lbl.textContent = label;
    lbl.setAttribute("for", `dir-${i}`);

    const sel = document.createElement("select");
    sel.id = `dir-${i}`;
    const optNorm = document.createElement("option");
    optNorm.value = "1";
    optNorm.textContent = "Normale";
    const optInv = document.createElement("option");
    optInv.value = "-1";
    optInv.textContent = "Invertito";
    sel.appendChild(optNorm);
    sel.appendChild(optInv);

    cell.appendChild(lbl);
    cell.appendChild(sel);
    el.appendChild(cell);
  });
}

// ---------------------------------------------------------------------------
// Lettura / scrittura valori nelle griglie
// ---------------------------------------------------------------------------
function setGridValues(prefix, values) {
  values.forEach((v, i) => {
    const inp = document.getElementById(`${prefix}-${i}`);
    if (inp) inp.value = v;
  });
}

function setDirValues(values) {
  values.forEach((v, i) => {
    const sel = document.getElementById(`dir-${i}`);
    if (sel) sel.value = (v >= 0 ? 1 : -1);
  });
}

function getGridValues(prefix) {
  return JOINTS.map((_, i) => {
    const inp = document.getElementById(`${prefix}-${i}`);
    return inp ? parseInt(inp.value, 10) : 90;
  });
}

function getDirValues() {
  return JOINTS.map((_, i) => {
    const sel = document.getElementById(`dir-${i}`);
    return sel ? (parseInt(sel.value, 10) >= 0 ? 1 : -1) : 1;
  });
}

// ---------------------------------------------------------------------------
// Popola l'intera UI con i dati ricevuti
// ---------------------------------------------------------------------------
function applySettings(data) {
  if (data.offsets) setGridValues("offset", data.offsets);
  if (data.dirs)    setDirValues(data.dirs);
  if (data.home)    setGridValues("home",   data.home);
  if (data.park)    setGridValues("park",   data.park);
  if (data.vr)      setGridValues("vr",     data.vr);

  if (data.vel_max !== undefined) {
    const slider = document.getElementById("param-vel-slider");
    if (slider) slider.value = data.vel_max;
    updateVelText(data.vel_max);
  }

  if (data.profile) {
    const sel = document.getElementById("param-profile");
    if (sel) sel.value = data.profile;
  }

  if (data.ws_port !== undefined) {
    const p = document.getElementById("param-ws-port");
    if (p) p.value = data.ws_port;
  }

  if (data.demo_steps) buildDemoSteps(data.demo_steps);

  Object.assign(_current, data);
}

// ---------------------------------------------------------------------------
// Aggiorna testo derivato velocità
// ---------------------------------------------------------------------------
function updateVelText(vel) {
  const el = document.getElementById("param-vel-text");
  if (el) el.textContent = `${vel} °/s`;
}

// ---------------------------------------------------------------------------
// Mostra feedback inline temporaneo
// ---------------------------------------------------------------------------
function showFeedback(id, ok, msg) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = msg;
  el.className = `save-feedback visible ${ok ? "ok" : "err"}`;
  setTimeout(() => { el.className = "save-feedback"; }, 3000);
}

// ---------------------------------------------------------------------------
// Invia SETPOSE con i valori di una posa
// ---------------------------------------------------------------------------
function applyPose(angles) {
  const vel     = document.getElementById("param-vel-slider")?.value || 50;
  const profile = document.getElementById("param-profile")?.value    || "RTR5";
  const cmd = `SETPOSE ${angles.join(" ")} ${vel} ${profile}`;
  sendCommand("uart", { cmd });
  addLog(`Applica posa: ${cmd}`);
}

// ---------------------------------------------------------------------------
// Salva un singolo campo posa
// ---------------------------------------------------------------------------
function savePoseField(key, angles, feedbackId) {
  const updated = { ...collectCurrentSettings(), [key]: angles };
  sendCommand("save_settings", updated);
  _pendingSaveFeedback = feedbackId;
}

let _pendingSaveFeedback = null;
let _jointLimits = {
  base: { min: 10, max: 170 },
  spalla: { min: 10, max: 170 },
  gomito: { min: 10, max: 170 },
  yaw: { min: 10, max: 170 },
  pitch: { min: 60, max: 120 },
  roll: { min: 60, max: 120 },
};

// ---------------------------------------------------------------------------
// Raccoglie i valori attuali dell'intera UI come oggetto settings
// ---------------------------------------------------------------------------
function collectCurrentSettings() {
  return {
    offsets:    getGridValues("offset"),
    dirs:       getDirValues(),
    home:       getGridValues("home"),
    park:       getGridValues("park"),
    vr:         getGridValues("vr"),
    vel_max:    parseInt(document.getElementById("param-vel-slider")?.value || 80, 10),
    profile:    document.getElementById("param-profile")?.value || "RTR5",
    demo_steps: getDemoSteps(),
  };
}

async function _loadJointLimitsFromRoutingConfig() {
  try {
    const cfg = await loadRoutingConfig();
    if (!cfg || typeof cfg !== "object" || !cfg.limits) return;
    for (const k of ["base", "spalla", "gomito", "yaw", "pitch", "roll"]) {
      const row = cfg.limits[k];
      if (!row) continue;
      const mn = Number(row.min);
      const mx = Number(row.max);
      if (Number.isFinite(mn) && Number.isFinite(mx)) {
        _jointLimits[k] = { min: mn, max: mx };
      }
    }
  } catch (_) {}
}

function _applyJointLimitsToUi() {
  for (const k of ["base", "spalla", "gomito", "yaw", "pitch", "roll"]) {
    const minEl = document.getElementById(`lim-${k}-min`);
    const maxEl = document.getElementById(`lim-${k}-max`);
    if (minEl) minEl.value = _jointLimits[k].min;
    if (maxEl) maxEl.value = _jointLimits[k].max;
  }
}

function _collectJointLimitsFromUi() {
  const out = {};
  for (const k of ["base", "spalla", "gomito", "yaw", "pitch", "roll"]) {
    const minEl = document.getElementById(`lim-${k}-min`);
    const maxEl = document.getElementById(`lim-${k}-max`);
    const minV = parseInt(minEl?.value ?? "0", 10);
    const maxV = parseInt(maxEl?.value ?? "180", 10);
    if (!Number.isFinite(minV) || !Number.isFinite(maxV) || minV < 0 || maxV > 180 || minV >= maxV) {
      return { ok: false, error: `Limiti non validi per ${k.toUpperCase()}` };
    }
    out[k] = { min: minV, max: maxV };
  }
  return { ok: true, limits: out };
}

async function _saveAndApplyJointLimits() {
  const chk = _collectJointLimitsFromUi();
  if (!chk.ok) return { ok: false, error: chk.error };
  try {
    await saveRoutingConfig({ limits: chk.limits, savedAt: new Date().toISOString() });
    _jointLimits = chk.limits;
    const sent = sendCommand("apply_saved_vr_config", {});
    if (!sent) {
      return { ok: false, error: "WebSocket non connesso: apply al firmware non inviato" };
    }
    return { ok: true };
  } catch (e) {
    return { ok: false, error: String(e) };
  }
}

// ---------------------------------------------------------------------------
// Demo sequence — costruzione e lettura righe
// ---------------------------------------------------------------------------

function buildDemoSteps(steps) {
  const container = document.getElementById("demo-steps-container");
  if (!container) return;
  container.innerHTML = "";
  steps.forEach((step, idx) => addDemoStepRow(container, step, idx));
  _renumberDemoSteps();
}

function addDemoStepRow(container, step = DEMO_STEP_DEFAULT, idx = -1) {
  const row = document.createElement("div");
  row.className = "demo-step-row";
  row.dataset.stepIdx = idx >= 0 ? idx : container.children.length;

  // Numero step
  const num = document.createElement("span");
  num.className = "demo-step-num";
  num.textContent = (idx >= 0 ? idx : container.children.length) + 1;

  // Griglia 6 angoli
  const grid = document.createElement("div");
  grid.className = "angle-grid";
  JOINTS.forEach((label, i) => {
    const cell = document.createElement("div");
    cell.className = "angle-cell";
    const lbl = document.createElement("label");
    lbl.textContent = label;
    const inp = document.createElement("input");
    inp.type = "number"; inp.min = 0; inp.max = 180; inp.step = 1;
    inp.value = step.angles[i] ?? 90;
    inp.className = `demo-angle`;
    cell.appendChild(lbl); cell.appendChild(inp);
    grid.appendChild(cell);
  });

  // Controlli: vel + profilo + elimina
  const controls = document.createElement("div");
  controls.className = "demo-step-controls";

  const velLabel = document.createElement("span");
  velLabel.className = "demo-col-label";
  velLabel.textContent = "Vel. (°/s)";

  const velInp = document.createElement("input");
  velInp.type = "number"; velInp.min = 1; velInp.max = 120; velInp.step = 1;
  velInp.value = step.vel ?? 40;
  velInp.title = "Velocità (°/s)";
  velInp.className = "demo-vel";

  const profSel = document.createElement("select");
  profSel.className = "demo-profile";
  PROFILES.forEach(p => {
    const opt = document.createElement("option");
    opt.value = p; opt.textContent = p;
    if (p === (step.profile ?? "RTR5")) opt.selected = true;
    profSel.appendChild(opt);
  });

  const delBtn = document.createElement("button");
  delBtn.className = "btn-del";
  delBtn.textContent = "✕";
  delBtn.title = "Rimuovi step";
  delBtn.addEventListener("click", () => {
    row.remove();
    _renumberDemoSteps();
  });

  controls.appendChild(velLabel);
  controls.appendChild(velInp);
  controls.appendChild(profSel);
  controls.appendChild(delBtn);

  row.appendChild(num);
  row.appendChild(grid);
  row.appendChild(controls);
  container.appendChild(row);
}

function _renumberDemoSteps() {
  const container = document.getElementById("demo-steps-container");
  if (!container) return;
  [...container.children].forEach((row, i) => {
    const num = row.querySelector(".demo-step-num");
    if (num) num.textContent = i + 1;
    row.dataset.stepIdx = i;
  });
}

function getDemoSteps() {
  const container = document.getElementById("demo-steps-container");
  if (!container) return [];
  return [...container.children].map(row => {
    const angleInputs = row.querySelectorAll(".demo-angle");
    const angles = [...angleInputs].map(inp => parseInt(inp.value, 10) || 90);
    const vel = parseInt(row.querySelector(".demo-vel")?.value || 40, 10);
    const profile = row.querySelector(".demo-profile")?.value || "RTR5";
    return { angles, vel, profile };
  });
}

// ---------------------------------------------------------------------------
// Collasso/espansione pannelli
// ---------------------------------------------------------------------------
function initCollapse() {
  document.querySelectorAll(".settings-section-header").forEach(header => {
    header.addEventListener("click", () => {
      const targetId = header.dataset.target;
      const body = document.getElementById(targetId);
      if (!body) return;
      const collapsed = header.classList.toggle("collapsed");
      body.style.display = collapsed ? "none" : "";
    });
  });
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
function init() {
  // Costruisce le griglie prima del collegamento WS
  buildAngleGrid("offsets-grid", "offset", false);   // modificabili: ora gli offset si possono inviare al firmware
  buildDirGrid("dirs-grid");
  buildAngleGrid("home-grid",    "home",   false);
  buildAngleGrid("park-grid",    "park",   false);
  buildAngleGrid("vr-grid",      "vr",     false);

  // Popola demo con i default finché non arrivano i settings dal server
  buildDemoSteps(_current.demo_steps || [DEMO_STEP_DEFAULT]);

  initCollapse();
  initRobotPoeSection();

  // Selettore profilo video MediaMTX (riavvia jonny5-mediamtx via backend)
  document.getElementById("settings-video-profile-apply")?.addEventListener("click", () => {
    const sel = document.getElementById("settings-video-profile");
    const status = document.getElementById("settings-video-profile-status");
    const btn = document.getElementById("settings-video-profile-apply");
    if (!sel) return;
    const profile = String(sel.value || "lowlatency");
    if (status) status.textContent = "applicazione " + profile + "...";
    if (btn) btn.disabled = true;
    sendCommand("set_video_profile", { profile: profile });
    addLog("Profilo video richiesto: " + profile);
    setTimeout(() => {
      if (btn) btn.disabled = false;
      if (status) status.textContent = "attivo: " + profile;
    }, 7000);
  });

  // Slider velocità → testo derivato
  const velSlider = document.getElementById("param-vel-slider");
  velSlider?.addEventListener("input", () => updateVelText(velSlider.value));

  // Applica offset meccanici al firmware (SET_OFFSETS UART → permanente finché non si reflasha)
  document.getElementById("btn-apply-offsets")?.addEventListener("click", () => {
    const vals = getGridValues("offset");
    sendCommand("apply_offsets", { offsets: vals });
    _pendingSaveFeedback = "fb-offsets";
    addLog(`Applica offset al firmware: ${vals.join(" ")}`);
  });

  // Salva offset in settings.json (senza inviare al firmware)
  document.getElementById("btn-save-offsets")?.addEventListener("click", () => {
    savePoseField("offsets", getGridValues("offset"), "fb-offsets");
  });

  // Applica pose
  document.getElementById("btn-apply-home")?.addEventListener("click", () =>
    applyPose(getGridValues("home")));
  document.getElementById("btn-apply-park")?.addEventListener("click", () =>
    applyPose(getGridValues("park")));
  document.getElementById("btn-apply-vr")?.addEventListener("click", () =>
    applyPose(getGridValues("vr")));

  // Salva singola posa
  document.getElementById("btn-save-home")?.addEventListener("click", () =>
    savePoseField("home", getGridValues("home"), "fb-home"));
  document.getElementById("btn-save-park")?.addEventListener("click", () =>
    savePoseField("park", getGridValues("park"), "fb-park"));
  document.getElementById("btn-save-vr")?.addEventListener("click", () =>
    savePoseField("vr", getGridValues("vr"), "fb-vr"));

  // Demo: aggiungi step
  document.getElementById("btn-demo-add-step")?.addEventListener("click", () => {
    const container = document.getElementById("demo-steps-container");
    if (!container) return;
    if (container.children.length >= 16) {
      showFeedback("fb-demo", false, "Massimo 16 passi");
      return;
    }
    addDemoStepRow(container);
    _renumberDemoSteps();
  });

  // Demo: salva sequenza
  document.getElementById("btn-save-demo")?.addEventListener("click", () => {
    const steps = getDemoSteps();
    if (steps.length === 0) { showFeedback("fb-demo", false, "Nessuno step"); return; }
    sendCommand("save_settings", { ...collectCurrentSettings(), demo_steps: steps });
    _pendingSaveFeedback = "fb-demo";
  });

  // Salva parametri sistema
  document.getElementById("btn-save-params")?.addEventListener("click", () => {
    sendCommand("save_settings", collectCurrentSettings());
    _pendingSaveFeedback = "fb-params";
  });

  // ── Configurazione PWM Servo ──────────────────────────────────────────────
  const PWM_FIELDS = [
    "pwm-tim8-hz", "pwm-tim8-min-us", "pwm-tim8-max-us", "pwm-tim8-max-deg",
    "pwm-tim1-hz", "pwm-tim1-min-us", "pwm-tim1-max-us", "pwm-tim1-max-deg",
  ];
  const PWM_KEYS = [
    "tim8_hz", "tim8_min_us", "tim8_max_us", "tim8_max_deg",
    "tim1_hz", "tim1_min_us", "tim1_max_us", "tim1_max_deg",
  ];
  const PWM_DEFAULTS = { tim8_hz:50,tim8_min_us:500,tim8_max_us:2500,tim8_max_deg:180,
                         tim1_hz:50,tim1_min_us:500,tim1_max_us:2500,tim1_max_deg:180 };

  function _readPwmConfig() {
    const cfg = {};
    PWM_FIELDS.forEach((id, i) => {
      const el = document.getElementById(id);
      cfg[PWM_KEYS[i]] = el ? (parseInt(el.value, 10) || PWM_DEFAULTS[PWM_KEYS[i]]) : PWM_DEFAULTS[PWM_KEYS[i]];
    });
    return cfg;
  }

  function _applyPwmConfigToUi(cfg) {
    if (!cfg) return;
    PWM_FIELDS.forEach((id, i) => {
      const el = document.getElementById(id);
      if (el && cfg[PWM_KEYS[i]] !== undefined) el.value = cfg[PWM_KEYS[i]];
    });
  }

  // La config PWM viene caricata via WS (vedi handler pwm_config sotto).

  document.getElementById("btn-apply-pwm")?.addEventListener("click", () => {
    const cfg = _readPwmConfig();
    sendCommand("save_pwm_config", { config: cfg });
    addLog(`PWM config → firmware: TIM8 ${cfg.tim8_hz}Hz ${cfg.tim8_min_us}-${cfg.tim8_max_us}µs ${cfg.tim8_max_deg}° | TIM1 ${cfg.tim1_hz}Hz ${cfg.tim1_min_us}-${cfg.tim1_max_us}µs ${cfg.tim1_max_deg}°`);
    // Feedback immediato di "in corso" (sostituito dall'ack quando arriva)
    showFeedback("fb-pwm", true, "⏳ Invio al firmware…");
  });

  // ── Pick & Place — Test attuatori (PA0/PA1 → IRFZ44N) ─────────────────────
  // Comandi UART: PP1 <0..100>, PP2 <0..100>, PP?
  // Debounce sullo slider input per non floodare il firmware durante il drag.
  function _ppSendDuty(channel, duty) {
    const d = Math.max(0, Math.min(100, parseInt(duty, 10) || 0));
    sendCommand("uart", { cmd: `PP${channel} ${d}` });
    addLog(`Pick&Place PP${channel} duty=${d}%`);
  }

  function _bindPpChannel(channel) {
    const slider = document.getElementById(`pp${channel}-slider`);
    const text   = document.getElementById(`pp${channel}-text`);
    const btnOn  = document.getElementById(`btn-pp${channel}-on`);
    const btnOff = document.getElementById(`btn-pp${channel}-off`);
    let lastSent = -1;
    let debounceTimer = null;

    const updateText = (v) => { if (text) text.textContent = `${v} %`; };

    slider?.addEventListener("input", () => {
      updateText(slider.value);
      clearTimeout(debounceTimer);
      debounceTimer = setTimeout(() => {
        const v = parseInt(slider.value, 10);
        if (v !== lastSent) { _ppSendDuty(channel, v); lastSent = v; }
      }, 120);
    });
    // change = rilascio slider → invio immediato (oltre al debounce)
    slider?.addEventListener("change", () => {
      const v = parseInt(slider.value, 10);
      if (v !== lastSent) { _ppSendDuty(channel, v); lastSent = v; }
    });

    btnOn?.addEventListener("click", () => {
      if (slider) { slider.value = 100; updateText(100); }
      _ppSendDuty(channel, 100); lastSent = 100;
    });
    btnOff?.addEventListener("click", () => {
      if (slider) { slider.value = 0; updateText(0); }
      _ppSendDuty(channel, 0); lastSent = 0;
    });
  }

  _bindPpChannel(1);
  _bindPpChannel(2);

  document.getElementById("btn-pp-stop-all")?.addEventListener("click", () => {
    ["1", "2"].forEach(ch => {
      const slider = document.getElementById(`pp${ch}-slider`);
      const text   = document.getElementById(`pp${ch}-text`);
      if (slider) slider.value = 0;
      if (text)   text.textContent = "0 %";
      sendCommand("uart", { cmd: `PP${ch} 0` });
    });
    showFeedback("fb-pickplace", true, "Entrambi i canali a 0");
    addLog("Pick&Place: STOP entrambi i canali");
  });

  document.getElementById("btn-pp-status")?.addEventListener("click", () => {
    sendCommand("uart", { cmd: "PP?" });
    addLog("Pick&Place: richiesta stato (PP?)");
  });

  // Handler risposte firmware per comandi PP* — mostra nel feedback box e log.
  // Payload: {type:"uart_response", cmd:<echo cmd_upper>, ok, response, warning}
  registerUartResponseHandler((msg) => {
    const cmd = String(msg?.cmd || "").trim().toUpperCase();
    if (!cmd.startsWith("PP")) return;
    const ok = msg.ok === true;
    const resp = String(msg.response || "").trim();
    showFeedback("fb-pickplace", ok, ok ? `← ${resp || "OK"}` : `ERR: ${resp || "no response"}`);
    addLog(`[PP] ${cmd} → ok=${ok} resp=${resp}`);
  });

  document.getElementById("btn-save-joint-limits")?.addEventListener("click", async () => {
    const out = await _saveAndApplyJointLimits();
    showFeedback(
      "fb-joint-limits",
      out.ok,
      out.ok ? "Limiti globali salvati e applicati" : "Errore nel salvataggio dei limiti globali"
    );
    addLog(out.ok ? "Limiti giunti globali aggiornati" : `Errore limiti giunti globali: ${out.error}`);
  });

  // Connessione WS e handler settings
  connectJ5Dashboard();

  registerSettingsHandler(msg => {
    if (msg.type === "settings") {
      applySettings(msg);
      addLog("Impostazioni caricate dal server");
    } else if (msg.type === "settings_saved") {
      const fbId = _pendingSaveFeedback;
      _pendingSaveFeedback = null;
      if (fbId) {
        showFeedback(fbId, msg.ok, msg.ok ? "Salvato" : "Errore nel salvataggio");
      }
      if (msg.ok) addLog("Impostazioni salvate");
      else        addLog("Errore: impostazioni non salvate");
    } else if (msg.type === "pwm_config") {
      _applyPwmConfigToUi(msg.config);
      addLog("PWM config caricata da Pi");
    } else if (msg.type === "pwm_config_applied") {
      const ok    = msg.ok === true;
      const saved = msg.saved === true;
      let txt;
      if (ok && saved)        txt = "✓ Applicato al firmware e salvato";
      else if (ok && !saved)  txt = "⚠ Applicato al firmware ma NON salvato (file non scrivibile)";
      else if (!ok && saved)  txt = "⚠ Salvato su Pi ma firmware NON risponde";
      else                    txt = "✗ Errore: firmware NON risponde";
      showFeedback("fb-pwm", ok, txt);
      addLog(`[PWM] ${txt} (ok=${ok}, saved=${saved})`);
    } else if (msg.type === "offsets_applied") {
      const fbId = _pendingSaveFeedback;
      _pendingSaveFeedback = null;
      const ok = msg.ok === true;
      if (fbId) {
        showFeedback(fbId, ok, ok ? "Offset applicati al firmware" : `Errore: ${msg.error || msg.response || "UART fallito"}`);
      }
      if (ok) {
        // Aggiorna snapshot locale con i valori confermati
        if (msg.offsets) {
          Object.assign(_current, { offsets: msg.offsets });
          setGridValues("offset", msg.offsets);
        }
        addLog(`Offset firmware aggiornati: ${(msg.offsets || []).join(" ")}`);
      } else {
        addLog(`Errore applicazione offset: ${msg.error || msg.response || "?"}`);
      }
    }
  });

  // Richiedi settings al server appena WS è aperto
  // Ritenta ogni 500ms finché il WS non è connesso (al massimo 5 tentativi)
  let attempts = 0;
  const tryGet = () => {
    attempts++;
    try {
      sendCommand("get_settings", {});
      if (attempts === 1) {
        sendCommand("get_poe_params", {});
        sendCommand("get_pwm_config", {});
      }
    } catch (_) {}
    if (attempts < 5) setTimeout(tryGet, 600);
  };
  // Piccolo ritardo per dare tempo al WS di aprirsi
  setTimeout(tryGet, 400);

  _loadJointLimitsFromRoutingConfig().then(_applyJointLimitsToUi);
}

document.addEventListener("DOMContentLoaded", init);
