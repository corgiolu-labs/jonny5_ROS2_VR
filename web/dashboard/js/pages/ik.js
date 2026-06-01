/**
 * ik.js — Pagina cinematica FK → IK per JONNY5 Dashboard
 *
 * Solo modello POE: FK `compute_fk_poe`, IK `IK_SOLVE` con solver POE (ottimizzazione numerica sul POE del controller).
 */

import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerIkResultHandler,
  registerFkPoeResultHandler,
  registerSettingsHandler,
  registerPoeParamsHandler,
  registerUartResponseHandler,
  registerSetposeDoneHandler,
  sendCommand,
  addLog,
  quatToEuler,
  loadRoutingConfig,
} from "../../../shared/js/j5_common.js";

const POE_STORAGE_KEY = "j5_poe_params";
const POE_MIGRATE_FLAG = "j5_poe_migrate_done";
const IK_TARGET_STORAGE_KEY = "j5_ik_target";
const JOINT_NAMES = ["B", "S", "G", "Y", "P", "R"];
/** Etichette risultato IK: stesso ordine B S G Y P R → nomi estesi. */
const JOINT_LABELS_IT = ["Base", "Spalla", "Gomito", "Yaw", "Pitch", "Roll"];
/** Ordine limiti = backend `routing_config.json` / `ik_solver._JOINT_LIMITS_ORDER`. */
const JOINT_LIMIT_KEYS = ["base", "spalla", "gomito", "yaw", "pitch", "roll"];
const FK_INPUT_IDS = ["fk-base", "fk-spalla", "fk-gomito", "fk-yaw", "fk-pitch", "fk-roll"];
const IK_COMPARE_FIELDS = [
  { key: "x", label: "X", unit: "mm", kind: "pos" },
  { key: "y", label: "Y", unit: "mm", kind: "pos" },
  { key: "z", label: "Z", unit: "mm", kind: "pos" },
  { key: "yaw", label: "Yaw", unit: "°", kind: "rot" },
  { key: "pitch", label: "Pitch", unit: "°", kind: "rot" },
  { key: "roll", label: "Roll", unit: "°", kind: "rot" },
];
const IK_COMPARE_CARD_META = {
  target: {
    className: "target",
    title: "🎯 Posa teorica (IK)",
    note: "Dove il robot dovrebbe trovarsi",
  },
  real: {
    className: "real",
    title: "🤖 Posa reale (IMU)",
    note: "Dove il robot si trova davvero",
  },
  error: {
    className: "error",
    title: "⚠️ Errore di posa",
    note: "Δ = reale − teorica",
  },
};

// Soglie per colorare / qualificare l'errore nel card "Errore di posa".
// Le soglie sono applicate per asse (e alla norma cumulativa) e sono quelle
// indicate per la presentazione tesi:
//   posizione  < 5 mm  verde   < 20 mm  giallo   ≥ 20 mm  rosso
//   angoli     < 2°    verde   < 8°     giallo   ≥ 8°     rosso
const ERROR_POS_GREEN_MM = 5;
const ERROR_POS_YELLOW_MM = 20;
const ERROR_ROT_GREEN_DEG = 2;
const ERROR_ROT_YELLOW_DEG = 8;

function _errorLevel(absVal, greenThr, yellowThr) {
  if (!Number.isFinite(absVal)) return "idle";
  if (absVal < greenThr) return "green";
  if (absVal < yellowThr) return "yellow";
  return "red";
}
const IMU_GRAVITY_MPS2 = 9.80665;
const IMU_ACCEL_DEADBAND_MPS2 = 0.18;
const IMU_STILL_GYRO_RAD_S = 0.12;
const IMU_MAX_DT_S = 0.08;
const IMU_ACTIVE_DAMP = 0.985;
const IMU_STILL_DAMP = 0.42;
const IMU_MAX_SPEED_MPS = 0.45;
const TOOL_OFFSET_M = [0.06, 0.0, 0.0];

/**
 * IMU → base-frame rotation calibration (read-only from backend).
 *
 * Physical model identical to the analytics validators
 * (raspberry/controller/imu_analytics/validate_imu_vs_ee.py):
 *
 *     R_imu = R_world_bias · R_ee · R_mount
 *
 *   - R_mount       : mechanical IMU-chip-to-EE rigid offset (roll/pitch,
 *                     yaw forced to 0)
 *   - R_world_bias  : BNO085 Rotation-Vector yaw reference offset
 *                     (magnetic-north → robot base yaw alignment)
 *   - R_ee          : end-effector (tool) orientation in robot base frame
 *                     — the same space FK live and target IK use.
 *
 * Inverting for observability:
 *
 *     R_ee = R_world_bias^-1 · R_imu · R_mount^-1
 *
 * That is the rotation we display in the IMU card, and the one we apply to
 * TOOL_OFFSET to position the IMU-derived tool-tip in base frame.
 *
 * Defaults: identity quaternions. If /api/imu-frame-calib is absent, or the
 * JSON configs are missing on the Pi, this collapses to R_ee ≡ R_imu so the
 * previous behavior is preserved (no regression for unconfigured rigs).
 */
const IDENTITY_QUAT_WXYZ = { w: 1, x: 0, y: 0, z: 0 };
let _imuFrameCalib = {
  mountQuat: { ...IDENTITY_QUAT_WXYZ },      // R_mount
  worldBiasQuat: { ...IDENTITY_QUAT_WXYZ },  // R_world_bias
  // R_home = q_observed_home ⊗ q_fk_home_conj — capturato con "Azzera IMU @ HOME".
  // Optional: se imu_home_ref.json è assente → identity → comportamento pre-fix.
  homeQuat: { ...IDENTITY_QUAT_WXYZ },
  mountPresent: false,
  worldBiasPresent: false,
  homePresent: false,
  homeCalibratedAt: null,
  loaded: false,
};

// Ultima telemetria valida ricevuta — usata al click "Azzera IMU @ HOME"
// per catturare q_imu e fk_live quat al momento del click (senza ulteriore WS).
let _lastTelemetryForZero = null;

/** Finestra stabilizzazione dopo SETPOSE_DONE (o fallback timeout) prima del prime. */
const IK_COMPARE_SETTLE_MS = 300;
/** Se SETPOSE_DONE non arriva, si passa comunque a stabilizzazione (evita stallo). */
const IK_COMPARE_MOVE_TIMEOUT_MS = 15000;

/** Macchina a stati confronto IK / stima IMU (solo timing del prime, non il modello). */
const IK_COMPARE_PHASE = {
  IDLE: "IDLE",
  WAIT_MOVE_DONE: "WAIT_MOVE_DONE",
  WAIT_SETTLE: "WAIT_SETTLE",
  READY_TO_PRIME: "READY_TO_PRIME",
  TRACKING: "TRACKING",
};

/** Timer wall-clock per uscire da WAIT_SETTLE senza dipendere dalla frequenza telemetry. */
let _ikCompareSettleTimerId = null;

const DEFAULT_JOINT_LIMITS_VIRTUAL = {
  base: { min: 10, max: 170 },
  spalla: { min: 10, max: 170 },
  gomito: { min: 10, max: 170 },
  yaw: { min: 10, max: 170 },
  pitch: { min: 60, max: 120 },
  roll: { min: 60, max: 120 },
};

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
const MAX_POS_ERROR_DEFAULT = 10.0;
const MAX_ORI_ERROR_DEFAULT = 10.0;

// Settings ricevuti dal server (vel_max e profilo per SETPOSE)
let _ikVel     = 40;
let _ikProfile = "RTR5";

// Unified "Invia posa IK": se true, il prossimo ik_result reachable
// triggera automaticamente SETPOSE. Cleared dopo l'auto-send o su fail.
let _ikAutoSendPending = false;

// Angoli giunto della Sezione 1 al momento di "Calcola FK" — usati come
// riferimento per il verdict "round-trip FK→IK" nella Sezione 3 (confronta
// con gli angoli ricostruiti dall'IK solver sulla posa FK).
let _roundTripInputJoints = null;  // [b, s, g, y, p, r] in gradi virtuali
let _ikCompare = {
  activeTarget: null,
  targetSentAtMs: 0,
  comparePhase: IK_COMPARE_PHASE.IDLE,
  moveWaitStartMs: 0,
  settleStartMs: null,
  fkLivePose: null,
  imuEstimatePose: null,
  estimatorPrimed: false,
  estWcPosM: null,
  estVelMps: [0, 0, 0],
  lastTelemetryTsMs: null,
  lastImuSampleCounter: null,
  lastImuRateHz: null,
  lastImuValid: false,
  // True quando la telemetria IMU pubblica accel_raw (integratore inerziale
  // possibile). False con BNO085 v1 (solo Rotation Vector) → usiamo FK wc
  // come ancora e l'IMU solo per orientazione, evitando il drift di -g.
  imuAccelRawAvailable: false,
};

/** Ultimo POE ricevuto dal backend (source of truth). */
let _poeFromServer = null;

function _poeCfgEqualsIk(a, b) {
  if (!a?.S || !b?.S || !a?.M || !b?.M) return false;
  try {
    return JSON.stringify(a.S) === JSON.stringify(b.S) && JSON.stringify(a.M) === JSON.stringify(b.M);
  } catch (_) {
    return false;
  }
}

function loadPoeFromLocalOnly() {
  try {
    const raw = localStorage.getItem(POE_STORAGE_KEY);
    if (!raw) return JSON.parse(JSON.stringify(POE_DEFAULT));
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed?.S) && parsed.S.length === 6 && Array.isArray(parsed?.M) && parsed.M.length === 4) {
      return parsed;
    }
  } catch (_) {}
  return JSON.parse(JSON.stringify(POE_DEFAULT));
}

function resolvePoeCfgFromServerMessageIk(msg) {
  let cfg = { S: msg.S, M: msg.M };
  if (msg.persisted === false && sessionStorage.getItem(POE_MIGRATE_FLAG) !== "1") {
    const ls = loadPoeFromLocalOnly();
    const def = JSON.parse(JSON.stringify(POE_DEFAULT));
    if (!_poeCfgEqualsIk(ls, def)) {
      sendCommand("set_poe_params", { S: ls.S, M: ls.M });
      sessionStorage.setItem(POE_MIGRATE_FLAG, "1");
      addLog("POE: migrazione browser → Raspberry");
      cfg = { S: ls.S, M: ls.M };
    } else {
      sessionStorage.setItem(POE_MIGRATE_FLAG, "1");
    }
  }
  return cfg;
}

function savePoeLocalMirror(cfg) {
  localStorage.setItem(POE_STORAGE_KEY, JSON.stringify({ S: cfg.S, M: cfg.M }));
}

function _copyLimits(src) {
  const o = {};
  for (const k of JOINT_LIMIT_KEYS) {
    o[k] = { min: Number(src[k].min), max: Number(src[k].max) };
  }
  return o;
}

/** Copia mutabile; allineata a `routing_config.json` dopo `loadJointLimitsFromBackend()`. */
let _jointLimitsVirtual = _copyLimits(DEFAULT_JOINT_LIMITS_VIRTUAL);

function clampVirtualJoint(jointKey, deg) {
  const L = _jointLimitsVirtual[jointKey] || { min: 0, max: 180 };
  const v = Number(deg);
  if (!Number.isFinite(v)) return L.min;
  return Math.max(L.min, Math.min(L.max, v));
}

/** 6 angoli virtuali in ordine B S G Y P R. */
function clampAnglesBsgYpr(angles) {
  if (!Array.isArray(angles) || angles.length !== 6) return angles;
  return JOINT_LIMIT_KEYS.map((k, i) => clampVirtualJoint(k, angles[i]));
}

function anglesClampedFromRaw(raw) {
  const c = clampAnglesBsgYpr(raw);
  let changed = false;
  for (let i = 0; i < 6; i++) {
    if (Math.abs(c[i] - Number(raw[i])) > 0.05) {
      changed = true;
      break;
    }
  }
  return { clamped: c, changed };
}

function refreshJointLimitsSummaryText() {
  const el = document.getElementById("ik-limits-summary-fk");
  if (!el) return;
  const parts = JOINT_LIMIT_KEYS.map((k) => {
    const L = _jointLimitsVirtual[k];
    const label = JOINT_LABELS_IT[JOINT_LIMIT_KEYS.indexOf(k)] || k;
    return `${label} [${L.min}–${L.max}°]`;
  });
  el.textContent = `Limiti giunto virtuali (routing_config): ${parts.join(" · ")}`;
}

function applyFkInputAttrLimits() {
  JOINT_LIMIT_KEYS.forEach((k, i) => {
    const inp = document.getElementById(FK_INPUT_IDS[i]);
    if (!inp) return;
    const L = _jointLimitsVirtual[k];
    inp.min = L.min;
    inp.max = L.max;
  });
}

/**
 * Stessi limiti usati da Settings (`/api/routing-config`) e dal clamp UART su Raspberry.
 */
async function loadJointLimitsFromBackend() {
  try {
    const cfg = await loadRoutingConfig();
    if (cfg?.limits && typeof cfg.limits === "object") {
      const next = _copyLimits(_jointLimitsVirtual);
      for (const k of JOINT_LIMIT_KEYS) {
        const row = cfg.limits[k];
        if (!row || typeof row !== "object") continue;
        const mn = Number(row.min);
        const mx = Number(row.max);
        if (Number.isFinite(mn) && Number.isFinite(mx) && mn >= 0 && mx <= 180 && mn < mx) {
          next[k] = { min: mn, max: mx };
        }
      }
      _jointLimitsVirtual = next;
    }
  } catch (_) {
    _jointLimitsVirtual = _copyLimits(DEFAULT_JOINT_LIMITS_VIRTUAL);
  }
  refreshJointLimitsSummaryText();
  applyFkInputAttrLimits();
}

// ---------------------------------------------------------------------------
// Persistenza target cartesiano (X,Y,Z,Roll,Pitch,Yaw)
// ---------------------------------------------------------------------------
function collectTarget() {
  const getNum = (id) => parseFloat(document.getElementById(id)?.value ?? "0") || 0;
  return {
    x: getNum("ik-x"),
    y: getNum("ik-y"),
    z: getNum("ik-z"),
    roll: getNum("ik-roll"),
    pitch: getNum("ik-pitch"),
    yaw: getNum("ik-yaw"),
  };
}

function applyTarget(t) {
  if (!t || typeof t !== "object") return;
  const setVal = (id, val) => {
    const el = document.getElementById(id);
    if (el && Number.isFinite(val)) el.value = val;
  };
  setVal("ik-x", t.x);
  setVal("ik-y", t.y);
  setVal("ik-z", t.z);
  setVal("ik-roll", t.roll);
  setVal("ik-pitch", t.pitch);
  setVal("ik-yaw", t.yaw);
}

function loadTarget() {
  try {
    const raw = localStorage.getItem(IK_TARGET_STORAGE_KEY);
    if (!raw) return null;
    const parsed = JSON.parse(raw);
    const ok = ["x", "y", "z", "roll", "pitch", "yaw"].every(
      (k) => Number.isFinite(Number(parsed?.[k]))
    );
    return ok ? parsed : null;
  } catch (_) {
    return null;
  }
}

function saveTarget(target) {
  localStorage.setItem(IK_TARGET_STORAGE_KEY, JSON.stringify(target));
}

// ---------------------------------------------------------------------------
// Costruisce la griglia risultato IK (6 celle placeholder)
// ---------------------------------------------------------------------------
function buildResultGrid() {
  const grid = document.getElementById("ik-result-grid");
  if (!grid) return;
  grid.innerHTML = "";
  JOINT_NAMES.forEach((name, i) => {
    const cell = document.createElement("div");
    cell.className = "ik-value-cell ik-result-cell";
    cell.id = `ik-res-${name.toLowerCase()}`;
    const lab = JOINT_LABELS_IT[i] || name;
    cell.innerHTML = `
      <div class="cell-label">${lab} <span class="short">(${name})</span></div>
      <div class="cell-value" id="ik-res-val-${name.toLowerCase()}">—</div>
    `;
    grid.appendChild(cell);
  });
}

function _num(v) {
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function _normAngleDeg(v) {
  let out = Number(v) || 0;
  while (out > 180) out -= 360;
  while (out < -180) out += 360;
  return out;
}

function _fmtPoseValue(key, value) {
  const n = _num(value);
  if (n === null) return "—";
  const unit = key === "x" || key === "y" || key === "z" ? " mm" : "°";
  return `${n.toFixed(1)}${unit}`;
}

function _isFiniteComparePose(pose) {
  return !!pose && IK_COMPARE_FIELDS.every((field) => Number.isFinite(Number(pose[field.key])));
}

function _vecAdd(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function _vecScale(v, s) {
  return [v[0] * s, v[1] * s, v[2] * s];
}

function _vecNorm(v) {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Hamilton product (q1 ⊗ q2) with wxyz convention. Order matters: returns
// the rotation "first apply q2, then apply q1" (same semantics as
// Rotation.__mul__ in scipy).
function _quatMul(q1, q2) {
  return {
    w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
  };
}

// Inverse of a unit quaternion = conjugate. We renormalize defensively to
// tolerate mild precision drift in the incoming configs.
function _quatConj(q) {
  const n = Math.hypot(q.w, q.x, q.y, q.z) || 1;
  return { w: q.w / n, x: -q.x / n, y: -q.y / n, z: -q.z / n };
}

// Compose q_ee_base (no-home, pipeline "nuda"):
//   q_noHome = q_world_bias^-1 ⊗ q_imu ⊗ q_mount^-1
// Ritorna l'osservazione IMU portata in base frame secondo il solo modello
// del validator. Usata internamente da _imuQuatToBaseFrame e dal capture
// "Azzera IMU @ HOME" — che deve calcolare l'offset q_home INDIPENDENTE-
// MENTE dal q_home corrente (altrimenti la zeroing sarebbe ricorsiva).
function _imuQuatToBaseFrameNoHome(qImuWxyz) {
  const qMountInv = _quatConj(_imuFrameCalib.mountQuat);
  const qWorldBiasInv = _quatConj(_imuFrameCalib.worldBiasQuat);
  return _quatMul(_quatMul(qWorldBiasInv, qImuWxyz), qMountInv);
}

// Versione completa (incl. R_home) usata dal compare-pipeline a runtime:
//   q_ee_base = q_home^-1 ⊗ q_world_bias^-1 ⊗ q_imu ⊗ q_mount^-1
// Se imu_home_ref.json è assente → homeQuat = identity → fall-back al
// comportamento pre-zero (backward-compatible: nessuna correzione).
function _imuQuatToBaseFrame(qImuWxyz) {
  const qNoHome = _imuQuatToBaseFrameNoHome(qImuWxyz);
  const qHomeInv = _quatConj(_imuFrameCalib.homeQuat);
  return _quatMul(qHomeInv, qNoHome);
}

async function _loadImuFrameCalib() {
  try {
    const res = await fetch("/api/imu-frame-calib", { cache: "no-store" });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    const parse = (slot) => {
      const q = slot?.quat_wxyz;
      if (!Array.isArray(q) || q.length !== 4) return null;
      const nums = q.map(Number);
      if (nums.some((v) => !Number.isFinite(v))) return null;
      const n = Math.hypot(...nums) || 1;
      return { w: nums[0] / n, x: nums[1] / n, y: nums[2] / n, z: nums[3] / n };
    };
    const mount = parse(data?.mount);
    const worldBias = parse(data?.world_bias);
    const home = parse(data?.home);
    if (mount) _imuFrameCalib.mountQuat = mount;
    if (worldBias) _imuFrameCalib.worldBiasQuat = worldBias;
    if (home) _imuFrameCalib.homeQuat = home;
    else _imuFrameCalib.homeQuat = { ...IDENTITY_QUAT_WXYZ };  // reset se rimosso
    _imuFrameCalib.mountPresent = !!data?.mount?.present;
    _imuFrameCalib.worldBiasPresent = !!data?.world_bias?.present;
    _imuFrameCalib.homePresent = !!data?.home?.present;
    _imuFrameCalib.homeCalibratedAt = data?.home?.calibrated_at || null;
    _imuFrameCalib.loaded = true;
    const dm = _imuFrameCalib.mountPresent ? "configured" : "identity";
    const db = _imuFrameCalib.worldBiasPresent ? "configured" : "identity";
    const dh = _imuFrameCalib.homePresent ? `configured (${_imuFrameCalib.homeCalibratedAt || "?"})` : "identity (non settato)";
    addLog(`IMU frame calib: mount=${dm}, world_bias=${db}, home=${dh}`);
    _updateZeroHomeStatusLabel();
  } catch (e) {
    _imuFrameCalib.loaded = false;
    addLog(`⚠ IMU frame calib non disponibile: compare IMU in IMU-world frame (${String(e)})`);
  }
}

/** Aggiorna l'etichetta dello status "Zero @ HOME attivo / non settato". */
function _updateZeroHomeStatusLabel() {
  const el = document.getElementById("zero-home-status");
  if (!el) return;
  if (_imuFrameCalib.homePresent) {
    el.innerHTML = `<span class="state-dot state-dot-on">●</span> Zero-at-HOME attivo${_imuFrameCalib.homeCalibratedAt ? ` · ${_imuFrameCalib.homeCalibratedAt}` : ""}`;
  } else {
    el.innerHTML = `<span class="state-dot state-dot-off">●</span> Zero-at-HOME non impostato (pipeline nuda)`;
  }
}

/**
 * Cattura lo snapshot IMU+FK al click e salva q_home_ref = q_observed · q_fk_conj.
 * Formula: al momento T (robot fermo a HOME):
 *    q_observed(T) = q_wb^-1 ⊗ q_imu(T) ⊗ q_mount^-1        (NO home chain)
 *    q_fk(T)       = fk_live_quat del robot a HOME
 *    q_home_ref    = q_observed(T) ⊗ q_fk(T)^-1
 * Il runtime usa q_ee = q_home_ref^-1 ⊗ q_observed, che a HOME ridà q_fk(T)
 * e — sotto l'ipotesi di drift costante (solo yaw mag) — ridà q_fk per
 * qualsiasi altra posa.
 */
async function _captureAndSaveHomeRef() {
  const btn = document.getElementById("btn-zero-at-home");
  const msg = _lastTelemetryForZero;
  if (!msg) {
    addLog("✗ Zero-at-HOME: nessuna telemetria disponibile. Aspetta un frame IMU e riprova.");
    return;
  }
  if (msg.imu_valid !== true || msg.fk_live_valid !== true) {
    addLog("✗ Zero-at-HOME: IMU o FK live non validi. Robot in moto? Riprova fermo.");
    return;
  }
  // Soft-check: robot near HOME. L'EE a HOME del modello POE ha orientazione
  // identity (yaw=pitch=roll=0); se l'utente ha premuto HOME, FK dovrebbe
  // riportarlo. Tolleranza ampia (5°) per assorbire settling servo.
  const yaw   = Number(msg.fk_live_yaw);
  const pitch = Number(msg.fk_live_pitch);
  const roll  = Number(msg.fk_live_roll);
  if (![yaw, pitch, roll].every(Number.isFinite)) {
    addLog("✗ Zero-at-HOME: FK pose incompleta.");
    return;
  }
  if (Math.abs(yaw) > 5 || Math.abs(pitch) > 5 || Math.abs(roll) > 5) {
    if (!confirm(`Il robot NON sembra a HOME (FK YPR = ${yaw.toFixed(1)}°, ${pitch.toFixed(1)}°, ${roll.toFixed(1)}°). Procedere comunque?`)) {
      addLog("⌫ Zero-at-HOME annullato dall'utente");
      return;
    }
  }

  // q_observed (NO home chain)
  const qImu = _extractImuQuat(msg);
  if (!qImu) { addLog("✗ Zero-at-HOME: quaternion IMU mancante"); return; }
  const qObs = _imuQuatToBaseFrameNoHome(qImu);

  // q_fk live
  const qFk = {
    w: Number(msg.fk_live_quat_w),
    x: Number(msg.fk_live_quat_x),
    y: Number(msg.fk_live_quat_y),
    z: Number(msg.fk_live_quat_z),
  };
  if (![qFk.w, qFk.x, qFk.y, qFk.z].every(Number.isFinite)) {
    addLog("✗ Zero-at-HOME: fk_live_quat mancante");
    return;
  }
  // Normalize to be safe.
  const fn = Math.hypot(qFk.w, qFk.x, qFk.y, qFk.z) || 1;
  const qFkN = { w: qFk.w/fn, x: qFk.x/fn, y: qFk.y/fn, z: qFk.z/fn };

  // q_home_ref = q_obs ⊗ q_fk^-1
  const qHomeRef = _quatMul(qObs, _quatConj(qFkN));

  const payload = {
    quat_wxyz: [qHomeRef.w, qHomeRef.x, qHomeRef.y, qHomeRef.z],
    calibrated_at: new Date().toISOString(),
    fk_pose_mm: [
      Number(msg.fk_live_x_mm), Number(msg.fk_live_y_mm), Number(msg.fk_live_z_mm),
      yaw, pitch, roll,
    ],
    note: "Captured via FK/IK dashboard · observability-layer zero",
  };

  if (btn) btn.disabled = true;
  try {
    const res = await fetch("/api/imu-home-ref", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data?.error || `HTTP ${res.status}`);
    }
    addLog(`✓ Zero-at-HOME salvato (${data.path})`);
    await _loadImuFrameCalib();
    renderIkCompare();
  } catch (e) {
    addLog(`✗ Zero-at-HOME errore: ${String(e)}`);
  } finally {
    if (btn) btn.disabled = false;
  }
}

async function _resetHomeRef() {
  if (!confirm("Rimuovere la correzione Zero-at-HOME? (torna al comportamento senza zero)")) return;
  const btn = document.getElementById("btn-reset-zero-home");
  if (btn) btn.disabled = true;
  try {
    const res = await fetch("/api/imu-home-ref/clear", { method: "POST" });
    const data = await res.json();
    if (!res.ok || !data.ok) throw new Error(data?.error || `HTTP ${res.status}`);
    addLog(`✓ Zero-at-HOME rimosso (removed=${data.removed})`);
    await _loadImuFrameCalib();
    renderIkCompare();
  } catch (e) {
    addLog(`✗ Reset Zero-at-HOME errore: ${String(e)}`);
  } finally {
    if (btn) btn.disabled = false;
  }
}

function initZeroAtHomeButtons() {
  const btn = document.getElementById("btn-zero-at-home");
  const rst = document.getElementById("btn-reset-zero-home");
  if (btn) btn.addEventListener("click", _captureAndSaveHomeRef);
  if (rst) rst.addEventListener("click", _resetHomeRef);
}

function _quatWxyzToMatrix3(w, x, y, z) {
  const n = Math.hypot(w, x, y, z) || 1;
  const qw = w / n;
  const qx = x / n;
  const qy = y / n;
  const qz = z / n;
  const xx = qx * qx;
  const yy = qy * qy;
  const zz = qz * qz;
  const xy = qx * qy;
  const xz = qx * qz;
  const yz = qy * qz;
  const wx = qw * qx;
  const wy = qw * qy;
  const wz = qw * qz;
  return [
    [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
    [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
    [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
  ];
}

function _matVecMul3(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

function _buildImuEstimatePose(wcPosM, quat) {
  if (!Array.isArray(wcPosM) || wcPosM.length !== 3 || !quat) return null;
  // Transform the raw IMU quaternion (in BNO085 world frame) into the robot
  // base frame using the same R_world_bias · R_ee · R_mount model the
  // analytics validators apply. Result qBase is R_ee_from_imu expressed in
  // base coordinates — geometrically comparable to FK live and target IK.
  // If neither mount nor world_bias were loaded, qBase == quat exactly
  // (identity composition), preserving the previous display.
  const qBase = _imuQuatToBaseFrame(quat);
  const rot = _quatWxyzToMatrix3(qBase.w, qBase.x, qBase.y, qBase.z);
  const toolPosM = _vecAdd(wcPosM, _matVecMul3(rot, TOOL_OFFSET_M));
  const euler = quatToEuler(qBase.w, qBase.x, qBase.y, qBase.z);
  const pose = {
    x: toolPosM[0] * 1000.0,
    y: toolPosM[1] * 1000.0,
    z: toolPosM[2] * 1000.0,
    yaw: _normAngleDeg(euler.yaw),
    pitch: _normAngleDeg(euler.pitch),
    roll: _normAngleDeg(euler.roll),
  };
  return _isFiniteComparePose(pose) ? pose : null;
}

function _extractFkLivePose(msg) {
  if (!msg || msg.fk_live_valid !== true) return null;
  const pose = {};
  for (const field of IK_COMPARE_FIELDS) {
    const key = field.key;
    // Backend (ws_handlers_imu) usa fk_live_*_mm per X/Y/Z, non fk_live_x/y/z.
    const rawKey =
      key === "x"
        ? "fk_live_x_mm"
        : key === "y"
          ? "fk_live_y_mm"
          : key === "z"
            ? "fk_live_z_mm"
            : `fk_live_${key}`;
    const val = _num(msg[rawKey]);
    if (val === null) return null;
    pose[key] = field.kind === "rot" ? _normAngleDeg(val) : val;
  }
  pose.wcMm = [
    _num(msg.fk_live_wc_x_mm),
    _num(msg.fk_live_wc_y_mm),
    _num(msg.fk_live_wc_z_mm),
  ];
  if (pose.wcMm.some((v) => v === null)) return null;
  pose.quat = {
    w: _num(msg.fk_live_quat_w),
    x: _num(msg.fk_live_quat_x),
    y: _num(msg.fk_live_quat_y),
    z: _num(msg.fk_live_quat_z),
  };
  if (Object.values(pose.quat).some((v) => v === null)) return null;
  return pose;
}

function _extractImuQuat(msg) {
  const w = _num(msg?.imu_q_w);
  const x = _num(msg?.imu_q_x);
  const y = _num(msg?.imu_q_y);
  const z = _num(msg?.imu_q_z);
  if ([w, x, y, z].some((v) => v === null)) return null;
  return { w, x, y, z };
}

function _statusClassForCompare() {
  if (!_ikCompare.activeTarget) return "idle";
  if (_ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING && _ikCompare.estimatorPrimed) return "ok";
  if (
    _ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE ||
    _ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_SETTLE ||
    _ikCompare.comparePhase === IK_COMPARE_PHASE.READY_TO_PRIME
  ) {
    return "computing";
  }
  if (!_ikCompare.lastImuValid) return "error";
  return "computing";
}

function buildCompareGrid() {
  const grid = document.getElementById("ik-compare-grid");
  if (!grid) return;
  grid.innerHTML = "";

  // Layout thesis-ready: due card affiancate (Target | Reale) + una card
  // full-width sotto (Errore). I tre id di card restano quelli già usati
  // dal resto del codice: "target", "real", "error".
  const topRow = document.createElement("div");
  topRow.className = "ik-compare-row-top";

  const buildCard = (cardKey) => {
    const meta = IK_COMPARE_CARD_META[cardKey];
    const card = document.createElement("div");
    card.className = `ik-compare-card ${meta.className}`;
    // L'errore ha, sotto i 6 numeri, una riga riassuntiva con pallino
    // tri-stato + norma posizione + norma orientazione. Viene popolata
    // dinamicamente in renderIkCompare.
    const footer = cardKey === "error"
      ? `<div class="ik-error-summary" id="cmp-error-summary">
           <span class="ik-error-dot" id="cmp-error-dot">●</span>
           <span class="ik-error-norm" id="cmp-error-norm-pos">Errore posizione: —</span>
           <span class="ik-error-norm" id="cmp-error-norm-rot">Errore orientazione: —</span>
         </div>`
      : "";
    card.innerHTML = `
      <div class="ik-compare-card-title">
        <h3>${meta.title}</h3>
        <span id="cmp-meta-${cardKey}">${meta.note}</span>
      </div>
      <div class="ik-num-grid" id="cmp-grid-${cardKey}"></div>
      ${footer}
    `;
    const inner = card.querySelector(`#cmp-grid-${cardKey}`);
    IK_COMPARE_FIELDS.forEach((field) => {
      const labelPrefix = cardKey === "error" ? "Δ" : "";
      const cell = document.createElement("div");
      cell.className = "ik-value-cell out-pose";
      cell.innerHTML = `
        <div class="cell-label">${labelPrefix}${field.label} <span class="short">${field.unit}</span></div>
        <div class="cell-value" id="cmp-${cardKey}-${field.key}">—</div>
      `;
      inner.appendChild(cell);
    });
    return card;
  };

  topRow.appendChild(buildCard("target"));
  topRow.appendChild(buildCard("real"));
  grid.appendChild(topRow);
  grid.appendChild(buildCard("error"));
}

function _setCompareMeta(cardKey, text) {
  const el = document.getElementById(`cmp-meta-${cardKey}`);
  if (el) el.textContent = text;
}

function _renderComparePose(cardKey, pose) {
  IK_COMPARE_FIELDS.forEach((field) => {
    const el = document.getElementById(`cmp-${cardKey}-${field.key}`);
    if (!el) return;
    el.textContent = pose ? _fmtPoseValue(field.key, pose[field.key]) : "—";
  });
}

/**
 * Popola le 6 celle della Sezione 3 con gli angoli ricostruiti dall'IK.
 * angles è null se l'IK è fallita → mostra "—".
 */
function _renderRecoveredJoints(angles) {
  const ids = ["ik-rec-base", "ik-rec-spalla", "ik-rec-gomito", "ik-rec-yaw", "ik-rec-pitch", "ik-rec-roll"];
  ids.forEach((id, i) => {
    const el = document.getElementById(id);
    if (!el) return;
    if (!Array.isArray(angles) || !Number.isFinite(Number(angles[i]))) {
      el.textContent = "—";
    } else {
      el.textContent = `${Number(angles[i]).toFixed(1)}°`;
    }
  });
}

/**
 * Calcola e mostra il verdict "round-trip FK→IK":
 *   max |input_joint - recovered_joint| su 6 giunti
 * Soglie: <1° ALLINEATO, <3° ENTRO TOLLERANZA, ≥3° SCOSTAMENTO.
 * Aggiorna: pallino, max, meta solver, verdict line.
 */
function _updateRoundTripVerdict(recovered, meta) {
  const input = _roundTripInputJoints;
  const maxEl = document.getElementById("ik-round-max");
  const metaEl = document.getElementById("ik-round-meta");
  const dotEl = document.getElementById("ik-round-dot");
  const verdictWrap = document.getElementById("ik-round-verdict");
  const verdictVal = document.getElementById("ik-round-verdict-value");
  const statusEl = document.getElementById("ik-round-status");

  if (!Array.isArray(recovered) || recovered.length !== 6 || !Array.isArray(input) || input.length !== 6) {
    if (maxEl) maxEl.textContent = "Max scostamento: —";
    if (metaEl && meta) metaEl.innerHTML = `Solver: ${meta.solver || "—"} · Iter: ${meta.iterations ?? "—"} · <span id="ik-elapsed-ms">${meta.elapsedMs ?? "-"}</span> ms`;
    return;
  }

  const diffs = recovered.map((v, i) => Math.abs(Number(v) - Number(input[i])));
  const maxDiff = Math.max(...diffs);
  const worstIdx = diffs.indexOf(maxDiff);
  const joints = ["B", "S", "G", "Y", "P", "R"];

  // Soglie round-trip: in condizioni normali max < 0.5°; sopra 3° c'è un
  // problema (target fuori manifold, jacobiano singolare, ecc.).
  let level;
  let label;
  if (maxDiff < 1.0)       { level = "green";  label = "ROUND-TRIP OK"; }
  else if (maxDiff < 3.0)  { level = "yellow"; label = "ENTRO TOLLERANZA"; }
  else                     { level = "red";    label = "SCOSTAMENTO SIGNIFICATIVO"; }

  if (maxEl) {
    maxEl.textContent = `Max scostamento: ${maxDiff.toFixed(2)}° (giunto ${joints[worstIdx]})`;
  }
  if (metaEl && meta) {
    metaEl.innerHTML = `Solver: ${meta.solver || "POE"} · Iter: ${meta.iterations ?? "—"} · <span id="ik-elapsed-ms">${meta.elapsedMs ?? "-"}</span> ms`;
  }
  if (dotEl) {
    dotEl.className = `ik-error-dot error-dot-${level}`;
  }
  if (verdictVal) verdictVal.textContent = label;
  if (verdictWrap) verdictWrap.className = `error-verdict verdict-${level}`;
  if (statusEl) {
    statusEl.textContent = level === "green"
      ? "IK ricostruisce gli stessi giunti della Sez 1 → premi «Invia posa» per muovere il robot."
      : level === "yellow"
        ? "IK entro tolleranza ma non identica — invio possibile."
        : "IK non recupera gli stessi giunti: target probabilmente singolare o fuori manifold.";
  }
}

function _clearRoundTripVerdict(errMsg) {
  const maxEl = document.getElementById("ik-round-max");
  const dotEl = document.getElementById("ik-round-dot");
  const verdictWrap = document.getElementById("ik-round-verdict");
  const verdictVal = document.getElementById("ik-round-verdict-value");
  const statusEl = document.getElementById("ik-round-status");
  if (maxEl) maxEl.textContent = "Max scostamento: — (IK non risolta)";
  if (dotEl) dotEl.className = "ik-error-dot error-dot-red";
  if (verdictVal) verdictVal.textContent = "IK NON RISOLTA";
  if (verdictWrap) verdictWrap.className = "error-verdict verdict-red";
  if (statusEl) statusEl.textContent = `IK fallita: ${errMsg || "target fuori workspace"}`;
}

/**
 * Aggiorna il pannello "Stato robot" (Sezione 2 della UI tesi-ready).
 * Pillole: Stato moto, IMU, Sample rate, Tracking.
 */
function _updateStatePanel() {
  // Motion state — derivato dalla macchina a stati del compare:
  //   IDLE: nessun target armato o robot in attesa SETPOSE_DONE
  //   MOVING: WAIT_MOVE_DONE / WAIT_SETTLE
  //   TRACKING: TRACKING + estimator primed
  let motionState = "IDLE";
  if (_ikCompare.activeTarget) {
    if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE ||
        _ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_SETTLE) {
      motionState = "MOVING";
    } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING &&
               _ikCompare.estimatorPrimed) {
      motionState = "TRACKING";
    } else {
      motionState = "WAITING";
    }
  }
  _setStatePill("motion", motionState);

  // IMU health — OK se valid+primed, WAIT se valid ma non ancora primed,
  // DEGRADED se invalid (debounce false).
  let imuState;
  if (!_ikCompare.lastImuValid) imuState = "DEGRADED";
  else if (!_ikCompare.estimatorPrimed && _ikCompare.activeTarget) imuState = "WAIT";
  else imuState = "OK";
  _setStatePill("imu", imuState);

  // Sample rate (Hz)
  const rateEl = document.getElementById("state-value-rate");
  if (rateEl) {
    rateEl.textContent = Number.isFinite(_ikCompare.lastImuRateHz)
      ? `${_ikCompare.lastImuRateHz.toFixed(1)} Hz`
      : "— Hz";
  }

  // Tracking indicator
  const isTracking = motionState === "TRACKING";
  const trackEl = document.getElementById("state-value-track");
  if (trackEl) {
    trackEl.innerHTML = isTracking
      ? '<span class="state-dot state-dot-on">●</span> ATTIVO'
      : '<span class="state-dot state-dot-off">●</span> inattivo';
  }
}

function _setStatePill(kind, value) {
  const el = document.getElementById(`state-value-${kind}`);
  if (!el) return;
  el.textContent = value;
  const pill = document.getElementById(`state-pill-${kind}`);
  if (!pill) return;
  // Reset classi livello e applica quella corrente.
  pill.classList.remove("state-level-ok", "state-level-wait", "state-level-bad", "state-level-idle", "state-level-moving");
  if (value === "OK" || value === "TRACKING") pill.classList.add("state-level-ok");
  else if (value === "WAIT" || value === "WAITING") pill.classList.add("state-level-wait");
  else if (value === "DEGRADED") pill.classList.add("state-level-bad");
  else if (value === "MOVING") pill.classList.add("state-level-moving");
  else pill.classList.add("state-level-idle");
}

/**
 * Aggiorna la "summary line" sotto il blocco Errore:
 *   "Stato: ALLINEATO / SCOSTAMENTO MODERATO / ERRORE ELEVATO / —"
 * Basata sulle norme ||Δpos|| e ||Δrot||, con le stesse soglie del card errore.
 */
function _updateVerdict(err) {
  const el = document.getElementById("ik-error-verdict-value");
  const wrap = document.getElementById("ik-error-verdict");
  if (!el || !wrap) return;

  if (!_ikCompare.activeTarget) {
    el.textContent = "—";
    wrap.className = "error-verdict verdict-idle";
    return;
  }
  const { posMm, rotDeg } = _errorNorms(err);
  if (posMm === null || rotDeg === null) {
    el.textContent = "in attesa stima IMU…";
    wrap.className = "error-verdict verdict-idle";
    return;
  }
  const posLevel = _errorLevel(posMm, ERROR_POS_GREEN_MM, ERROR_POS_YELLOW_MM);
  const rotLevel = _errorLevel(rotDeg, ERROR_ROT_GREEN_DEG, ERROR_ROT_YELLOW_DEG);
  const order = { green: 0, yellow: 1, red: 2 };
  const worst = (order[rotLevel] > order[posLevel]) ? rotLevel : posLevel;
  let label;
  if (worst === "green") label = "ALLINEATO";
  else if (worst === "yellow") label = "SCOSTAMENTO MODERATO";
  else label = "ERRORE ELEVATO";
  el.textContent = label;
  wrap.className = `error-verdict verdict-${worst}`;
}

/**
 * Rende la card "Errore di posa":
 *  - valori Δ per-campo (con segno, mm/°)
 *  - pallino tri-stato + norme ||Δpos||, ||Δrot||
 *  - ciascuna cella colorata secondo la soglia (verde/giallo/rosso)
 */
function _renderErrorCard(err) {
  IK_COMPARE_FIELDS.forEach((field) => {
    const el = document.getElementById(`cmp-error-${field.key}`);
    if (!el) return;
    const val = err ? _num(err[field.key]) : null;
    el.textContent = val === null ? "—" : _fmtPoseValue(field.key, val);
    // Colora la cella con la classe di livello (pos vs rot hanno soglie diverse).
    const level = (val === null)
      ? "idle"
      : (field.kind === "rot"
          ? _errorLevel(Math.abs(val), ERROR_ROT_GREEN_DEG, ERROR_ROT_YELLOW_DEG)
          : _errorLevel(Math.abs(val), ERROR_POS_GREEN_MM, ERROR_POS_YELLOW_MM));
    el.className = `cell-value error-${level}`;
  });

  const { posMm, rotDeg } = _errorNorms(err);
  const posEl = document.getElementById("cmp-error-norm-pos");
  const rotEl = document.getElementById("cmp-error-norm-rot");
  const dotEl = document.getElementById("cmp-error-dot");
  if (posEl) {
    posEl.textContent = posMm === null
      ? "Errore posizione: —"
      : `Errore posizione: ${posMm.toFixed(1)} mm`;
  }
  if (rotEl) {
    rotEl.textContent = rotDeg === null
      ? "Errore orientazione: —"
      : `Errore orientazione: ${rotDeg.toFixed(1)}°`;
  }
  if (dotEl) {
    // Il livello complessivo è il peggiore tra posizione e orientazione.
    const levelPos = posMm === null ? "idle" : _errorLevel(posMm, ERROR_POS_GREEN_MM, ERROR_POS_YELLOW_MM);
    const levelRot = rotDeg === null ? "idle" : _errorLevel(rotDeg, ERROR_ROT_GREEN_DEG, ERROR_ROT_YELLOW_DEG);
    const order = { idle: 0, green: 1, yellow: 2, red: 3 };
    const worst = (order[levelRot] || 0) > (order[levelPos] || 0) ? levelRot : levelPos;
    dotEl.className = `ik-error-dot error-dot-${worst}`;
  }
}

function _currentTargetPose() {
  return _ikCompare.activeTarget || collectTarget();
}

function _currentTargetLabel() {
  if (_ikCompare.activeTarget && _ikCompare.targetSentAtMs > 0) {
    return `Ultimo invio: ${new Date(_ikCompare.targetSentAtMs).toLocaleTimeString()}`;
  }
  return "Campi IK correnti (non ancora inviati)";
}

/**
 * Errore firmato "reale − teorico": ΔX = real.x − target.x, ecc.
 * Per gli angoli applica wrap [-180, +180] così il segno resta leggibile
 * anche attraverso il confine yaw ±180°.
 * Ritorna null se uno dei due pose manca; i singoli campi possono essere
 * null se l'origine non li espone ancora.
 */
function _computeRealMinusTarget(target, real) {
  if (!target || !real) return null;
  const out = {};
  IK_COMPARE_FIELDS.forEach((field) => {
    const t = _num(target[field.key]);
    const r = _num(real[field.key]);
    if (t === null || r === null) {
      out[field.key] = null;
      return;
    }
    out[field.key] = field.kind === "rot" ? _normAngleDeg(r - t) : (r - t);
  });
  return out;
}

function _errorNorms(err) {
  if (!err) return { posMm: null, rotDeg: null };
  const dx = _num(err.x);
  const dy = _num(err.y);
  const dz = _num(err.z);
  const dY = _num(err.yaw);
  const dP = _num(err.pitch);
  const dR = _num(err.roll);
  const posMm = [dx, dy, dz].every(Number.isFinite)
    ? Math.hypot(dx, dy, dz)
    : null;
  // Norma angolare: somma euclidea degli Euler ZYX signed (proxy leggibile;
  // non è una metrica geodetica ma basta per il confronto della card).
  const rotDeg = [dY, dP, dR].every(Number.isFinite)
    ? Math.hypot(dY, dP, dR)
    : null;
  return { posMm, rotDeg };
}

function _setCompareStatus(text) {
  const badge = document.getElementById("ik-compare-status");
  if (!badge) return;
  const cls = _statusClassForCompare();
  const dot = cls === "ok" ? "●" : cls === "error" ? "✕" : cls === "computing" ? "…" : "○";
  badge.className = `ik-status ${cls}`;
  badge.innerHTML = `<span>${dot}</span><span>${text}</span>`;
}

function renderIkCompare() {
  const target = _currentTargetPose();
  const real = _ikCompare.imuEstimatePose;
  const err = _computeRealMinusTarget(_ikCompare.activeTarget, real);

  _renderComparePose("target", target);
  _renderComparePose("real", real);
  _renderErrorCard(err);
  _updateStatePanel();
  _updateVerdict(err);

  _setCompareMeta("target", _currentTargetLabel());
  _setCompareMeta(
    "real",
    !_ikCompare.estimatorPrimed
      ? "In attesa di prime IMU (dopo SETPOSE + stabilizzazione)"
      : _ikCompare.imuAccelRawAvailable
        ? "Stima con IMU (integrazione inerziale)"
        : "Orientazione IMU + ancoraggio FK (accel raw non esposto)",
  );
  _setCompareMeta("error", _ikCompare.activeTarget
    ? "Δ = reale − teorica (verde <5 mm / <2° · giallo <20 mm / <8° · rosso oltre)"
    : "Diventa utile dopo il primo invio IK");

  if (!_ikCompare.activeTarget) {
    _setCompareStatus("Invia una posa IK per armare il confronto realtime");
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE) {
    _setCompareStatus("In attesa completamento movimento (SETPOSE_DONE)…");
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_SETTLE) {
    _setCompareStatus("Movimento completato, attesa stabilizzazione…");
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.READY_TO_PRIME) {
    if (!_ikCompare.lastImuValid) {
      _setCompareStatus("Pronto al prime: in attesa IMU valida…");
    } else {
      _setCompareStatus("Pronto al prime della stima (primo campione utile)…");
    }
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING && _ikCompare.estimatorPrimed) {
    _setCompareStatus("Tracking IMU attivo: confronto realtime in corso");
  } else if (!_ikCompare.lastImuValid) {
    _setCompareStatus("Target armato ma IMU non valida: tracking in attesa");
  } else {
    _setCompareStatus("Target armato: fase compare in transizione…");
  }

  const meta = document.getElementById("ik-compare-meta");
  if (meta) {
    const parts = [
      `Target attivo: ${_ikCompare.activeTarget ? "sì" : "no"}`,
      `Fase: ${_ikCompare.activeTarget ? _ikCompare.comparePhase : IK_COMPARE_PHASE.IDLE}`,
      `FK live: ${_ikCompare.fkLivePose ? "ok" : "—"}`,
      `IMU: ${_ikCompare.lastImuValid ? "valida" : "non valida"}`,
      `Sample: ${_ikCompare.lastImuSampleCounter ?? "-"}`,
    ];
    if (_ikCompare.lastImuRateHz !== null && Number.isFinite(_ikCompare.lastImuRateHz)) {
      parts.push(`Rate: ${_ikCompare.lastImuRateHz.toFixed(1)} Hz`);
    }
    meta.textContent = parts.join(" · ");
  }
}

function _clearIkCompareSettleTimer() {
  if (_ikCompareSettleTimerId != null) {
    clearTimeout(_ikCompareSettleTimerId);
    _ikCompareSettleTimerId = null;
  }
}

/** Dopo SETPOSE_DONE (o fallback timeout movimento): finestra IK_COMPARE_SETTLE_MS poi READY_TO_PRIME. */
function _scheduleIkCompareSettleEnd() {
  _clearIkCompareSettleTimer();
  _ikCompare.settleStartMs = performance.now();
  _ikCompareSettleTimerId = setTimeout(() => {
    _ikCompareSettleTimerId = null;
    if (!_ikCompare.activeTarget || _ikCompare.comparePhase !== IK_COMPARE_PHASE.WAIT_SETTLE) return;
    _ikCompare.comparePhase = IK_COMPARE_PHASE.READY_TO_PRIME;
    _ikCompare.settleStartMs = null;
    renderIkCompare();
  }, IK_COMPARE_SETTLE_MS);
}

function armIkCompareTarget(targetPose) {
  _clearIkCompareSettleTimer();
  _ikCompare = {
    activeTarget: { ...targetPose },
    targetSentAtMs: Date.now(),
    comparePhase: IK_COMPARE_PHASE.WAIT_MOVE_DONE,
    moveWaitStartMs: performance.now(),
    settleStartMs: null,
    // FK live è telemetry-driven: non invalidare la cache JS (evita buco UI); reset solo stato integratore.
    fkLivePose: _ikCompare.fkLivePose,
    imuEstimatePose: null,
    estimatorPrimed: false,
    estWcPosM: null,
    estVelMps: [0, 0, 0],
    lastTelemetryTsMs: null,
    lastImuSampleCounter: null,
    lastImuRateHz: _ikCompare.lastImuRateHz,
    lastImuValid: _ikCompare.lastImuValid,
  };
  renderIkCompare();
}

function _advanceIkComparePhases() {
  if (!_ikCompare.activeTarget) return;
  const now = performance.now();
  if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE) {
    if (now - _ikCompare.moveWaitStartMs >= IK_COMPARE_MOVE_TIMEOUT_MS) {
      addLog(
        `⚠ IK compare: nessun SETPOSE_DONE entro ${IK_COMPARE_MOVE_TIMEOUT_MS} ms — fallback a stabilizzazione (${IK_COMPARE_SETTLE_MS} ms) prima del prime.`,
      );
      _ikCompare.comparePhase = IK_COMPARE_PHASE.WAIT_SETTLE;
      _scheduleIkCompareSettleEnd();
    }
  }
}

function initIkCompareSetposeDone() {
  registerSetposeDoneHandler((msg) => {
    if (!msg || msg.type !== "setpose_done") return;
    if (_ikCompare.comparePhase !== IK_COMPARE_PHASE.WAIT_MOVE_DONE || !_ikCompare.activeTarget) return;
    _ikCompare.comparePhase = IK_COMPARE_PHASE.WAIT_SETTLE;
    _scheduleIkCompareSettleEnd();
    addLog("IK compare: SETPOSE_DONE ricevuto — avvio finestra stabilizzazione prima del prime.");
    renderIkCompare();
  });
}

function _primeImuEstimatorFromTelemetry(msg) {
  if (!_ikCompare.activeTarget || _ikCompare.estimatorPrimed) return;
  const fkPose = _ikCompare.fkLivePose || _extractFkLivePose(msg);
  const quat = _extractImuQuat(msg);
  if (!fkPose || !quat) return;
  const estWcPosM = fkPose.wcMm.map((v) => Number(v) / 1000.0);
  const pose = _buildImuEstimatePose(estWcPosM, quat);
  if (!pose) return;
  _ikCompare.fkLivePose = fkPose;
  _ikCompare.estWcPosM = estWcPosM;
  _ikCompare.estVelMps = [0, 0, 0];
  _ikCompare.lastTelemetryTsMs = performance.now();
  _ikCompare.lastImuSampleCounter = _num(msg.imu_sample_counter);
  _ikCompare.imuEstimatePose = pose;
  _ikCompare.estimatorPrimed = true;
  renderIkCompare();
}

function _updateImuEstimator(msg) {
  _ikCompare.lastImuValid = msg?.imu_valid === true;
  _ikCompare.lastImuRateHz = Number.isFinite(Number(msg?.imu_rate_hz_est)) ? Number(msg.imu_rate_hz_est) : _ikCompare.lastImuRateHz;

  if (_ikCompare.activeTarget) {
    _advanceIkComparePhases();
  }

  if (!_ikCompare.activeTarget) {
    renderIkCompare();
    return;
  }

  const phase = _ikCompare.comparePhase;
  if (phase === IK_COMPARE_PHASE.WAIT_MOVE_DONE || phase === IK_COMPARE_PHASE.WAIT_SETTLE) {
    renderIkCompare();
    return;
  }

  if (!_ikCompare.lastImuValid) {
    renderIkCompare();
    return;
  }

  if (phase === IK_COMPARE_PHASE.READY_TO_PRIME) {
    _primeImuEstimatorFromTelemetry(msg);
    if (_ikCompare.estimatorPrimed) {
      _ikCompare.comparePhase = IK_COMPARE_PHASE.TRACKING;
    }
  }

  const canIntegrate =
    _ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING &&
    _ikCompare.estimatorPrimed &&
    _ikCompare.estWcPosM;

  if (!canIntegrate) {
    renderIkCompare();
    return;
  }

  const quat = _extractImuQuat(msg);
  const accBody = [_num(msg?.imu_accel_x), _num(msg?.imu_accel_y), _num(msg?.imu_accel_z)];
  const gyro = [_num(msg?.imu_gyro_x), _num(msg?.imu_gyro_y), _num(msg?.imu_gyro_z)];
  if (!quat || accBody.some((v) => v === null) || gyro.some((v) => v === null)) {
    renderIkCompare();
    return;
  }

  // BNO085 v1 pubblica solo Rotation Vector: accel/gyro sono esattamente 0
  // in telemetria (imu.c:672-677). In quel caso non si può integrare
  // un'accelerazione lineare: la sottrazione di IMU_GRAVITY_MPS2 lascerebbe
  // un residuo di -g su Z che farebbe divergere la stima (esattamente il
  // "Z esplode" osservato). Finché i campi raw restano zero, ancoriamo il
  // wrist-center stimato al wrist-center FK live corrente e usiamo l'IMU
  // solo per l'orientazione. Il test è sull'accel: gyro a zero da solo è
  // valido anche a robot fermo; accel raw identicamente zero NON lo è
  // (gravity exists), quindi è un marker affidabile di "accel non esposto".
  const accelRawAvailable = accBody.some((v) => Math.abs(Number(v)) > 1e-6);
  _ikCompare.imuAccelRawAvailable = accelRawAvailable;
  if (!accelRawAvailable) {
    const fkPose = _ikCompare.fkLivePose || _extractFkLivePose(msg);
    if (fkPose) {
      const wcM = fkPose.wcMm.map((v) => Number(v) / 1000.0);
      const pose = _buildImuEstimatePose(wcM, quat);
      if (pose) {
        _ikCompare.fkLivePose = fkPose;
        _ikCompare.estWcPosM = wcM;
        _ikCompare.estVelMps = [0, 0, 0];
        _ikCompare.lastTelemetryTsMs = performance.now();
        _ikCompare.lastImuSampleCounter = _num(msg?.imu_sample_counter);
        _ikCompare.imuEstimatePose = pose;
      }
    }
    renderIkCompare();
    return;
  }

  const sampleCounter = _num(msg?.imu_sample_counter);
  const nowMs = performance.now();
  if (
    sampleCounter !== null &&
    _ikCompare.lastImuSampleCounter !== null &&
    sampleCounter === _ikCompare.lastImuSampleCounter
  ) {
    renderIkCompare();
    return;
  }

  let dt = IMU_MAX_DT_S;
  if (_ikCompare.lastTelemetryTsMs !== null) {
    dt = Math.max(0.001, Math.min(IMU_MAX_DT_S, (nowMs - _ikCompare.lastTelemetryTsMs) / 1000.0));
  }
  _ikCompare.lastTelemetryTsMs = nowMs;
  _ikCompare.lastImuSampleCounter = sampleCounter;

  // Dormant path (BNO085 v1 keeps accBody identically zero, so the guard
  // above returns before we get here). When Phase 6 enables Calibrated
  // Accel, we want integrated velocity/position directly in base frame, so
  // rotate body accel via qBase = R_ee (same transform used for orientation
  // and tool-offset display above). Gravity cancellation then holds under
  // the operational assumption that base frame Z is gravity-up.
  const qBase = _imuQuatToBaseFrame(quat);
  const rot = _quatWxyzToMatrix3(qBase.w, qBase.x, qBase.y, qBase.z);
  const accBase = _matVecMul3(rot, accBody);
  let linAccWorld = [accBase[0], accBase[1], accBase[2] - IMU_GRAVITY_MPS2];
  const accMag = _vecNorm(linAccWorld);
  const gyroMag = _vecNorm(gyro);
  if (accMag < IMU_ACCEL_DEADBAND_MPS2) {
    linAccWorld = [0, 0, 0];
  }

  const damp = (linAccWorld[0] === 0 && linAccWorld[1] === 0 && linAccWorld[2] === 0 && gyroMag < IMU_STILL_GYRO_RAD_S)
    ? IMU_STILL_DAMP
    : IMU_ACTIVE_DAMP;
  let nextVel = _vecAdd(_vecScale(_ikCompare.estVelMps, damp), _vecScale(linAccWorld, dt));
  const speed = _vecNorm(nextVel);
  if (speed > IMU_MAX_SPEED_MPS) {
    nextVel = _vecScale(nextVel, IMU_MAX_SPEED_MPS / Math.max(speed, 1e-6));
  }
  if (gyroMag < IMU_STILL_GYRO_RAD_S * 0.7 && _vecNorm(linAccWorld) < IMU_ACCEL_DEADBAND_MPS2 * 0.5) {
    nextVel = [0, 0, 0];
  }
  _ikCompare.estVelMps = nextVel;
  const nextWcPosM = _vecAdd(_ikCompare.estWcPosM, _vecScale(nextVel, dt));
  const pose = _buildImuEstimatePose(nextWcPosM, quat);
  if (!pose) {
    renderIkCompare();
    return;
  }
  _ikCompare.estWcPosM = nextWcPosM;
  _ikCompare.imuEstimatePose = pose;
  renderIkCompare();
}

function initCompareTelemetry() {
  registerTelemetryHandler((msg) => {
    const fkPose = _extractFkLivePose(msg);
    if (fkPose) _ikCompare.fkLivePose = fkPose;
    // Cache l'ultimo frame valido con IMU+FK per il capture "Azzera IMU @ HOME":
    // così al click non dobbiamo fare request aggiuntive.
    if (msg && msg.imu_valid === true && msg.fk_live_valid === true &&
        Number.isFinite(Number(msg.imu_q_w)) && Number.isFinite(Number(msg.fk_live_quat_w))) {
      _lastTelemetryForZero = msg;
    }
    _updateImuEstimator(msg);
  });
}

// ---------------------------------------------------------------------------
// Aggiorna lo status badge
// ---------------------------------------------------------------------------
function setIKStatus(state, text) {
  const badge = document.getElementById("ik-status-badge");
  const dot   = document.getElementById("ik-status-dot");
  const label = document.getElementById("ik-status-text");
  if (badge) {
    badge.className = `ik-status ${state}`;
    if (dot)   dot.textContent   = state === "ok" ? "●" : state === "error" ? "✕" : state === "computing" ? "…" : "○";
    if (label) label.textContent = text;
  }
  // Specchio tesi-ready sotto il bottone primario di Sezione 1.
  const tesisLine = document.getElementById("tesis-target-status");
  if (tesisLine) {
    let short = text;
    if (state === "computing") short = "Calcolo IK in corso…";
    else if (state === "ok") short = _ikAutoSendPending
        ? "Soluzione trovata — invio in corso…"
        : "Soluzione trovata — pronto all'invio";
    else if (state === "error") short = `IK non risolta · ${text}`;
    tesisLine.textContent = short;
  }
}

// ---------------------------------------------------------------------------
// Aggiorna le celle risultato IK
// ---------------------------------------------------------------------------
function setIKResult(angles, reachable) {
  let display = angles;
  if (reachable && Array.isArray(angles) && angles.length === 6) {
    const { clamped, changed } = anglesClampedFromRaw(angles);
    display = clamped;
    if (changed) {
      addLog("IK: angoli mostrati riportati entro limiti globali (per giunto).");
    }
  }
  JOINT_NAMES.forEach((name, i) => {
    const cell = document.getElementById(`ik-res-${name.toLowerCase()}`);
    const val  = document.getElementById(`ik-res-val-${name.toLowerCase()}`);
    if (!cell || !val) return;
    if (reachable) {
      cell.className = "ik-value-cell ik-result-cell reachable";
      val.className  = "cell-value";
      val.textContent = `${display[i].toFixed(1)}°`;
    } else {
      cell.className = "ik-value-cell ik-result-cell unreachable";
      val.className  = "cell-value unreachable";
      val.textContent = "—";
    }
  });

  const btnSend = document.getElementById("btn-send-ik");
  if (btnSend) btnSend.disabled = !reachable;
}

// ---------------------------------------------------------------------------
// Sezioni collassabili (stesso pattern di settings.js)
// ---------------------------------------------------------------------------
function initCollapsibles() {
  document.querySelectorAll(".ik-section-header").forEach(header => {
    const targetId = header.dataset.target;
    const body     = document.getElementById(targetId);
    if (!body) return;
    header.addEventListener("click", () => {
      const isCollapsed = header.classList.toggle("collapsed");
      body.style.display = isCollapsed ? "none" : "";
    });
  });
}

// ---------------------------------------------------------------------------
// Pulsante Salva target cartesiano
// ---------------------------------------------------------------------------
function initSaveTarget() {
  const btn = document.getElementById("btn-save-target");
  const fb = document.getElementById("fb-target");
  if (!btn) return;

  btn.addEventListener("click", () => {
    const t = collectTarget();
    saveTarget(t);
    if (fb) {
      fb.textContent = "Target salvato ✓";
      fb.className = "save-feedback ok visible";
      setTimeout(() => { fb.className = "save-feedback"; }, 2500);
    }
    addLog(`Target IK salvato: (${t.x}, ${t.y}, ${t.z}) YPR=(${t.yaw}, ${t.pitch}, ${t.roll})`);
  });
}

// ---------------------------------------------------------------------------
// Payload base64 per IK_SOLVE: solo solver POE + tolleranze (il modello S/M è sul controller)
// ---------------------------------------------------------------------------
function buildIkSolvePayloadB64() {
  const resetSolverCb = document.getElementById("ik-reset-solver");
  const resetSolver = resetSolverCb ? !!resetSolverCb.checked : true;
  const maxPosEl = document.getElementById("ik-max-pos-error");
  const maxOriEl = document.getElementById("ik-max-ori-error");
  const maxPos = Math.max(0.1, parseFloat(maxPosEl?.value || `${MAX_POS_ERROR_DEFAULT}`) || MAX_POS_ERROR_DEFAULT);
  const maxOri = Math.max(0.1, parseFloat(maxOriEl?.value || `${MAX_ORI_ERROR_DEFAULT}`) || MAX_ORI_ERROR_DEFAULT);
  try {
    const json = JSON.stringify({
      solver: "POE",
      fallback_numeric: false,
      reset_solver: resetSolver,
      max_pos_error_mm: maxPos,
      max_ori_error_deg: maxOri,
      preferred_angles_deg: [90, 90, 90, 90, 90, 90],
    });
    const bytes = new TextEncoder().encode(json);
    let bin = "";
    for (const b of bytes) bin += String.fromCharCode(b);
    return btoa(bin);
  } catch (_) {
    return "";
  }
}

// ---------------------------------------------------------------------------
// Pulsante Calcola IK — invia IK_SOLVE al backend Python
// ---------------------------------------------------------------------------
function initCalcIK() {
  const btn = document.getElementById("btn-calc-ik");
  if (!btn) return;

  // Abilita il bottone (il solver è ora attivo)
  btn.disabled = false;
  btn.title = "";

  // Rimuove eventuale nota "disponibile con solver attivo"
  const note = btn.nextElementSibling;
  if (note && note.tagName === "SPAN") note.style.display = "none";

  btn.addEventListener("click", () => {
    const x     = parseFloat(document.getElementById("ik-x")?.value)     || 0;
    const y     = parseFloat(document.getElementById("ik-y")?.value)     || 0;
    const z     = parseFloat(document.getElementById("ik-z")?.value)     || 0;
    const roll  = parseFloat(document.getElementById("ik-roll")?.value)  || 0;
    const pitch = parseFloat(document.getElementById("ik-pitch")?.value) || 0;
    const yaw   = parseFloat(document.getElementById("ik-yaw")?.value)   || 0;

    setIKStatus("computing", "Calcolo in corso…");
    btn.disabled = true;

    const payloadB64 = buildIkSolvePayloadB64();
    const cmd   = `IK_SOLVE ${x} ${y} ${z} ${roll} ${pitch} ${yaw}${payloadB64 ? " " + payloadB64 : ""}`;
    sendCommand("uart", { cmd });
    addLog(`IK_SOLVE[POE] → (${x}, ${y}, ${z}) YPR=(${yaw}, ${pitch}, ${roll})`);
  });
}

// ---------------------------------------------------------------------------
// Handler risposta ik_result dal server
// ---------------------------------------------------------------------------
function initIkResultHandler() {
  registerIkResultHandler((msg) => {
    const btn = document.getElementById("btn-calc-ik");
    if (btn) btn.disabled = false;

    const solverUsedEl = document.getElementById("ik-solver-used");
    const elapsedEl = document.getElementById("ik-elapsed-ms");
    if (solverUsedEl) solverUsedEl.textContent = msg.solver_used || "POE";
    if (elapsedEl) elapsedEl.textContent = Number.isFinite(Number(msg.elapsed_ms)) ? Number(msg.elapsed_ms).toFixed(2) : "-";

    if (msg.reachable) {
      const su = msg.solver_used || "POE";
      setIKStatus("ok", `Soluzione trovata — ${su} · err_pos ${msg.error_pos} mm, err_ori ${msg.error_ori}°, iter ${msg.iterations}`);
      setIKResult(msg.angles_deg, true);
      const shown = clampAnglesBsgYpr(msg.angles_deg);
      _renderRecoveredJoints(msg.angles_deg);
      _updateRoundTripVerdict(msg.angles_deg, {
        solver: su,
        iterations: msg.iterations,
        elapsedMs: elapsedEl?.textContent || "-",
      });
      addLog(`IK ${su} OK — angoli: [${shown.map(v => v.toFixed(1)).join(", ")}] (${elapsedEl?.textContent || "-"} ms)`);
      // Auto-send se l'utente ha premuto "Invia posa IK" (unified button).
      if (_ikAutoSendPending) {
        _ikAutoSendPending = false;
        const ok = _sendIkSetposeFromAngles(msg.angles_deg, collectTarget());
        const tesisLine = document.getElementById("tesis-target-status");
        if (tesisLine) {
          tesisLine.textContent = ok
            ? "Target inviato al robot — attendo esecuzione + stima IMU…"
            : "Invio non riuscito (WS non connesso)";
        }
      }
    } else {
      const su = msg.solver_used || "POE";
      setIKStatus("error", msg.message || "Target fuori workspace");
      setIKResult([], false);
      _renderRecoveredJoints(null);
      _clearRoundTripVerdict(msg.message || "Target fuori workspace");
      addLog(`IK ${su} FAIL — ${msg.message || "fuori workspace"}`);
      if (_ikAutoSendPending) {
        _ikAutoSendPending = false;
        addLog("⚠ Auto-send annullato: target non raggiungibile");
      }
    }

    // Aggiorna il pill solver
    const solver = document.getElementById("diag-solver");
    if (solver) {
      solver.textContent = msg.reachable ? "OK" : "FAIL";
      solver.className   = `diag-value state-pill ${msg.reachable ? "state-on" : "state-warn"}`;
    }
  });
}

/**
 * Inviare SETPOSE a partire da una lista di angoli virtuali (°).
 * Estratto da initSendIK per riuso dal flusso "Invia posa IK" unificato,
 * dove non è disponibile la griglia angoli DOM al momento dell'invio.
 */
function _sendIkSetposeFromAngles(angles, targetPose) {
  if (!Array.isArray(angles) || angles.length !== 6) return false;
  const [b, s, g, y, p, r] = angles.map((v, i) => {
    const key = JOINT_LIMIT_KEYS[i];
    const base = Number.isFinite(Number(v)) ? Number(v) : 90;
    return Math.round(clampVirtualJoint(key, base));
  });
  const cmd = `SETPOSE ${b} ${s} ${g} ${y} ${p} ${r} ${_ikVel} ${_ikProfile}`;
  if (!sendCommand("uart", { cmd })) {
    addLog("✗ SETPOSE IK non inviato (WS non connesso)");
    return false;
  }
  addLog(`… SETPOSE IK inviato (pending conferma): ${cmd}`);
  armIkCompareTarget(targetPose);
  return true;
}

// ---------------------------------------------------------------------------
// Unified "Invia posa IK" — calcola + attendi ik_result + invia in un click.
// Il calcolo riusa initCalcIK (UI/stato coerenti); l'auto-send avviene in
// initIkResultHandler quando _ikAutoSendPending=true.
// ---------------------------------------------------------------------------
function initUnifiedSendButton() {
  const btn = document.getElementById("btn-send-ik-unified");
  if (!btn) return;
  btn.addEventListener("click", () => {
    _ikAutoSendPending = true;
    const calcBtn = document.getElementById("btn-calc-ik");
    if (calcBtn && !calcBtn.disabled) {
      calcBtn.click();
    } else {
      _ikAutoSendPending = false;
      addLog("✗ Calcolo IK in corso o non disponibile");
    }
  });
}

// ---------------------------------------------------------------------------
// Pulsante Invia SETPOSE
// ---------------------------------------------------------------------------
function initSendIK() {
  const btn = document.getElementById("btn-send-ik");
  if (!btn) return;
  btn.addEventListener("click", () => {
    const targetPose = collectTarget();
    const vals = JOINT_NAMES.map(name => {
      const el = document.getElementById(`ik-res-val-${name.toLowerCase()}`);
      return el ? parseFloat(el.textContent) : NaN;
    });
    const [b, s, g, y, p, r] = vals.map((v, i) => {
      const key = JOINT_LIMIT_KEYS[i];
      const base = Number.isFinite(v) ? v : 90;
      return Math.round(clampVirtualJoint(key, base));
    });

    // Usa vel e profilo ricevuti dal server via get_settings (aggiornati in _ikVel/_ikProfile).
    // ws_server si aspetta: SETPOSE B S G Y P R vel_deg_s PLANNER
    const cmd = `SETPOSE ${b} ${s} ${g} ${y} ${p} ${r} ${_ikVel} ${_ikProfile}`;
    if (!sendCommand("uart", { cmd })) {
      addLog("✗ SETPOSE IK non inviato (WS non connesso)");
      return;
    }
    addLog(`… SETPOSE IK inviato (pending conferma): ${cmd}`);
    addLog("… Stima IMU reset: attesa primo ancoraggio FK/IMU post-SETPOSE");
    armIkCompareTarget(targetPose);
  });
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Section 4 — Quick actions: "HOME + Azzera IMU" e "Invia posa al robot"
// ---------------------------------------------------------------------------

// Listener temporaneo per attendere il prossimo setpose_done.
let _cmpQuickResolver = null;
function _cmpInstallSetposeDoneWaiter() {
  registerSetposeDoneHandler((msg) => {
    if (typeof _cmpQuickResolver === "function") {
      const r = _cmpQuickResolver;
      _cmpQuickResolver = null;
      try { r(msg); } catch (_) {}
    }
  });
}
function _cmpWaitSetposeDone(timeoutMs = 8000) {
  return new Promise((resolve) => {
    let done = false;
    const t = setTimeout(() => {
      if (!done) { done = true; _cmpQuickResolver = null; resolve(null); }
    }, timeoutMs);
    _cmpQuickResolver = (msg) => {
      if (done) return;
      done = true;
      clearTimeout(t);
      resolve(msg);
    };
  });
}

function _cmpStatus(text, kind) {
  const el = document.getElementById("cmp-action-status");
  if (!el) return;
  el.textContent = text;
  el.style.color =
    kind === "ok"   ? "var(--success, #4caf50)" :
    kind === "err"  ? "var(--danger, #ff5161)"  :
    kind === "wait" ? "var(--primary, #3d9dff)" :
                      "var(--muted, #9db1cc)";
}

function initCmpQuickActions() {
  _cmpInstallSetposeDoneWaiter();

  // ---- ⌂ HOME + 🎯 Azzera IMU ----
  const btnHomeZero = document.getElementById("btn-cmp-home-zero");
  if (btnHomeZero) {
    btnHomeZero.addEventListener("click", async () => {
      btnHomeZero.disabled = true;
      _cmpStatus("⏳ Invio HOME al firmware…", "wait");
      const sent = sendCommand("uart", { cmd: "HOME" });
      if (!sent) {
        _cmpStatus("✗ HOME non inviato (WS non connesso)", "err");
        btnHomeZero.disabled = false;
        return;
      }
      // Attendi che il robot finisca il movimento
      const done = await _cmpWaitSetposeDone(9000);
      if (!done) {
        _cmpStatus("⚠ HOME timeout — IMU NON azzerata", "err");
        btnHomeZero.disabled = false;
        return;
      }
      _cmpStatus("Robot in HOME — assestamento meccanico…", "wait");
      // Settle 700 ms perché IMU si stabilizzi prima dell'azzeramento
      await new Promise(r => setTimeout(r, 700));
      // Triggera la stessa logica del pulsante "Azzera IMU @ HOME" già esistente
      const btnZero = document.getElementById("btn-zero-at-home");
      if (btnZero) {
        btnZero.click();
        _cmpStatus("✓ HOME completato + IMU azzerata", "ok");
        addLog("[Sec 4] HOME + Azzera IMU completato");
      } else {
        _cmpStatus("⚠ HOME ok ma pulsante zero IMU non trovato", "err");
      }
      btnHomeZero.disabled = false;
    });
  }

  // ---- ▶ Invia posa al robot (target Sec 4 → IK + SETPOSE) ----
  // Replica esatta di initUnifiedSendButton: setta il flag _ikAutoSendPending
  // PRIMA del click su btn-calc-ik, così initIkResultHandler invia SETPOSE
  // automaticamente quando il solver IK risponde.
  const btnSend = document.getElementById("btn-cmp-send-pose");
  if (btnSend) {
    btnSend.addEventListener("click", () => {
      const target = collectTarget();
      const btnCalc = document.getElementById("btn-calc-ik");
      if (!btnCalc) {
        _cmpStatus("✗ btn-calc-ik non trovato in DOM", "err");
        return;
      }
      if (btnCalc.disabled) {
        _cmpStatus("⚠ Calcolo IK in corso, attendi", "err");
        return;
      }
      _cmpStatus(`⏳ IK + SETPOSE su (${target.x.toFixed(1)}, ${target.y.toFixed(1)}, ${target.z.toFixed(1)})…`, "wait");
      _ikAutoSendPending = true;          // ← chiave: gate dell'auto-send dopo ik_result
      btnCalc.click();                    // calcola IK; al ricevimento risultato scatta SETPOSE
      addLog(`[Sec 4] Invio posa: X=${target.x} Y=${target.y} Z=${target.z} R=${target.roll} P=${target.pitch} Y=${target.yaw}`);
      // Auto-clear status dopo 4s (i feedback dettagliati arrivano sul confronto IMU)
      setTimeout(() => _cmpStatus("Pronto.", "info"), 4000);
    });
  }
}

// ---------------------------------------------------------------------------
// Pulsante HOME (pose HOME via comando UART)
// ---------------------------------------------------------------------------
function initHomeIK() {
  const btn = document.getElementById("btn-home-ik");
  if (!btn) return;
  btn.addEventListener("click", () => {
    // Richiama la pose HOME gestita dal controller (usa offsets/settings correnti).
    if (!sendCommand("uart", { cmd: "HOME" })) {
      addLog("✗ IK HOME non inviato (WS non connesso)");
      return;
    }
    addLog("… IK HOME inviato (pending conferma)");
  });
}

// ---------------------------------------------------------------------------
// Forward kinematics (POE) — compute_fk_poe
// ---------------------------------------------------------------------------
function initCalcFk() {
  const btn = document.getElementById("btn-calc-fk");
  if (!btn) return;
  btn.addEventListener("click", () => {
    const gv = (id) => parseFloat(document.getElementById(id)?.value);
    const angles = JOINT_LIMIT_KEYS.map((k, i) => {
      const id = FK_INPUT_IDS[i];
      const raw = gv(id);
      return clampVirtualJoint(k, Number.isFinite(raw) ? raw : 90);
    });
    // Memorizza gli angoli giunto della Sezione 1 come riferimento del
    // round-trip FK→IK: verranno confrontati con gli angoli ricostruiti
    // dall'IK solver applicata alla posa FK qui sotto.
    _roundTripInputJoints = angles.slice();
    const st = document.getElementById("fk-status");
    if (st) st.textContent = "Calcolo in corso…";
    if (!sendCommand("compute_fk_poe", { angles_deg: angles })) {
      if (st) st.textContent = "WebSocket non connesso";
      addLog("FK: WS non pronto");
    }
  });
}

function initFkResultHandler() {
  registerFkPoeResultHandler((msg) => {
    const st = document.getElementById("fk-status");
    const setOut = (id, v) => {
      const el = document.getElementById(id);
      if (el) el.textContent = v;
    };
    if (!msg.ok) {
      if (st) st.textContent = msg.error || "FK fallita";
      setOut("fk-out-x", "—");
      setOut("fk-out-y", "—");
      setOut("fk-out-z", "—");
      setOut("fk-out-roll", "—");
      setOut("fk-out-pitch", "—");
      setOut("fk-out-yaw", "—");
      const qel = document.getElementById("fk-out-quat");
      if (qel) qel.textContent = "Quaternione (xyzw): —";
      return;
    }
    if (st) st.textContent = "OK";
    const x = Number(msg.x_mm);
    const y = Number(msg.y_mm);
    const z = Number(msg.z_mm);
    const roll = Number(msg.roll_deg);
    const pitch = Number(msg.pitch_deg);
    const yaw = Number(msg.yaw_deg);
    setOut("fk-out-x", String(msg.x_mm));
    setOut("fk-out-y", String(msg.y_mm));
    setOut("fk-out-z", String(msg.z_mm));
    setOut("fk-out-roll", String(msg.roll_deg));
    setOut("fk-out-pitch", String(msg.pitch_deg));
    setOut("fk-out-yaw", String(msg.yaw_deg));
    if ([x, y, z, roll, pitch, yaw].every((v) => Number.isFinite(v))) {
      applyTarget({ x, y, z, roll, pitch, yaw });
      addLog("FK: posa copiata nei campi target IK");
      // Mirror sotto il card FK per dire "questa è la posa che entra in IK"
      const setMirror = (id, v) => {
        const el = document.getElementById(id);
        if (el) el.textContent = Number.isFinite(v) ? v.toFixed(1) : "—";
      };
      setMirror("ik-target-mirror-x", x);
      setMirror("ik-target-mirror-y", y);
      setMirror("ik-target-mirror-z", z);
    }
    const q = msg.quat_xyzw;
    const qel = document.getElementById("fk-out-quat");
    if (qel && Array.isArray(q) && q.length === 4) {
      qel.textContent = `Quaternione (xyzw): ${q.map((n) => Number(n).toFixed(4)).join(", ")}`;
    }
    // Segnala che ora si può passare a Sezione 3 (Calcola IK).
    const roundStatus = document.getElementById("ik-round-status");
    if (roundStatus) roundStatus.textContent = "FK completata → premi «Calcola IK» per ricostruire i giunti.";
  });
}

// ---------------------------------------------------------------------------
// Handler settings dal server — aggiorna vel e profilo per SETPOSE
// ---------------------------------------------------------------------------
function onSettings(data) {
  if (data.vel_max !== undefined) _ikVel     = Number(data.vel_max);
  if (data.profile !== undefined) _ikProfile = String(data.profile);
}

function initPoeParamsHandler() {
  registerPoeParamsHandler((msg) => {
    if (msg.type === "poe_params" && Array.isArray(msg.S) && msg.S.length === 6 && Array.isArray(msg.M) && msg.M.length === 4) {
      const cfg = resolvePoeCfgFromServerMessageIk(msg);
      _poeFromServer = cfg;
      savePoeLocalMirror(cfg);
      addLog("POE sincronizzato dal Raspberry (FK/IK)");
    } else if (msg.type === "poe_params_saved") {
      if (msg.ok) sendCommand("get_poe_params", {});
    }
  });
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
function init() {
  console.log("[IK-FRAME-FIX v1] ik.js loaded — IMU compare expressed in base frame via R_ee = R_world_bias^-1 · R_imu · R_mount^-1");
  const savedTarget = loadTarget();
  if (savedTarget) {
    applyTarget(savedTarget);
  }
  buildResultGrid();
  buildCompareGrid();
  loadJointLimitsFromBackend();
  // Fire-and-forget: identity until loaded; subsequent telemetry frames
  // automatically pick up the new calib once the fetch resolves.
  void _loadImuFrameCalib();
  initCollapsibles();
  initSaveTarget();
  initCalcIK();
  initIkResultHandler();
  initCalcFk();
  initFkResultHandler();
  initSendIK();
  initUnifiedSendButton();
  initHomeIK();
  initZeroAtHomeButtons();
  initPoeParamsHandler();
  initCompareTelemetry();
  initIkCompareSetposeDone();
  initCmpQuickActions();
  registerSettingsHandler(onSettings);
  registerUartResponseHandler((msg) => {
    if (msg?.type === "uart_response" && msg?.warning) {
      addLog(`⚠ ${msg.warning}`);
      alert(msg.warning);
    }
  });
  connectJ5Dashboard();
  sendCommand("get_settings", {});
  sendCommand("get_poe_params", {});
  renderIkCompare();
  addLog("Pagina FK / IK caricata");
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
