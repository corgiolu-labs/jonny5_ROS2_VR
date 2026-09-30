/**
 * twin.js — JONNY5 Digital Twin (3D)
 *
 * Clona il visualizzatore 3D della pagina IK Live (stessa geometria a
 * cilindri + sfere emissive, stesso Three.js locale `window.THREE`), in una
 * pagina dedicata pilotata dalla telemetria giunti.
 *
 * Posa fedele al robot: applica la calibrazione reale per-giunto
 *   angolo_virtuale[i] = (servo_deg[i] - offset[i]) * dir[i] + 90
 * (offsets/dirs da `get_settings`, fallback ai valori di j5_settings.json),
 * poi updateRobotPose fa (deg-90). Il segno dei pivot è lo stesso del modello
 * POE e dell'URDF ROS 2: positivo attorno a +Z / +Y / +X.
 *
 * Lunghezze dei link dai parametri POE salvati sul Raspberry (`get_poe_params`):
 * altezza asse spalla, gomito, polso pitch e sporgenza utensile (M).
 *
 * Modalità comando (passo 1): un "fantasma" semitrasparente mostra la posa
 * obiettivo scelta con i cursori; "Esegui" invia un solo SETPOSE_T_HR (angoli
 * virtuali x10, il backend converte in fisici e applica i limiti giunto) con
 * una durata che non supera MAX_SPEED_DEG_S.
 *
 * Segui dal vivo (passo 2): finché il deadman è premuto (SPAZIO, o il pulsante
 * "Tieni premuto per seguire") il robot insegue il fantasma: un SETPOSE_T_HR
 * al massimo ogni FOLLOW_PERIOD_MS quando il fantasma si sposta. Il firmware
 * interrompe la traiettoria in corso e riparte dal punto raggiunto. Al rilascio
 * (o se la pagina perde il focus, la telemetria o lo stato IDLE) parte un
 * comando di tenuta sulla posa attuale.
 */
import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerSettingsHandler,
  registerPoeParamsHandler,
  registerSetposeDoneHandler,
  registerUartResponseHandler,
  loadRoutingConfig,
  sendCommand,
} from "../../../shared/js/j5_common.js";

const THREE = window.THREE;
if (!THREE) console.error("[TWIN] Three.js non caricato (window.THREE assente)");

// Geometria (mm). Le lunghezze vengono sostituite da quelle del POE appena arrivano.
const GEOM = {
  baseRadius: 50, baseHeight: 30,
  shoulderZ: 134.3, shoulderRadius: 28,
  upperArmLen: 60, upperArmRadius: 18,
  forearmLen: 186.925, forearmRadius: 14,
  wristRadius: 14, wristLen: 24,
  toolOffsetX: 57, toolRadius: 7,
};

// Calibrazione di fallback (da raspberry/config_runtime/robot/j5_settings.json)
const DEFAULT_OFFSETS = [100, 88, 93, 95, 90, 95];
const DEFAULT_DIRS = [1, -1, 1, 1, -1, 1];
// Limiti fisici di fallback (routing_config.json), ordine B S G Y P R.
const DEFAULT_LIMITS = [[45, 135], [30, 145], [30, 145], [45, 135], [35, 120], [45, 135]];
const LIMIT_KEYS = ["base", "spalla", "gomito", "yaw", "pitch", "roll"];
const JOINT_NAMES = ["Base", "Spalla", "Gomito", "Yaw", "Pitch", "Roll"];

// Modalità comando
const MAX_SPEED_DEG_S = 30;     // velocità massima del giunto che si muove di più
const MIN_DURATION_MS = 1500;
const MAX_DURATION_MS = 12000;
const PROFILE = "RTR5";
// Segui dal vivo
const FOLLOW_PERIOD_MS = 150;     // al massimo un comando ogni 150 ms
const FOLLOW_MIN_MS = 250;        // durata minima di ogni tratto
const FOLLOW_MIN_STEP_DEG = 0.5;  // non reinviare per spostamenti più piccoli
const FOLLOW_PROFILE = "RTR3";    // cubico: riparte più deciso dopo ogni interruzione
const HOLD_MS = 200;              // tenuta al rilascio del deadman

const S = {
  scene: null, camera: null, renderer: null,
  real: null,    // { root, pivots }
  ghost: null,   // { root, pivots }
  settings: null,
  limitsPhys: DEFAULT_LIMITS.map((r) => r.slice()),
  viz: { jointAngles: [90, 90, 90, 90, 90, 90], alpha: 0.22 },
  realVirtual: null,          // ultima posa reale (gradi virtuali, senza EMA)
  robotState: "–",
  cmd: { enabled: false, target: [90, 90, 90, 90, 90, 90], busyUntil: 0 },
  follow: { active: false, lastSent: null, lastSentAt: 0, sentAny: false },
};
let lastTelem = 0;

function makeMaterial(color, opts = {}) {
  if (opts.ghost) {
    return new THREE.MeshStandardMaterial({
      color: 0x5dffa8, metalness: 0.1, roughness: 0.6,
      emissive: 0x1d5a3a, emissiveIntensity: 0.6,
      transparent: true, opacity: 0.32, depthWrite: false,
    });
  }
  return new THREE.MeshStandardMaterial({
    color, metalness: 0.55, roughness: 0.4,
    emissive: opts.emissive || 0x000000,
    emissiveIntensity: opts.emissiveIntensity || 0,
  });
}

// --------------------------------------------------------------------------
// Costruzione braccio (catena pivot gerarchica, come IK Live)
// --------------------------------------------------------------------------
function buildRobot(ghost = false) {
  const root = new THREE.Group();
  const pivots = [];
  const mat = (color, opts = {}) => makeMaterial(color, { ...opts, ghost });

  if (!ghost) {
    const basement = new THREE.Mesh(
      new THREE.CylinderGeometry(GEOM.baseRadius, GEOM.baseRadius * 1.1, GEOM.baseHeight, 36),
      mat(0x162640));
    basement.rotation.x = Math.PI / 2;
    basement.position.z = GEOM.baseHeight / 2;
    root.add(basement);
  }

  const p0 = new THREE.Group(); root.add(p0); pivots.push(p0);

  const column = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.shoulderRadius * 0.85, GEOM.shoulderRadius, GEOM.shoulderZ, 28),
    mat(0x2a4675));
  column.rotation.x = Math.PI / 2;
  column.position.z = GEOM.shoulderZ / 2;
  p0.add(column);

  const p1 = new THREE.Group(); p1.position.set(0, 0, GEOM.shoulderZ); p0.add(p1); pivots.push(p1);
  p1.add(new THREE.Mesh(new THREE.SphereGeometry(GEOM.shoulderRadius, 28, 18),
    mat(0x12c2b2, { emissive: 0x0a3a35, emissiveIntensity: 0.3 })));

  const upperArm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.upperArmRadius, GEOM.upperArmRadius * 1.05, GEOM.upperArmLen, 22),
    mat(0x3d9dff));
  upperArm.rotation.x = Math.PI / 2;
  upperArm.position.z = GEOM.upperArmLen / 2;
  p1.add(upperArm);

  const p2 = new THREE.Group(); p2.position.set(0, 0, GEOM.upperArmLen); p1.add(p2); pivots.push(p2);
  p2.add(new THREE.Mesh(new THREE.SphereGeometry(GEOM.upperArmRadius * 1.45, 24, 16),
    mat(0x12c2b2, { emissive: 0x0a3a35, emissiveIntensity: 0.3 })));

  const forearm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.forearmRadius, GEOM.forearmRadius * 1.05, GEOM.forearmLen, 20),
    mat(0x3d9dff));
  forearm.rotation.x = Math.PI / 2;
  forearm.position.z = GEOM.forearmLen / 2;
  p2.add(forearm);

  // Polso: yaw, pitch e roll nello stesso punto (asse pitch), come nel POE.
  const p3 = new THREE.Group(); p3.position.set(0, 0, GEOM.forearmLen); p2.add(p3); pivots.push(p3);
  p3.add(new THREE.Mesh(new THREE.SphereGeometry(GEOM.wristRadius * 1.4, 22, 14),
    mat(0xb07fd9, { emissive: 0x3a1a4a, emissiveIntensity: 0.4 })));

  const p4 = new THREE.Group(); p3.add(p4); pivots.push(p4);
  const p5 = new THREE.Group(); p4.add(p5); pivots.push(p5);

  const toolLink = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.toolRadius, GEOM.toolRadius * 1.1, GEOM.toolOffsetX, 16),
    mat(0xff9d3d));
  toolLink.rotation.z = Math.PI / 2;
  toolLink.position.x = GEOM.toolOffsetX / 2;
  p5.add(toolLink);

  const tcp = new THREE.Mesh(new THREE.SphereGeometry(GEOM.toolRadius * 1.6, 20, 14),
    mat(0xffb84d, { emissive: 0x4a3300, emissiveIntensity: 0.5 }));
  tcp.position.x = GEOM.toolOffsetX;
  p5.add(tcp);

  if (!ghost) {
    // --- Testa stereo: 2 camere IMX708 sull'end-effector (guardano avanti, +X) ---
    // Montate sul polso (p5) -> ruotano con il roll, come sul robot reale.
    const matCamBody = makeMaterial(0x0c0c12, { emissive: 0x00141f, emissiveIntensity: 0.25 });
    const matCamLens = makeMaterial(0x0a1822, { emissive: 0x00bcd4, emissiveIntensity: 0.75 });
    const bracket = new THREE.Mesh(new THREE.BoxGeometry(9, 48, 9), makeMaterial(0x1b2a3d));
    bracket.position.set(GEOM.toolOffsetX - 8, 0, 0);
    p5.add(bracket);
    for (const sy of [18, -18]) {
      const body = new THREE.Mesh(new THREE.BoxGeometry(20, 15, 15), matCamBody);
      body.position.set(GEOM.toolOffsetX - 8, sy, 0);
      p5.add(body);
      const lens = new THREE.Mesh(new THREE.CylinderGeometry(5.5, 5.5, 9, 20), matCamLens);
      lens.rotation.z = Math.PI / 2;                 // asse cilindro lungo X (obiettivo in avanti)
      lens.position.set(GEOM.toolOffsetX + 6, sy, 0);
      p5.add(lens);
    }
  }

  S.scene.add(root);
  return { root, pivots };
}

function disposeRobot(robot) {
  if (!robot) return;
  S.scene.remove(robot.root);
  robot.root.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) o.material.dispose();
  });
}

function rebuildRobots() {
  disposeRobot(S.real);
  disposeRobot(S.ghost);
  S.real = buildRobot(false);
  S.ghost = buildRobot(true);
  S.ghost.root.visible = S.cmd.enabled;
  updateRobotPose(S.real, S.viz.jointAngles);
  updateRobotPose(S.ghost, S.cmd.target);
}

// Lunghezze dai parametri POE (metri): assi di vite S (omega, v) e M.
// Per un asse omega=+y passante per (0, 0, h): v = -omega x q = (-h, 0, 0).
function applyPoeGeometry(msg) {
  if (!msg || !Array.isArray(msg.S) || msg.S.length !== 6 || !Array.isArray(msg.M)) return;
  const h = (i) => -Number(msg.S[i][3]) * 1000;        // altezza asse (mm)
  const hS = h(1), hE = h(2), hP = h(4);
  const tool = Number(msg.M[0][3]) * 1000;
  if (![hS, hE, hP, tool].every((v) => Number.isFinite(v) && v > 0) || !(hS < hE && hE < hP)) {
    console.warn("[TWIN] parametri POE non validi, geometria invariata", msg);
    return;
  }
  GEOM.shoulderZ = hS;
  GEOM.upperArmLen = hE - hS;
  GEOM.forearmLen = hP - hE;
  GEOM.toolOffsetX = tool;
  if (S.scene) rebuildRobots();
  const g = document.getElementById("tw-geom");
  if (g) g.textContent = `POE: spalla ${hS.toFixed(1)} · gomito ${hE.toFixed(1)} · polso ${hP.toFixed(1)} · utensile ${tool.toFixed(1)} mm`;
}

// --------------------------------------------------------------------------
// Scena
// --------------------------------------------------------------------------
function build3DScene() {
  const wrap = document.getElementById("twin-3d-wrap");
  const canvas = document.getElementById("twin-3d-canvas");
  const w = wrap.clientWidth, h = wrap.clientHeight;

  S.scene = new THREE.Scene();
  S.scene.fog = new THREE.Fog(0x02060c, 1000, 2200);
  S.camera = new THREE.PerspectiveCamera(35, w / h, 1, 5000);
  S.renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
  S.renderer.setPixelRatio(window.devicePixelRatio || 1);
  S.renderer.setSize(w, h, false);

  S.scene.add(new THREE.AmbientLight(0xffffff, 0.45));
  const key = new THREE.DirectionalLight(0xc8e2ff, 1.0); key.position.set(400, 700, 500); S.scene.add(key);
  const fill = new THREE.DirectionalLight(0x00e5ff, 0.5); fill.position.set(-500, 200, -200); S.scene.add(fill);
  const rim = new THREE.DirectionalLight(0x5dffa8, 0.3); rim.position.set(0, -400, 200); S.scene.add(rim);

  const grid = new THREE.GridHelper(900, 18, 0x00e5ff, 0x0a2a45);
  grid.material.opacity = 0.5; grid.material.transparent = true; S.scene.add(grid);
  const axes = new THREE.AxesHelper(140); axes.material.depthTest = false; S.scene.add(axes);

  rebuildRobots();
  attachMouseOrbit(canvas);
  window.addEventListener("resize", onResize);
  animate();
}

function attachMouseOrbit(canvas) {
  let dragging = false, lx = 0, ly = 0, az = Math.PI / 4, el = Math.PI / 3.5, dist = 1200;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const apply = () => {
    const x = dist * Math.cos(el) * Math.cos(az);
    const y = dist * Math.cos(el) * Math.sin(az);
    const z = dist * Math.sin(el);
    S.camera.position.set(x, y, z);
    S.camera.up.set(0, 0, 1);
    S.camera.lookAt(0, 0, 200);
  };
  canvas.addEventListener("mousedown", (e) => { dragging = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener("mouseup", () => { dragging = false; });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    const dx = e.clientX - lx, dy = e.clientY - ly;
    az -= dx * 0.008;
    el = clamp(el + dy * 0.008, 0.05, Math.PI / 2 - 0.05);
    lx = e.clientX; ly = e.clientY;
    apply();
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    dist = clamp(dist * (1 + e.deltaY * 0.001), 400, 2800);
    apply();
  }, { passive: false });
  // Touch (visore/tablet): 1 dito orbita, pinch zoom
  let pinch0 = 0;
  canvas.addEventListener("touchstart", (e) => {
    if (e.touches.length === 1) { dragging = true; lx = e.touches[0].clientX; ly = e.touches[0].clientY; }
    else if (e.touches.length === 2) { pinch0 = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY); }
  }, { passive: true });
  canvas.addEventListener("touchmove", (e) => {
    if (e.touches.length === 1 && dragging) {
      const dx = e.touches[0].clientX - lx, dy = e.touches[0].clientY - ly;
      az -= dx * 0.008; el = clamp(el + dy * 0.008, 0.05, Math.PI / 2 - 0.05);
      lx = e.touches[0].clientX; ly = e.touches[0].clientY; apply();
    } else if (e.touches.length === 2 && pinch0) {
      const d = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
      dist = clamp(dist * (pinch0 / (d || pinch0)), 400, 2800); pinch0 = d; apply();
    }
  }, { passive: true });
  canvas.addEventListener("touchend", () => { dragging = false; pinch0 = 0; });
  apply();
}

function onResize() {
  if (!S.renderer) return;
  const wrap = document.getElementById("twin-3d-wrap");
  const w = wrap.clientWidth, h = wrap.clientHeight;
  S.renderer.setSize(w, h, false);
  S.camera.aspect = w / h;
  S.camera.updateProjectionMatrix();
}

function updateRobotPose(robot, anglesVirtualDeg) {
  // Stessa convenzione del solver POE / IK Live: segno positivo per tutti i
  // pivot, le inversioni reali sono già nei `dirs` applicati a monte.
  if (!robot) return;
  const a = anglesVirtualDeg.map((d) => THREE.MathUtils.degToRad(d - 90));
  const p = robot.pivots; if (p.length < 6) return;
  p[0].rotation.set(0, 0, a[0]);   // BASE   Z
  p[1].rotation.set(0, a[1], 0);   // SPALLA Y
  p[2].rotation.set(0, a[2], 0);   // GOMITO Y
  p[3].rotation.set(0, 0, a[3]);   // YAW    Z
  p[4].rotation.set(0, a[4], 0);   // PITCH  Y
  p[5].rotation.set(a[5], 0, 0);   // ROLL   X
}

function animate() {
  requestAnimationFrame(animate);
  if (S.renderer) S.renderer.render(S.scene, S.camera);
}

// --------------------------------------------------------------------------
// Calibrazione: fisico <-> virtuale, limiti
// --------------------------------------------------------------------------
function calib() {
  const offs = (S.settings && Array.isArray(S.settings.offsets)) ? S.settings.offsets : DEFAULT_OFFSETS;
  const dirs = (S.settings && Array.isArray(S.settings.dirs)) ? S.settings.dirs : DEFAULT_DIRS;
  return { offs, dirs };
}

function physToVirtual(p, i) {
  const { offs, dirs } = calib();
  return (p - (offs[i] ?? 90)) * (dirs[i] ?? 1) + 90;
}

// Limiti del giunto i in gradi virtuali [min, max].
function virtualLimits(i) {
  const [pMin, pMax] = S.limitsPhys[i];
  const a = physToVirtual(pMin, i), b = physToVirtual(pMax, i);
  return [Math.min(a, b), Math.max(a, b)];
}

// --------------------------------------------------------------------------
// Telemetria -> posa (con calibrazione offsets/dirs + EMA), come IK Live
// --------------------------------------------------------------------------
function setTxt(id, v) {
  const e = document.getElementById(id);
  if (e) e.textContent = (v == null || Number.isNaN(Number(v))) ? "–" : Math.round(Number(v)) + "°";
}

registerTelemetryHandler((t) => {
  const angP = [t.servo_deg_B, t.servo_deg_S, t.servo_deg_G, t.servo_deg_Y, t.servo_deg_P, t.servo_deg_R];
  if (angP.some((v) => v == null)) return;
  const angV = angP.map((p, i) => physToVirtual(p, i));
  S.realVirtual = angV;

  const al = S.viz.alpha;
  for (let i = 0; i < 6; i++) S.viz.jointAngles[i] = al * angV[i] + (1 - al) * S.viz.jointAngles[i];
  updateRobotPose(S.real, S.viz.jointAngles);

  setTxt("tw-b", t.servo_deg_B); setTxt("tw-s", t.servo_deg_S); setTxt("tw-g", t.servo_deg_G);
  setTxt("tw-y", t.servo_deg_Y); setTxt("tw-p", t.servo_deg_P); setTxt("tw-r", t.servo_deg_R);
  S.robotState = t.robot_state || "–";
  const st = document.getElementById("tw-state"); if (st) st.textContent = S.robotState;
  lastTelem = performance.now();
});

registerSettingsHandler((msg) => {
  if (msg && msg.type === "settings") { S.settings = msg; refreshSliders(); }
});
registerPoeParamsHandler((msg) => applyPoeGeometry(msg));

// Indicatore connessione/live + stato del pulsante Esegui
setInterval(() => {
  const link = document.getElementById("tw-link");
  if (link) {
    const live = isLive();
    link.textContent = live ? "● live" : "○ in attesa telemetria…";
    link.style.color = live ? "#5dffa8" : "#9db1cc";
  }
  refreshCommandState();
}, 250);

function isLive() { return (performance.now() - lastTelem) < 2000; }

// --------------------------------------------------------------------------
// Modalità comando (passo 1: anteprima + esegui)
// --------------------------------------------------------------------------
function buildCommandPanel() {
  const grid = document.getElementById("tw-sliders");
  if (!grid) return;
  grid.innerHTML = "";
  JOINT_NAMES.forEach((name, i) => {
    const row = document.createElement("label");
    row.className = "tw-slider";
    row.innerHTML = `<span>${name}</span>
      <input type="range" id="tw-sl-${i}" step="0.5" />
      <b id="tw-sv-${i}">–</b>`;
    grid.appendChild(row);
    row.querySelector("input").addEventListener("input", (e) => {
      S.cmd.target[i] = Number(e.target.value);
      onTargetChanged();
    });
  });
  refreshSliders();

  document.getElementById("tw-cmd-toggle")?.addEventListener("change", (e) => {
    S.cmd.enabled = e.target.checked;
    document.getElementById("tw-cmd-body").hidden = !S.cmd.enabled;
    if (S.cmd.enabled) copyRealToTarget();
    if (S.ghost) S.ghost.root.visible = S.cmd.enabled;
    refreshCommandState();
  });
  document.getElementById("tw-copy")?.addEventListener("click", copyRealToTarget);
  document.getElementById("tw-home")?.addEventListener("click", () => {
    S.cmd.target = [90, 90, 90, 90, 90, 90];
    clampTarget(); refreshSliders(); onTargetChanged();
  });
  document.getElementById("tw-exec")?.addEventListener("click", executeTarget);
  // Deadman "Segui dal vivo": SPAZIO (tastiera) o pulsante tenuto premuto (touch).
  window.addEventListener("keydown", (e) => {
    if (e.code !== "Space" || !S.cmd.enabled) return;
    e.preventDefault();
    if (!e.repeat) setFollow(true);
  });
  window.addEventListener("keyup", (e) => {
    if (e.code !== "Space") return;
    if (S.cmd.enabled) e.preventDefault();   // niente "click" sul pulsante a fuoco
    setFollow(false);
  });
  window.addEventListener("blur", () => setFollow(false));
  document.addEventListener("visibilitychange", () => { if (document.hidden) setFollow(false); });
  const fb = document.getElementById("tw-follow");
  if (fb) {
    fb.addEventListener("pointerdown", (e) => { e.preventDefault(); fb.setPointerCapture?.(e.pointerId); setFollow(true); });
    for (const ev of ["pointerup", "pointercancel", "lostpointercapture"]) fb.addEventListener(ev, () => setFollow(false));
  }
  setInterval(followTick, 50);

  document.getElementById("tw-stop")?.addEventListener("click", () => {
    sendCommand("uart", { cmd: "STOP" });
    S.cmd.busyUntil = 0;
    setCmdStatus("STOP inviato: per ripartire usa SAFE → ENABLE nella dashboard.", true);
  });
}

function clampTarget() {
  for (let i = 0; i < 6; i++) {
    const [lo, hi] = virtualLimits(i);
    S.cmd.target[i] = Math.max(lo, Math.min(hi, S.cmd.target[i]));
  }
}

function refreshSliders() {
  for (let i = 0; i < 6; i++) {
    const el = document.getElementById(`tw-sl-${i}`);
    if (!el) continue;
    const [lo, hi] = virtualLimits(i);
    el.min = String(lo); el.max = String(hi);
    el.value = String(S.cmd.target[i]);
    const v = document.getElementById(`tw-sv-${i}`);
    if (v) v.textContent = `${(S.cmd.target[i] - 90).toFixed(1)}°`;
  }
}

function copyRealToTarget() {
  if (S.realVirtual) S.cmd.target = S.realVirtual.map((v) => Math.round(v * 2) / 2);
  clampTarget(); refreshSliders(); onTargetChanged();
}

function maxDelta() {
  if (!S.realVirtual) return 0;
  return Math.max(...S.cmd.target.map((v, i) => Math.abs(v - S.realVirtual[i])));
}

function durationMs() {
  const d = maxDelta() / MAX_SPEED_DEG_S * 1000;
  return Math.round(Math.max(MIN_DURATION_MS, Math.min(MAX_DURATION_MS, d)));
}

function onTargetChanged() {
  updateRobotPose(S.ghost, S.cmd.target);
  for (let i = 0; i < 6; i++) {
    const v = document.getElementById(`tw-sv-${i}`);
    if (v) v.textContent = `${(S.cmd.target[i] - 90).toFixed(1)}°`;
  }
  refreshCommandState();
}

function refreshCommandState() {
  const btn = document.getElementById("tw-exec");
  const info = document.getElementById("tw-plan");
  if (!btn) return;
  const busy = performance.now() < S.cmd.busyUntil;
  const delta = maxDelta();
  let why = "";
  if (!S.cmd.enabled) why = "modalità comando spenta";
  else if (!isLive()) why = "nessuna telemetria";
  else if (S.robotState !== "IDLE") why = `robot in ${S.robotState}: serve IDLE (ENABLE dalla dashboard)`;
  else if (S.follow.active) why = "segui dal vivo attivo";
  else if (busy) why = "movimento in corso…";
  else if (delta < 0.5) why = "il fantasma coincide con il robot";
  btn.disabled = Boolean(why);
  btn.title = why;
  if (info) {
    info.textContent = why
      ? why
      : `spostamento max ${delta.toFixed(1)}° · durata ${(durationMs() / 1000).toFixed(1)} s (≤ ${MAX_SPEED_DEG_S}°/s)`;
  }
}

function setCmdStatus(text, isError = false) {
  const el = document.getElementById("tw-cmd-status");
  if (!el) return;
  el.textContent = text;
  el.style.color = isError ? "#ff8a8a" : "#9db1cc";
}

function executeTarget() {
  refreshCommandState();
  if (document.getElementById("tw-exec")?.disabled) return;
  clampTarget();
  const t = durationMs();
  const x10 = S.cmd.target.map((v) => Math.round(v * 10));
  sendCommand("uart", { cmd: `SETPOSE_T_HR ${x10.join(" ")} ${t} ${PROFILE}` });
  S.cmd.busyUntil = performance.now() + t + 4000;
  setCmdStatus(`Inviato: durata ${(t / 1000).toFixed(1)} s…`);
  refreshCommandState();
}

registerUartResponseHandler((msg) => {
  if (!msg || !String(msg.cmd || "").toUpperCase().startsWith("SETPOSE_T_HR")) return;
  if (!msg.ok && S.follow.active) {
    stopFollow(false);
    setCmdStatus(`Segui interrotto, comando rifiutato: ${msg.response || "errore"}`, true);
    return;
  }
  if (!msg.ok) {
    S.cmd.busyUntil = 0;
    setCmdStatus(`Rifiutato: ${msg.response || "errore"}`, true);
  } else if (msg.warning) {
    setCmdStatus(`Accettato con limiti applicati: ${msg.warning}`, true);
  }
  refreshCommandState();
});

registerSetposeDoneHandler(() => {
  if (performance.now() < S.cmd.busyUntil) {
    S.cmd.busyUntil = 0;
    setCmdStatus("Arrivato (SETPOSE_DONE).");
    refreshCommandState();
  }
});

// --------------------------------------------------------------------------
// Segui dal vivo (passo 2)
// --------------------------------------------------------------------------
function followBlockReason() {
  if (!S.cmd.enabled) return "modalità comando spenta";
  if (!isLive()) return "nessuna telemetria";
  if (S.robotState !== "IDLE") return `robot in ${S.robotState}`;
  if (performance.now() < S.cmd.busyUntil) return "movimento «Esegui» in corso";
  return "";
}

function setGhostFollowing(on) {
  if (!S.ghost) return;
  S.ghost.root.traverse((o) => {
    if (o.material && o.material.transparent) {
      o.material.color.setHex(on ? 0xffa84d : 0x5dffa8);
      o.material.emissive.setHex(on ? 0x5a3a10 : 0x1d5a3a);
    }
  });
}

function setFollow(on) {
  if (on === S.follow.active) return;
  if (on) {
    const why = followBlockReason();
    if (why) { setCmdStatus(`Segui non disponibile: ${why}.`, true); return; }
    S.follow.active = true;
    S.follow.sentAny = false;
    S.follow.lastSent = null;
    S.follow.lastSentAt = 0;
    setGhostFollowing(true);
    document.getElementById("tw-follow")?.classList.add("on");
    setCmdStatus("Segui dal vivo: il robot insegue il fantasma finché tieni premuto.");
  } else {
    stopFollow(true);
  }
  refreshCommandState();
}

// sendHold: al rilascio, tenuta sulla posa comandata attuale (telemetria).
function stopFollow(sendHold) {
  if (!S.follow.active) return;
  S.follow.active = false;
  setGhostFollowing(false);
  document.getElementById("tw-follow")?.classList.remove("on");
  if (sendHold && S.follow.sentAny && S.realVirtual && isLive()) {
    const x10 = S.realVirtual.map((v) => Math.round(v * 10));
    sendCommand("uart", { cmd: `SETPOSE_T_HR ${x10.join(" ")} ${HOLD_MS} ${FOLLOW_PROFILE}` });
    setCmdStatus("Rilasciato: il robot si ferma sulla posa attuale.");
  }
  refreshCommandState();
}

function followTick() {
  if (!S.follow.active) return;
  const why = followBlockReason();
  if (why) {
    stopFollow(true);
    setCmdStatus(`Segui interrotto: ${why}.`, true);
    return;
  }
  const now = performance.now();
  if (now - S.follow.lastSentAt < FOLLOW_PERIOD_MS) return;
  clampTarget();
  const tgt = S.cmd.target;
  const moved = S.follow.lastSent
    ? Math.max(...tgt.map((v, i) => Math.abs(v - S.follow.lastSent[i])))
    : maxDelta();
  if (moved < FOLLOW_MIN_STEP_DEG) return;
  const t = Math.round(Math.max(FOLLOW_MIN_MS, Math.min(MAX_DURATION_MS, maxDelta() / MAX_SPEED_DEG_S * 1000)));
  const x10 = tgt.map((v) => Math.round(v * 10));
  sendCommand("uart", { cmd: `SETPOSE_T_HR ${x10.join(" ")} ${t} ${FOLLOW_PROFILE}` });
  S.follow.lastSent = tgt.slice();
  S.follow.lastSentAt = now;
  S.follow.sentAny = true;
}

// --------------------------------------------------------------------------
// Avvio
// --------------------------------------------------------------------------
if (THREE) build3DScene();
buildCommandPanel();
connectJ5Dashboard();
setTimeout(() => {
  sendCommand("get_settings", {});      // carica offsets/dirs
  sendCommand("get_poe_params", {});    // geometria dei link
}, 500);
loadRoutingConfig()
  .then((cfg) => {
    if (!cfg || !cfg.limits) return;
    LIMIT_KEYS.forEach((k, i) => {
      const r = cfg.limits[k];
      if (r && Number.isFinite(Number(r.min)) && Number.isFinite(Number(r.max))) {
        S.limitsPhys[i] = [Number(r.min), Number(r.max)];
      }
    });
    clampTarget(); refreshSliders();
  })
  .catch((e) => console.warn("[TWIN] routing-config non disponibile, limiti di fallback", e));
