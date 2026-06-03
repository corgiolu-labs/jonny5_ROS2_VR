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
 * poi updateRobotPose fa (deg-90). I `dirs` [1,-1,-1,1,-1,1] correggono i
 * giunti spalla/gomito/pitch che altrimenti girerebbero al contrario.
 */
import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerSettingsHandler,
  sendCommand,
} from "../../../shared/js/j5_common.js";

const THREE = window.THREE;
if (!THREE) console.error("[TWIN] Three.js non caricato (window.THREE assente)");

// Geometria (mm) — identica a IK Live
const GEOM = {
  baseRadius: 50, baseHeight: 30,
  shoulderZ: 94, shoulderRadius: 28,
  upperArmLen: 60, upperArmRadius: 18,
  forearmLen: 157, forearmRadius: 14,
  wristRadius: 14, wristLen: 24,
  toolOffsetX: 60, toolRadius: 7,
};

// Calibrazione di fallback (da raspberry/config_runtime/robot/j5_settings.json)
const DEFAULT_OFFSETS = [100, 88, 93, 95, 90, 95];
const DEFAULT_DIRS = [1, -1, -1, 1, -1, 1];

const S = {
  scene: null, camera: null, renderer: null, pivots: [],
  settings: null,
  viz: { jointAngles: [90, 90, 90, 90, 90, 90], alpha: 0.22 },
};
let lastTelem = 0;

function makeMaterial(color, opts = {}) {
  return new THREE.MeshStandardMaterial({
    color, metalness: 0.55, roughness: 0.4,
    emissive: opts.emissive || 0x000000,
    emissiveIntensity: opts.emissiveIntensity || 0,
  });
}

// --------------------------------------------------------------------------
// Costruzione braccio (catena pivot gerarchica, come IK Live)
// --------------------------------------------------------------------------
function buildRobot() {
  const pivots = [];

  const basement = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.baseRadius, GEOM.baseRadius * 1.1, GEOM.baseHeight, 36),
    makeMaterial(0x162640));
  basement.rotation.x = Math.PI / 2;
  basement.position.z = GEOM.baseHeight / 2;
  S.scene.add(basement);

  const p0 = new THREE.Group(); S.scene.add(p0); pivots.push(p0);

  const column = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.shoulderRadius * 0.85, GEOM.shoulderRadius, GEOM.shoulderZ, 28),
    makeMaterial(0x2a4675));
  column.rotation.x = Math.PI / 2;
  column.position.z = GEOM.shoulderZ / 2;
  p0.add(column);

  const p1 = new THREE.Group(); p1.position.set(0, 0, GEOM.shoulderZ); p0.add(p1); pivots.push(p1);
  p1.add(new THREE.Mesh(new THREE.SphereGeometry(GEOM.shoulderRadius, 28, 18),
    makeMaterial(0x12c2b2, { emissive: 0x0a3a35, emissiveIntensity: 0.3 })));

  const upperArm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.upperArmRadius, GEOM.upperArmRadius * 1.05, GEOM.upperArmLen, 22),
    makeMaterial(0x3d9dff));
  upperArm.rotation.x = Math.PI / 2;
  upperArm.position.z = GEOM.upperArmLen / 2;
  p1.add(upperArm);

  const p2 = new THREE.Group(); p2.position.set(0, 0, GEOM.upperArmLen); p1.add(p2); pivots.push(p2);
  p2.add(new THREE.Mesh(new THREE.SphereGeometry(GEOM.upperArmRadius * 1.45, 24, 16),
    makeMaterial(0x12c2b2, { emissive: 0x0a3a35, emissiveIntensity: 0.3 })));

  const forearm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.forearmRadius, GEOM.forearmRadius * 1.05, GEOM.forearmLen, 20),
    makeMaterial(0x3d9dff));
  forearm.rotation.x = Math.PI / 2;
  forearm.position.z = GEOM.forearmLen / 2;
  p2.add(forearm);

  const p3 = new THREE.Group(); p3.position.set(0, 0, GEOM.forearmLen); p2.add(p3); pivots.push(p3);
  p3.add(new THREE.Mesh(new THREE.SphereGeometry(GEOM.wristRadius * 1.4, 22, 14),
    makeMaterial(0xb07fd9, { emissive: 0x3a1a4a, emissiveIntensity: 0.4 })));

  const p4 = new THREE.Group(); p3.add(p4); pivots.push(p4);
  const p5 = new THREE.Group(); p4.add(p5); pivots.push(p5);

  const toolLink = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.toolRadius, GEOM.toolRadius * 1.1, GEOM.toolOffsetX, 16),
    makeMaterial(0xff9d3d));
  toolLink.rotation.z = Math.PI / 2;
  toolLink.position.x = GEOM.toolOffsetX / 2;
  p5.add(toolLink);

  const tcp = new THREE.Mesh(new THREE.SphereGeometry(GEOM.toolRadius * 1.6, 20, 14),
    makeMaterial(0xffb84d, { emissive: 0x4a3300, emissiveIntensity: 0.5 }));
  tcp.position.x = GEOM.toolOffsetX;
  p5.add(tcp);

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

  S.pivots = pivots;
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

  buildRobot();
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

function updateRobotPose(anglesVirtualDeg) {
  // Stessa convenzione del solver POE / IK Live: segno positivo per tutti i
  // pivot, le inversioni reali sono già nei `dirs` applicati a monte.
  const a = anglesVirtualDeg.map((d) => THREE.MathUtils.degToRad(d - 90));
  const p = S.pivots; if (p.length < 6) return;
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
// Telemetria -> posa (con calibrazione offsets/dirs + EMA), come IK Live
// --------------------------------------------------------------------------
function setTxt(id, v) {
  const e = document.getElementById(id);
  if (e) e.textContent = (v == null || Number.isNaN(Number(v))) ? "–" : Math.round(Number(v)) + "°";
}

registerTelemetryHandler((t) => {
  const angP = [t.servo_deg_B, t.servo_deg_S, t.servo_deg_G, t.servo_deg_Y, t.servo_deg_P, t.servo_deg_R];
  if (angP.some((v) => v == null)) return;
  const offs = (S.settings && Array.isArray(S.settings.offsets)) ? S.settings.offsets : DEFAULT_OFFSETS;
  const dirs = (S.settings && Array.isArray(S.settings.dirs)) ? S.settings.dirs : DEFAULT_DIRS;
  const angV = angP.map((p, i) => (p - (offs[i] ?? 90)) * (dirs[i] ?? 1) + 90);

  const al = S.viz.alpha;
  for (let i = 0; i < 6; i++) S.viz.jointAngles[i] = al * angV[i] + (1 - al) * S.viz.jointAngles[i];
  updateRobotPose(S.viz.jointAngles);

  setTxt("tw-b", t.servo_deg_B); setTxt("tw-s", t.servo_deg_S); setTxt("tw-g", t.servo_deg_G);
  setTxt("tw-y", t.servo_deg_Y); setTxt("tw-p", t.servo_deg_P); setTxt("tw-r", t.servo_deg_R);
  const st = document.getElementById("tw-state"); if (st) st.textContent = t.robot_state || "–";
  lastTelem = performance.now();
});

registerSettingsHandler((msg) => { if (msg && msg.type === "settings") S.settings = msg; });

// Indicatore connessione/live
setInterval(() => {
  const link = document.getElementById("tw-link");
  if (!link) return;
  const live = (performance.now() - lastTelem) < 2000;
  link.textContent = live ? "● live" : "○ in attesa telemetria…";
  link.style.color = live ? "#5dffa8" : "#9db1cc";
}, 500);

// --------------------------------------------------------------------------
// Avvio
// --------------------------------------------------------------------------
if (THREE) build3DScene();
connectJ5Dashboard();
setTimeout(() => sendCommand("get_settings", {}), 500);   // carica offsets/dirs
