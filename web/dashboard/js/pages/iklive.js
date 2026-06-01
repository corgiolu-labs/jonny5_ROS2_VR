/**
 * iklive.js — Pagina IK Live · validazione cinematica con confronto IMU
 *
 * Funzionalità:
 *   1. Vista 3D Three.js del braccio (FK live continuo)
 *   2. Marker target (cyan pulsante) + TCP raggiunto (verde) + frecce orientamento polso (FK rosso, IMU blu)
 *   3. Calcolo errore: pos (target↔FK) · ori (target↔FK) · Δ IMU vs FK (validazione esterna)
 *   4. 5 pattern demo (cubo, cerchio, linea, spline, random) con loop opzionale
 *   5. Statistiche aggregate (RMS, max) per il video tesi
 *   6. Grafico errore vs tempo (canvas 2D) con marker SETPOSE_DONE
 *
 * Three.js caricato via UMD globale (window.THREE).
 */

import {
  connectJ5Dashboard,
  sendCommand,
  addLog,
  registerTelemetryHandler,
  registerIkResultHandler,
  registerSetposeDoneHandler,
  registerSettingsHandler,
} from "../../../shared/js/j5_common.js";

const THREE = window.THREE;
if (!THREE) console.error("[IKL] Three.js non caricato");

// ============================================================
// Stato globale
// ============================================================
const STATE = {
  scene: null, camera: null, renderer: null,
  robot: { pivots: [], links: [] },
  tcpMarker: null, targetMarker: null,
  arrowFK: null, arrowIMU: null, arrowsVisible: true,
  trailPoints: [], trailLine: null, trailMax: 800,

  jointAngles: [90, 90, 90, 90, 90, 90],     // virtual deg (grezzi — per stats)
  fkLive: { x: null, y: null, z: null, roll: null, pitch: null, yaw: null,
            qw: 1, qx: 0, qy: 0, qz: 0 },     // grezzi — per stats / jitter
  imuQuat: { w: 1, x: 0, y: 0, z: 0 },
  imuRpy:  { roll: 0, pitch: 0, yaw: 0 },
  imuValid: false,

  // Visualizzazione 3D filtrata con EMA per smussare l'hunting servo PWM.
  // Le stats e il jitter usano comunque i valori grezzi STATE.fkLive / STATE.jointAngles.
  viz: {
    tcpX: null, tcpY: null, tcpZ: null,        // posizione TCP filtrata
    jointAngles: [90, 90, 90, 90, 90, 90],     // angoli giunti filtrati
    enabled: true,
    alpha: 0.20,                                // EMA factor (più basso = più smoothing)
  },

  // Target attuale (richiesto dall'IK)
  target: { x: null, y: null, z: null, roll: 0, pitch: 0, yaw: 0 },

  // Settings
  settings: null,
  profile: "RTR5",

  // Demo
  demoRunning: false, demoAbort: false,
  demoPattern: "cube",
  demoPoints: [],

  // Riferimento HOME per validazione IMU vs FK senza bias di montaggio.
  // Quando settato, l'errore "Δ IMU vs FK" è calcolato sui DELTA da HOME
  // di entrambi i quaternion, non sui valori assoluti.
  refHome: null,   // { qImu: {w,x,y,z}, qFk: {w,x,y,z}, ts }

  // Stats aggregati
  stats: {
    samples: [],   // { errPos, errImu, t }
    pointsDone: 0,
    pointsTotal: 0,
  },

  // Error chart buffer
  errBuffer: [],   // { t_ms, errPos, errImu }
  errEvents: [],   // { t_ms, label }
  errBufMax: 600,
  errWindowMs: 30000,

  _setposeDoneResolvers: [],
  _ikResultOnce: null,
};

// ============================================================
// Util
// ============================================================
const $ = (id) => document.getElementById(id);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const fmt = (v, d = 1) => (v == null || isNaN(v)) ? "–" : Number(v).toFixed(d);

function showFeedback(id, ok, msg) {
  const el = $(id);
  if (!el) return;
  el.textContent = msg;
  el.className = "save-feedback " + (ok ? "ok" : "err");
  setTimeout(() => { el.className = "save-feedback"; }, 3500);
}

// ============================================================
// 3D scene
// ============================================================
const GEOM = {
  baseRadius: 50, baseHeight: 30,
  shoulderZ: 94, shoulderRadius: 28,
  upperArmLen: 60, upperArmRadius: 18,
  forearmLen: 157, forearmRadius: 14,
  wristRadius: 14, wristLen: 24,
  toolOffsetX: 60, toolRadius: 7,
};

function makeMaterial(color, opts = {}) {
  return new THREE.MeshStandardMaterial({
    color, metalness: 0.55, roughness: 0.4,
    emissive: opts.emissive || 0x000000,
    emissiveIntensity: opts.emissiveIntensity || 0,
  });
}

function build3DScene() {
  const wrap = $("ikl-3d-wrap");
  const canvas = $("ikl-3d-canvas");
  const w = wrap.clientWidth, h = wrap.clientHeight;

  STATE.scene = new THREE.Scene();
  STATE.scene.fog = new THREE.Fog(0x02060c, 1000, 2200);

  STATE.camera = new THREE.PerspectiveCamera(35, w / h, 1, 5000);

  STATE.renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
  STATE.renderer.setPixelRatio(window.devicePixelRatio || 1);
  STATE.renderer.setSize(w, h, false);

  // Luci drammatiche
  STATE.scene.add(new THREE.AmbientLight(0xffffff, 0.45));
  const key = new THREE.DirectionalLight(0xc8e2ff, 1.0);
  key.position.set(400, 700, 500);
  STATE.scene.add(key);
  const fill = new THREE.DirectionalLight(0x00e5ff, 0.5);
  fill.position.set(-500, 200, -200);
  STATE.scene.add(fill);
  const rim = new THREE.DirectionalLight(0x5dffa8, 0.3);
  rim.position.set(0, -400, 200);
  STATE.scene.add(rim);

  // Pavimento + griglia
  const grid = new THREE.GridHelper(900, 18, 0x00e5ff, 0x0a2a45);
  grid.material.opacity = 0.5; grid.material.transparent = true;
  STATE.scene.add(grid);
  const axes = new THREE.AxesHelper(140);
  axes.material.depthTest = false;
  STATE.scene.add(axes);

  buildRobot();

  // Marker TCP (verde lucido)
  STATE.tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(11, 28, 18),
    makeMaterial(0x5dffa8, { emissive: 0x0a4a2a, emissiveIntensity: 0.7 }),
  );
  STATE.tcpMarker.visible = false;
  STATE.scene.add(STATE.tcpMarker);

  // Marker target (cyan pulsante)
  STATE.targetMarker = new THREE.Mesh(
    new THREE.SphereGeometry(13, 28, 18),
    makeMaterial(0x00e5ff, { emissive: 0x004a55, emissiveIntensity: 0.9 }),
  );
  STATE.targetMarker.visible = false;
  STATE.scene.add(STATE.targetMarker);

  // Frecce orientamento polso
  STATE.arrowFK = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 70, 0xff5577, 18, 8,
  );
  STATE.arrowFK.visible = false;
  STATE.scene.add(STATE.arrowFK);
  STATE.arrowIMU = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 70, 0x5db8ff, 18, 8,
  );
  STATE.arrowIMU.visible = false;
  STATE.scene.add(STATE.arrowIMU);

  // Trail
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(STATE.trailMax * 3), 3));
  const trailMat = new THREE.LineBasicMaterial({ color: 0x5dffa8, transparent: true, opacity: 0.65 });
  STATE.trailLine = new THREE.Line(trailGeom, trailMat);
  STATE.trailLine.frustumCulled = false;
  STATE.scene.add(STATE.trailLine);

  setView("iso");
  attachMouseOrbit(canvas);
  window.addEventListener("resize", onResize);
  animate();
}

function buildRobot() {
  const pivots = [], links = [];

  // Basement
  const basement = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.baseRadius, GEOM.baseRadius * 1.1, GEOM.baseHeight, 36),
    makeMaterial(0x162640),
  );
  basement.rotation.x = Math.PI / 2;
  basement.position.z = GEOM.baseHeight / 2;
  STATE.scene.add(basement);

  const p0 = new THREE.Group(); STATE.scene.add(p0); pivots.push(p0);

  const column = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.shoulderRadius * 0.85, GEOM.shoulderRadius, GEOM.shoulderZ, 28),
    makeMaterial(0x2a4675),
  );
  column.rotation.x = Math.PI / 2;
  column.position.z = GEOM.shoulderZ / 2;
  p0.add(column);

  const p1 = new THREE.Group(); p1.position.set(0, 0, GEOM.shoulderZ); p0.add(p1); pivots.push(p1);

  const sJoint = new THREE.Mesh(new THREE.SphereGeometry(GEOM.shoulderRadius, 28, 18), makeMaterial(0x12c2b2, { emissive:0x0a3a35, emissiveIntensity:0.3 }));
  p1.add(sJoint);

  const upperArm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.upperArmRadius, GEOM.upperArmRadius * 1.05, GEOM.upperArmLen, 22),
    makeMaterial(0x3d9dff),
  );
  upperArm.rotation.x = Math.PI / 2;
  upperArm.position.z = GEOM.upperArmLen / 2;
  p1.add(upperArm);
  links.push(upperArm);

  const p2 = new THREE.Group(); p2.position.set(0, 0, GEOM.upperArmLen); p1.add(p2); pivots.push(p2);

  const eJoint = new THREE.Mesh(new THREE.SphereGeometry(GEOM.upperArmRadius * 1.45, 24, 16), makeMaterial(0x12c2b2, { emissive:0x0a3a35, emissiveIntensity:0.3 }));
  p2.add(eJoint);

  const forearm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.forearmRadius, GEOM.forearmRadius * 1.05, GEOM.forearmLen, 20),
    makeMaterial(0x3d9dff),
  );
  forearm.rotation.x = Math.PI / 2;
  forearm.position.z = GEOM.forearmLen / 2;
  p2.add(forearm);
  links.push(forearm);

  const p3 = new THREE.Group(); p3.position.set(0, 0, GEOM.forearmLen); p2.add(p3); pivots.push(p3);

  const wristJoint = new THREE.Mesh(new THREE.SphereGeometry(GEOM.wristRadius * 1.4, 22, 14), makeMaterial(0xb07fd9, { emissive:0x3a1a4a, emissiveIntensity:0.4 }));
  p3.add(wristJoint);

  const p4 = new THREE.Group(); p3.add(p4); pivots.push(p4);
  const p5 = new THREE.Group(); p4.add(p5); pivots.push(p5);

  const toolLink = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.toolRadius, GEOM.toolRadius * 1.1, GEOM.toolOffsetX, 16),
    makeMaterial(0xff9d3d),
  );
  toolLink.rotation.z = Math.PI / 2;
  toolLink.position.x = GEOM.toolOffsetX / 2;
  p5.add(toolLink);

  const tcp = new THREE.Mesh(new THREE.SphereGeometry(GEOM.toolRadius * 1.6, 20, 14), makeMaterial(0xffb84d, {emissive:0x4a3300,emissiveIntensity:0.5}));
  tcp.position.x = GEOM.toolOffsetX;
  p5.add(tcp);

  STATE.robot.pivots = pivots; STATE.robot.links = links;
  STATE.robot.tcpLocal = tcp; STATE.robot.wristGroup = p3;
}

function updateRobotPose(anglesVirtualDeg) {
  // Convenzione assi: i pivot Three.js gerarchici usano la stessa
  // convenzione del solver POE → segno positivo per tutti i giunti.
  const a = anglesVirtualDeg.map(d => THREE.MathUtils.degToRad(d - 90));
  const p = STATE.robot.pivots;
  if (p.length < 6) return;
  p[0].rotation.set(0, 0, a[0]);     // BASE   on Z
  p[1].rotation.set(0, a[1], 0);     // SPALLA on Y
  p[2].rotation.set(0, a[2], 0);     // GOMITO on Y
  p[3].rotation.set(0, 0, a[3]);     // YAW    on Z
  p[4].rotation.set(0, a[4], 0);     // PITCH  on Y
  p[5].rotation.set(a[5], 0, 0);     // ROLL   on X
}

function setView(name) {
  if (!STATE.camera) return;
  switch (name) {
    case "iso":   STATE.camera.position.set( 750,  750,  500); break;
    case "top":   STATE.camera.position.set( 0.01, 0.01, 1300); break;
    case "front": STATE.camera.position.set( 0.01,-1300, 350); break;
    case "side":  STATE.camera.position.set( 1300, 0.01, 350); break;
  }
  STATE.camera.up.set(0, 0, 1);
  STATE.camera.lookAt(0, 0, 200);
}

function attachMouseOrbit(canvas) {
  let dragging = false, lx = 0, ly = 0, az = Math.PI/4, el = Math.PI/3.5, dist = 1200;
  const apply = () => {
    const x = dist * Math.cos(el) * Math.cos(az);
    const y = dist * Math.cos(el) * Math.sin(az);
    const z = dist * Math.sin(el);
    STATE.camera.position.set(x, y, z);
    STATE.camera.up.set(0, 0, 1);
    STATE.camera.lookAt(0, 0, 200);
  };
  canvas.addEventListener("mousedown", (e) => { dragging = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener("mouseup", () => { dragging = false; });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    const dx = e.clientX - lx, dy = e.clientY - ly;
    az -= dx * 0.008;
    el = clamp(el + dy * 0.008, 0.05, Math.PI/2 - 0.05);
    lx = e.clientX; ly = e.clientY;
    apply();
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    dist = clamp(dist * (1 + e.deltaY * 0.001), 400, 2800);
    apply();
  }, { passive: false });
  apply();
}

function onResize() {
  if (!STATE.renderer) return;
  const wrap = $("ikl-3d-wrap");
  const w = wrap.clientWidth, h = wrap.clientHeight;
  STATE.renderer.setSize(w, h, false);
  STATE.camera.aspect = w / h;
  STATE.camera.updateProjectionMatrix();
}

function pushTrailPoint(x, y, z) {
  STATE.trailPoints.push([x, y, z]);
  if (STATE.trailPoints.length > STATE.trailMax) STATE.trailPoints.shift();
  const arr = STATE.trailLine.geometry.attributes.position.array;
  for (let i = 0; i < STATE.trailPoints.length; i++) {
    arr[i*3]   = STATE.trailPoints[i][0];
    arr[i*3+1] = STATE.trailPoints[i][1];
    arr[i*3+2] = STATE.trailPoints[i][2];
  }
  STATE.trailLine.geometry.setDrawRange(0, STATE.trailPoints.length);
  STATE.trailLine.geometry.attributes.position.needsUpdate = true;
}

function animate() {
  requestAnimationFrame(animate);
  // Pulse target marker
  if (STATE.targetMarker && STATE.targetMarker.visible) {
    const s = 1 + 0.15 * Math.sin(performance.now() * 0.005);
    STATE.targetMarker.scale.set(s, s, s);
  }
  if (STATE.renderer) STATE.renderer.render(STATE.scene, STATE.camera);
}

// ============================================================
// Quaternion math utilities
// ============================================================
function rpyToQuat(rollDeg, pitchDeg, yawDeg) {
  // ZYX Tait-Bryan (yaw·pitch·roll). Coerente con scipy "ZYX" del solver POE.
  const cr = Math.cos(THREE.MathUtils.degToRad(rollDeg)  / 2);
  const sr = Math.sin(THREE.MathUtils.degToRad(rollDeg)  / 2);
  const cp = Math.cos(THREE.MathUtils.degToRad(pitchDeg) / 2);
  const sp = Math.sin(THREE.MathUtils.degToRad(pitchDeg) / 2);
  const cy = Math.cos(THREE.MathUtils.degToRad(yawDeg)   / 2);
  const sy = Math.sin(THREE.MathUtils.degToRad(yawDeg)   / 2);
  return {
    w: cr*cp*cy + sr*sp*sy,
    x: sr*cp*cy - cr*sp*sy,
    y: cr*sp*cy + sr*cp*sy,
    z: cr*cp*sy - sr*sp*cy,
  };
}

function quatNormalize(q) {
  const n = Math.hypot(q.w, q.x, q.y, q.z) || 1;
  return { w: q.w/n, x: q.x/n, y: q.y/n, z: q.z/n };
}

function quatConjugate(q) { return { w: q.w, x: -q.x, y: -q.y, z: -q.z }; }

function quatMultiply(a, b) {
  return {
    w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
  };
}

/** Angolo tra due quaternioni (gradi). */
function quatAngleDiffDeg(qa, qb) {
  const a = quatNormalize(qa), b = quatNormalize(qb);
  const dot = Math.abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
  return THREE.MathUtils.radToDeg(2 * Math.acos(clamp(dot, -1, 1)));
}

// ============================================================
// Telemetria
// ============================================================
function applyTelemetry(t) {
  // FK live
  const fx = t.fk_live_x_mm, fy = t.fk_live_y_mm, fz = t.fk_live_z_mm;
  if (t.fk_live_valid !== false && fx != null && fz != null) {
    STATE.fkLive.x = fx; STATE.fkLive.y = fy; STATE.fkLive.z = fz;
    STATE.fkLive.roll  = t.fk_live_roll;
    STATE.fkLive.pitch = t.fk_live_pitch;
    STATE.fkLive.yaw   = t.fk_live_yaw;
    STATE.fkLive.qw = t.fk_live_quat_w ?? 1;
    STATE.fkLive.qx = t.fk_live_quat_x ?? 0;
    STATE.fkLive.qy = t.fk_live_quat_y ?? 0;
    STATE.fkLive.qz = t.fk_live_quat_z ?? 0;

    // Filtro EMA per visualizzazione 3D (riduce hunting servo PWM nelle riprese).
    const a = STATE.viz.alpha;
    if (STATE.viz.enabled && STATE.viz.tcpX != null) {
      STATE.viz.tcpX = a * fx + (1 - a) * STATE.viz.tcpX;
      STATE.viz.tcpY = a * fy + (1 - a) * STATE.viz.tcpY;
      STATE.viz.tcpZ = a * fz + (1 - a) * STATE.viz.tcpZ;
    } else {
      STATE.viz.tcpX = fx; STATE.viz.tcpY = fy; STATE.viz.tcpZ = fz;
    }

    if (STATE.tcpMarker) {
      STATE.tcpMarker.position.set(STATE.viz.tcpX, STATE.viz.tcpY, STATE.viz.tcpZ);
      STATE.tcpMarker.visible = true;
    }
    // Trail (basato su valori filtrati per coerenza visiva con il marker)
    const last = STATE.trailPoints[STATE.trailPoints.length - 1];
    if (!last || Math.hypot(last[0]-STATE.viz.tcpX, last[1]-STATE.viz.tcpY, last[2]-STATE.viz.tcpZ) > 2) {
      pushTrailPoint(STATE.viz.tcpX, STATE.viz.tcpY, STATE.viz.tcpZ);
    }
    // HUD top-right
    $("hud-fk-x").textContent = fmt(fx, 0);
    $("hud-fk-y").textContent = fmt(fy, 0);
    $("hud-fk-z").textContent = fmt(fz, 0);
    // Coord row FK
    $("coord-fk-x").textContent = fmt(fx, 1);
    $("coord-fk-y").textContent = fmt(fy, 1);
    $("coord-fk-z").textContent = fmt(fz, 1);
    $("coord-fk-roll").textContent  = fmt(t.fk_live_roll, 1);
    $("coord-fk-pitch").textContent = fmt(t.fk_live_pitch, 1);
    $("coord-fk-yaw").textContent   = fmt(t.fk_live_yaw, 1);
  }

  // Angoli giunti (per animazione modello 3D)
  const angP = [t.servo_deg_B, t.servo_deg_S, t.servo_deg_G, t.servo_deg_Y, t.servo_deg_P, t.servo_deg_R];
  if (angP.every(v => v != null)) {
    let angV = angP;
    if (STATE.settings && Array.isArray(STATE.settings.offsets) && Array.isArray(STATE.settings.dirs)) {
      angV = angP.map((p, i) => {
        const off = STATE.settings.offsets[i] ?? 90;
        const dir = STATE.settings.dirs[i] ?? 1;
        return (p - off) * dir + 90;
      });
    }
    STATE.jointAngles = angV;     // grezzi — usati altrove se serve

    // Filtro EMA su angoli per il rendering del braccio 3D
    const aJ = STATE.viz.alpha;
    if (STATE.viz.enabled) {
      for (let i = 0; i < 6; i++) {
        STATE.viz.jointAngles[i] = aJ * angV[i] + (1 - aJ) * STATE.viz.jointAngles[i];
      }
      updateRobotPose(STATE.viz.jointAngles);
    } else {
      for (let i = 0; i < 6; i++) STATE.viz.jointAngles[i] = angV[i];
      updateRobotPose(angV);
    }
  }

  // IMU quaternion
  if (t.imu_valid && t.imu_q_w != null) {
    STATE.imuQuat = { w: t.imu_q_w, x: t.imu_q_x, y: t.imu_q_y, z: t.imu_q_z };
    STATE.imuValid = true;
    // RPY da IMU (informativo)
    const q = STATE.imuQuat;
    // ZYX from quat
    const sinr_cosp = 2 * (q.w*q.x + q.y*q.z);
    const cosr_cosp = 1 - 2 * (q.x*q.x + q.y*q.y);
    const roll  = THREE.MathUtils.radToDeg(Math.atan2(sinr_cosp, cosr_cosp));
    const sinp  = 2 * (q.w*q.y - q.z*q.x);
    const pitch = THREE.MathUtils.radToDeg(Math.asin(clamp(sinp, -1, 1)));
    const siny_cosp = 2 * (q.w*q.z + q.x*q.y);
    const cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z);
    const yaw   = THREE.MathUtils.radToDeg(Math.atan2(siny_cosp, cosy_cosp));
    STATE.imuRpy = { roll, pitch, yaw };
    $("coord-imu-roll").textContent  = fmt(roll, 1);
    $("coord-imu-pitch").textContent = fmt(pitch, 1);
    $("coord-imu-yaw").textContent   = fmt(yaw, 1);
    $("pill-imu").textContent = "OK";
    $("pill-imu").className = "diag-value state-pill state-on";
  } else {
    STATE.imuValid = false;
    $("pill-imu").textContent = "no data";
    $("pill-imu").className = "diag-value state-pill state-warn";
  }

  // Stato robot
  if (t.robot_state) {
    const el = $("pill-robot");
    el.textContent = t.robot_state;
    el.className = "diag-value state-pill " + (t.robot_state === "IDLE" ? "state-on" :
                                                t.robot_state === "STOPPED" ? "state-warn" : "state-unknown");
  }
  if (t.uart_active != null) {
    const el = $("pill-stm32");
    el.textContent = t.uart_active ? "OK" : "NO LINK";
    el.className = "diag-value state-pill " + (t.uart_active ? "state-on" : "state-off");
  }

  // Aggiornamento metriche live (errore vs target + IMU)
  updateLiveMetrics();
}

function updateLiveMetrics() {
  // 1. Errore posizione: target ↔ FK
  let errPos = null;
  const tg = STATE.target, fk = STATE.fkLive;
  if (tg.x != null && fk.x != null) {
    errPos = Math.hypot(tg.x - fk.x, tg.y - fk.y, tg.z - fk.z);
    setBigCell("big-err-pos", "big-err-pos-bar", errPos, "mm", [5, 20]);
    $("hud-tgt-delta").textContent = errPos.toFixed(1) + " mm";
  } else {
    $("big-err-pos").innerHTML = "–<span class='unit'>mm</span>";
    $("hud-tgt-delta").textContent = "–";
  }

  // 2. Errore orientamento target ↔ FK (in gradi)
  let errOri = null;
  if (tg.x != null) {
    const qTarget = rpyToQuat(tg.roll, tg.pitch, tg.yaw);
    const qFK = { w: fk.qw, x: fk.qx, y: fk.qy, z: fk.qz };
    if (Math.hypot(qFK.w, qFK.x, qFK.y, qFK.z) > 0.01) {
      errOri = quatAngleDiffDeg(qTarget, qFK);
      setBigCell("big-err-ori", "big-err-ori-bar", errOri, "°", [2, 8]);
    }
  }
  if (errOri == null) $("big-err-ori").innerHTML = "–<span class='unit'>°</span>";

  // 3. Δ IMU vs FK (validazione esterna del polso)
  // Se è stato registrato un riferimento HOME, calcoliamo i delta relativi
  // (= rotazione del polso DAL HOME secondo IMU vs DAL HOME secondo FK).
  // Questo elimina il bias di montaggio costante dell'IMU.
  let errImu = null;
  if (STATE.imuValid && Math.hypot(fk.qw, fk.qx, fk.qy, fk.qz) > 0.01) {
    const qImuNow = STATE.imuQuat;
    const qFkNow  = { w: fk.qw, x: fk.qx, y: fk.qy, z: fk.qz };
    if (STATE.refHome) {
      const dImu = quatMultiply(qImuNow, quatConjugate(STATE.refHome.qImu));
      const dFk  = quatMultiply(qFkNow,  quatConjugate(STATE.refHome.qFk));
      errImu = quatAngleDiffDeg(dImu, dFk);
    } else {
      errImu = quatAngleDiffDeg(qImuNow, qFkNow);
    }
    setBigCell("big-err-imu", "big-err-imu-bar", errImu, "°", [3, 10], true);
    $("hud-imu-delta").textContent = errImu.toFixed(1) + "°";
  } else {
    $("big-err-imu").innerHTML = "–<span class='unit'>°</span>";
    $("hud-imu-delta").textContent = "–";
  }

  // Aggiorna frecce orientamento polso
  updateWristArrows();

  // Buffer chart
  if (errPos != null || errImu != null) {
    const t_ms = performance.now();
    STATE.errBuffer.push({ t_ms, errPos: errPos != null ? errPos : 0, errImu: errImu != null ? errImu : 0 });
    while (STATE.errBuffer.length && t_ms - STATE.errBuffer[0].t_ms > STATE.errWindowMs) STATE.errBuffer.shift();
    while (STATE.errEvents.length && t_ms - STATE.errEvents[0].t_ms > STATE.errWindowMs) STATE.errEvents.shift();
  }
}

function setBigCell(valId, barId, value, unit, [warnTh, errTh], isBlue = false) {
  const el = $(valId);
  const bar = $(barId);
  const v = Number(value);
  let cls = "value " + (isBlue ? "blue " : "cyan ");
  let pct = 0, color = isBlue ? "var(--ikl-imu)" : "var(--ikl-tcp)";
  if (v < warnTh)      { cls += isBlue ? "blue" : "green"; pct = (v/warnTh)*40; color = isBlue ? "var(--ikl-imu)" : "var(--ikl-tcp)"; }
  else if (v < errTh)  { cls += "warn";    pct = 40 + ((v-warnTh)/(errTh-warnTh))*40; color = "var(--ikl-warn)"; }
  else                 { cls += "err";     pct = clamp(80 + (v-errTh)/errTh*20, 80, 100); color = "var(--danger)"; }
  el.className = cls;
  el.innerHTML = `${v.toFixed(1)}<span class="unit">${unit}</span>`;
  if (bar) { bar.style.width = pct.toFixed(0) + "%"; bar.style.background = color; }
}

function updateWristArrows() {
  if (!STATE.arrowFK || !STATE.arrowIMU) return;
  if (!STATE.arrowsVisible || STATE.fkLive.x == null) {
    STATE.arrowFK.visible = false;
    STATE.arrowIMU.visible = false;
    return;
  }
  // Posiziona entrambe le frecce al wrist center (= TCP - tool offset locale).
  // Usiamo la posizione del wristGroup direttamente.
  const wc = new THREE.Vector3();
  STATE.robot.wristGroup.getWorldPosition(wc);
  STATE.arrowFK.position.copy(wc);
  STATE.arrowIMU.position.copy(wc);

  // Direzione FK: ruota +X mondo per quaternion FK
  const qFK = new THREE.Quaternion(STATE.fkLive.qx, STATE.fkLive.qy, STATE.fkLive.qz, STATE.fkLive.qw);
  const dirFK = new THREE.Vector3(1, 0, 0).applyQuaternion(qFK).normalize();
  STATE.arrowFK.setDirection(dirFK);
  STATE.arrowFK.visible = true;

  if (STATE.imuValid) {
    const qIMU = new THREE.Quaternion(STATE.imuQuat.x, STATE.imuQuat.y, STATE.imuQuat.z, STATE.imuQuat.w);
    const dirIMU = new THREE.Vector3(1, 0, 0).applyQuaternion(qIMU).normalize();
    STATE.arrowIMU.setDirection(dirIMU);
    STATE.arrowIMU.visible = true;
  }
}

function setTargetMarker(t) {
  STATE.target = { ...STATE.target, ...t };
  if (t.x != null) {
    STATE.targetMarker.position.set(t.x, t.y, t.z);
    STATE.targetMarker.visible = true;
    $("coord-tgt-x").textContent = fmt(t.x, 1);
    $("coord-tgt-y").textContent = fmt(t.y, 1);
    $("coord-tgt-z").textContent = fmt(t.z, 1);
    $("coord-tgt-roll").textContent  = fmt(t.roll  ?? 0, 1);
    $("coord-tgt-pitch").textContent = fmt(t.pitch ?? 0, 1);
    $("coord-tgt-yaw").textContent   = fmt(t.yaw   ?? 0, 1);
  }
}

// ============================================================
// IK / SETPOSE
// ============================================================
function ikSolveOnce(x, y, z, roll, pitch, yaw) {
  return new Promise((resolve) => {
    let resolved = false;
    STATE._ikResultOnce = (msg) => {
      if (resolved) return;
      resolved = true;
      STATE._ikResultOnce = null;
      resolve(msg);
    };
    sendCommand("uart", { cmd: `IK_SOLVE ${x} ${y} ${z} ${roll} ${pitch} ${yaw}` });
    setTimeout(() => { if (!resolved) { resolved = true; STATE._ikResultOnce = null; resolve(null); } }, 2500);
  });
}

function waitSetposeDone(timeoutMs = 8000) {
  return new Promise((resolve) => {
    let done = false;
    const t = setTimeout(() => { if (!done) { done = true; resolve(null); } }, timeoutMs);
    STATE._setposeDoneResolvers.push((p) => { if (done) return; done = true; clearTimeout(t); resolve(p); });
  });
}

function fireSetposeDone(payload) {
  // Marker su error chart
  STATE.errEvents.push({ t_ms: performance.now(), label: "done" });
  const r = STATE._setposeDoneResolvers.splice(0);
  r.forEach(f => { try { f(payload); } catch(_){} });
}

async function moveToTarget(x, y, z, roll, pitch, yaw, timeMs) {
  setTargetMarker({ x, y, z, roll, pitch, yaw });
  const ik = await ikSolveOnce(x, y, z, roll, pitch, yaw);
  if (!ik || !ik.reachable) {
    addLog(`[IKL] IK fail (${x},${y},${z})`);
    return false;
  }
  if (Number(ik.error_pos || 0) > 15) {
    addLog(`[IKL] IK errore alto ${ik.error_pos}mm — skip`);
    return false;
  }
  // High-resolution: angoli moltiplicati per 10 (50..1750 = 5.0..175.0°).
  // Eliminata la quantizzazione 1° → movimento sub-degree fluido sui servo digitali.
  const angX10 = (ik.angles_deg || []).map(v => Math.round(v * 10));
  if (angX10.length !== 6) return false;
  sendCommand("uart", { cmd: `SETPOSE_T_HR ${angX10.join(" ")} ${timeMs} ${STATE.profile}` });
  const done = await waitSetposeDone(timeMs + 4000);
  // Dopo arrivo, attendi assestamento meccanico dei servo poi mediа più campioni
  // per smorzare oscillazioni residue post-setpose_done
  await new Promise(r => setTimeout(r, 800));   // settle time servo PWM hobby
  await recordStatSampleAveraged({ samples: 6, intervalMs: 80 });
  return done != null;
}

/** Vecchia versione — singolo snapshot, lasciata per riferimento. */
function recordStatSample() {
  const tg = STATE.target, fk = STATE.fkLive;
  if (tg.x == null || fk.x == null) return;
  const errPos = Math.hypot(tg.x - fk.x, tg.y - fk.y, tg.z - fk.z);
  let errImu = 0;
  if (STATE.imuValid) {
    errImu = quatAngleDiffDeg(STATE.imuQuat, { w: fk.qw, x: fk.qx, y: fk.qy, z: fk.qz });
  }
  STATE.stats.samples.push({ errPos, errImu, t: performance.now() });
  STATE.stats.pointsDone += 1;
  refreshStatsUi();
}

/** Versione mediata: raccoglie N campioni di fk_live a distanza intervalMs,
 *  ne fa la media (X,Y,Z) e calcola l'errore sulla posizione media.
 *  Compensa le oscillazioni residue dei servo PWM hobby post-setpose_done. */
async function recordStatSampleAveraged({ samples = 6, intervalMs = 80 } = {}) {
  const tg = STATE.target;
  if (tg.x == null) return;

  const buf = [];          // {x,y,z, qw,qx,qy,qz, qiw,qix,qiy,qiz, imuValid}
  for (let i = 0; i < samples; i++) {
    const fk = STATE.fkLive;
    if (fk.x != null) {
      buf.push({
        x: fk.x, y: fk.y, z: fk.z,
        qw: fk.qw, qx: fk.qx, qy: fk.qy, qz: fk.qz,
        qiw: STATE.imuQuat.w, qix: STATE.imuQuat.x, qiy: STATE.imuQuat.y, qiz: STATE.imuQuat.z,
        imuValid: STATE.imuValid,
      });
    }
    if (i < samples - 1) await new Promise(r => setTimeout(r, intervalMs));
  }
  if (buf.length === 0) return;

  // Media posizione e quaternioni (per piccoli intorni la media seguita da
  // normalizzazione è una buona approssimazione del Karcher mean).
  const N = buf.length;
  const avg = (key) => buf.reduce((a, b) => a + b[key], 0) / N;
  const xAvg = avg("x"), yAvg = avg("y"), zAvg = avg("z");
  const qFkAvg = quatNormalize({ w: avg("qw"), x: avg("qx"), y: avg("qy"), z: avg("qz") });
  const imuValid = buf.every(b => b.imuValid);
  let errImu = 0;
  if (imuValid) {
    const qImuAvg = quatNormalize({ w: avg("qiw"), x: avg("qix"), y: avg("qiy"), z: avg("qiz") });
    if (STATE.refHome) {
      const dImu = quatMultiply(qImuAvg, quatConjugate(STATE.refHome.qImu));
      const dFk  = quatMultiply(qFkAvg,  quatConjugate(STATE.refHome.qFk));
      errImu = quatAngleDiffDeg(dImu, dFk);
    } else {
      errImu = quatAngleDiffDeg(qImuAvg, qFkAvg);
    }
  }
  const errPos = Math.hypot(tg.x - xAvg, tg.y - yAvg, tg.z - zAvg);

  // Calcola anche jitter (deviazione max sui campioni) come metrica di stabilità
  const jitter = Math.max(...buf.map(b => Math.hypot(b.x-xAvg, b.y-yAvg, b.z-zAvg)));

  STATE.stats.samples.push({ errPos, errImu, jitter, t: performance.now() });
  STATE.stats.pointsDone += 1;
  refreshStatsUi();
}

function refreshStatsUi() {
  const s = STATE.stats.samples;
  $("stat-n").textContent = `${STATE.stats.pointsDone}/${STATE.stats.pointsTotal}`;
  if (s.length === 0) {
    $("stat-rms-pos").textContent = "–";
    $("stat-max-pos").textContent = "–";
    $("stat-rms-imu").textContent = "–";
    const j = $("stat-jitter"); if (j) j.textContent = "–";
    return;
  }
  const rmsPos = Math.sqrt(s.reduce((a, x) => a + x.errPos*x.errPos, 0) / s.length);
  const maxPos = s.reduce((m, x) => Math.max(m, x.errPos), 0);
  const rmsImu = Math.sqrt(s.reduce((a, x) => a + x.errImu*x.errImu, 0) / s.length);
  $("stat-rms-pos").textContent = rmsPos.toFixed(2);
  $("stat-max-pos").textContent = maxPos.toFixed(2);
  $("stat-rms-imu").textContent = rmsImu.toFixed(2);
  // Jitter medio (= media della max deviazione fra campioni, per ogni punto)
  const j = $("stat-jitter");
  if (j) {
    const jitterSamples = s.filter(x => x.jitter != null);
    if (jitterSamples.length === 0) {
      j.textContent = "–";
    } else {
      const jAvg = jitterSamples.reduce((a, x) => a + x.jitter, 0) / jitterSamples.length;
      j.textContent = jAvg.toFixed(2);
    }
  }
}

function resetStats() {
  STATE.stats = { samples: [], pointsDone: 0, pointsTotal: 0 };
  refreshStatsUi();
  STATE.trailPoints = [];
  STATE.trailLine.geometry.setDrawRange(0, 0);
}

/** Media n campioni di telemetria (smoothing) per ottenere quaternion affidabili
 *  IMU + FK al momento corrente. Restituisce { qImu, qFk } o null se mancano dati. */
async function _samplePolsoQuats(timeoutMs = 700) {
  return new Promise((resolve) => {
    const samplesImu = [], samplesFk = [];
    const start = performance.now();
    const tick = () => {
      const elapsed = performance.now() - start;
      // Ad ogni tick raccoglie lo snapshot più recente
      if (STATE.imuValid) samplesImu.push({ ...STATE.imuQuat });
      const fk = STATE.fkLive;
      if (Math.hypot(fk.qw||0, fk.qx||0, fk.qy||0, fk.qz||0) > 0.01) {
        samplesFk.push({ w: fk.qw, x: fk.qx, y: fk.qy, z: fk.qz });
      }
      if (elapsed < timeoutMs) {
        setTimeout(tick, 50);
      } else {
        if (samplesImu.length === 0 || samplesFk.length === 0) {
          resolve(null); return;
        }
        const avg = (qs) => {
          const w = qs.reduce((a,q)=>a+q.w,0)/qs.length;
          const x = qs.reduce((a,q)=>a+q.x,0)/qs.length;
          const y = qs.reduce((a,q)=>a+q.y,0)/qs.length;
          const z = qs.reduce((a,q)=>a+q.z,0)/qs.length;
          return quatNormalize({ w, x, y, z });
        };
        resolve({ qImu: avg(samplesImu), qFk: avg(samplesFk) });
      }
    };
    tick();
  });
}

/** Va a HOME, attende che il robot sia stabile, registra il quaternion di
 *  riferimento per IMU + FK. Ogni successiva metrica "Δ IMU vs FK" sarà
 *  relativa al delta dal HOME → elimina bias di montaggio IMU.
 *  Restituisce true se ok. */
async function goHomeAndZeroReference({ silent = false } = {}) {
  if (!silent) {
    $("demo-status-text").textContent = "Vai a HOME per azzerare riferimento IMU…";
    setDemoDot("active");
  }
  // Assicurati ENABLE
  sendCommand("uart", { cmd: "ENABLE" });
  await new Promise(r => setTimeout(r, 200));
  // SETPOSE_T HOME (90,90,90,90,90,90 virtual). Il backend Pi traduce in physical.
  // Useremo HOME comando esplicito che usa offsets del settings.
  sendCommand("uart", { cmd: "HOME" });
  await waitSetposeDone(7000);
  await new Promise(r => setTimeout(r, 600));   // assestamento meccanico + IMU
  const ref = await _samplePolsoQuats(800);
  if (!ref) {
    if (!silent) {
      $("demo-status-text").textContent = "✗ Riferimento NON registrato (IMU/FK non validi)";
      setDemoDot("fail");
    }
    addLog("[IKL] Azzeramento riferimento fallito: IMU/FK non validi");
    return false;
  }
  STATE.refHome = { qImu: ref.qImu, qFk: ref.qFk, ts: Date.now() };
  addLog(`[IKL] Riferimento HOME azzerato: qImu=(${ref.qImu.w.toFixed(2)},${ref.qImu.x.toFixed(2)},${ref.qImu.y.toFixed(2)},${ref.qImu.z.toFixed(2)})`);
  if (!silent) {
    $("demo-status-text").textContent = "Riferimento HOME registrato ✓ — pronto per validazione";
    setDemoDot("done");
  }
  return true;
}

// ============================================================
// Pattern generators
// ============================================================
function genCubePoints(cx, cy, cz, size, n) {
  const h = size / 2;
  const verts = [
    [-h,-h,-h], [+h,-h,-h], [+h,+h,-h], [-h,+h,-h],
    [-h,-h,+h], [+h,-h,+h], [+h,+h,+h], [-h,+h,+h],
  ];
  // Se n >= 8 prende tutti, altrimenti subset
  return verts.slice(0, Math.max(2, Math.min(8, n))).map(v => ({ x: cx + v[0], y: cy + v[1], z: cz + v[2] }));
}
function genCirclePoints(cx, cy, cz, radius, n) {
  const out = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    out.push({ x: cx + radius*Math.cos(a), y: cy + radius*Math.sin(a), z: cz });
  }
  return out;
}
function genLinePoints(cx, cy, cz, length, n) {
  const out = [];
  const step = length / Math.max(1, n - 1);
  for (let i = 0; i < n; i++) {
    out.push({ x: cx + (-length/2 + step*i), y: cy, z: cz });
  }
  return out;
}
function genSplinePoints(cx, cy, cz, size, n) {
  // Curva di Bezier 3D fra 4 control points
  const ctrl = [
    [cx - size/2, cy - size/2, cz - size/3],
    [cx - size/3, cy + size/2, cz + size/3],
    [cx + size/3, cy - size/2, cz + size/2],
    [cx + size/2, cy + size/2, cz - size/4],
  ];
  const out = [];
  for (let i = 0; i < n; i++) {
    const t = i / Math.max(1, n - 1);
    const u = 1 - t;
    const x = u*u*u*ctrl[0][0] + 3*u*u*t*ctrl[1][0] + 3*u*t*t*ctrl[2][0] + t*t*t*ctrl[3][0];
    const y = u*u*u*ctrl[0][1] + 3*u*u*t*ctrl[1][1] + 3*u*t*t*ctrl[2][1] + t*t*t*ctrl[3][1];
    const z = u*u*u*ctrl[0][2] + 3*u*u*t*ctrl[1][2] + 3*u*t*t*ctrl[2][2] + t*t*t*ctrl[3][2];
    out.push({ x, y, z });
  }
  return out;
}
function genRandomPoints(cx, cy, cz, size, n) {
  const out = [];
  for (let i = 0; i < n; i++) {
    out.push({
      x: cx + (Math.random() - 0.5) * size,
      y: cy + (Math.random() - 0.5) * size,
      z: cz + (Math.random() - 0.5) * size,
    });
  }
  return out;
}

function generatePattern(pattern, opts) {
  const { cx, cy, cz, size, n } = opts;
  switch (pattern) {
    case "cube":   return genCubePoints(cx, cy, cz, size, n);
    case "circle": return genCirclePoints(cx, cy, cz, size/2, n);
    case "line":   return genLinePoints(cx, cy, cz, size, n);
    case "spline": return genSplinePoints(cx, cy, cz, size, n);
    case "random": return genRandomPoints(cx, cy, cz, size, n);
    default:       return [];
  }
}

// ============================================================
// Demo runner
// ============================================================
async function runDemo() {
  if (STATE.demoRunning) return;
  STATE.demoRunning = true; STATE.demoAbort = false;
  $("btn-demo-start").disabled = true;
  $("pill-demo").textContent = "init";
  $("pill-demo").className = "diag-value state-pill state-warn";
  $("ikl-step-indicator").style.display = "";

  // 1. Pre-demo: vai a HOME e azzera il riferimento IMU per validazione pulita
  $("hud-step-text").textContent = "PRE-DEMO · HOME + zero IMU";
  const refOk = await goHomeAndZeroReference({ silent: false });
  if (!refOk) {
    $("pill-demo").textContent = "idle";
    $("pill-demo").className = "diag-value state-pill state-off";
    $("ikl-step-indicator").style.display = "none";
    STATE.demoRunning = false;
    $("btn-demo-start").disabled = false;
    return;
  }

  $("pill-demo").textContent = "running";
  $("pill-demo").className = "diag-value state-pill state-on";

  const opts = {
    cx: parseFloat($("demo-cx").value) || 180,
    cy: parseFloat($("demo-cy").value) || 0,
    cz: parseFloat($("demo-cz").value) || 200,
    size: parseFloat($("demo-size").value) || 100,
    n: parseInt($("demo-n").value, 10) || 8,
  };
  const stepMs = parseInt($("demo-step-ms").value, 10) || 1500;
  STATE.profile = $("demo-profile").value || "RTR5";
  const loop = $("demo-loop").checked;

  resetStats();

  do {
    const pts = generatePattern(STATE.demoPattern, opts);
    STATE.demoPoints = pts;
    STATE.stats.pointsTotal += pts.length;
    refreshStatsUi();

    for (let i = 0; i < pts.length; i++) {
      if (STATE.demoAbort) break;
      const p = pts[i];
      const label = `${STATE.demoPattern.toUpperCase()} step ${i+1}/${pts.length}`;
      $("hud-step-text").textContent = label;
      $("demo-status-text").textContent = label + ` → (${p.x.toFixed(0)}, ${p.y.toFixed(0)}, ${p.z.toFixed(0)})`;
      setDemoDot("active");
      addLog(`[IKL-DEMO] ${label}`);
      const ok = await moveToTarget(p.x, p.y, p.z, 0, 0, 0, stepMs);
      if (!ok) {
        $("demo-status-text").textContent = `Errore a step ${i+1}: target irraggiungibile`;
        setDemoDot("fail");
        if (!STATE.demoAbort) await new Promise(r => setTimeout(r, 600));
      }
    }
  } while (loop && !STATE.demoAbort);

  if (STATE.demoAbort) {
    $("demo-status-text").textContent = `Demo interrotta`;
    setDemoDot("fail");
  } else {
    $("demo-status-text").textContent = `Demo completata · RMS pos ${$("stat-rms-pos").textContent}mm`;
    setDemoDot("done");
  }
  $("pill-demo").textContent = "idle";
  $("pill-demo").className = "diag-value state-pill state-off";
  $("ikl-step-indicator").style.display = "none";
  STATE.demoRunning = false;
  $("btn-demo-start").disabled = false;
}

function setDemoDot(state) {
  const el = $("demo-dot");
  if (!el) return;
  el.className = "ikl-demo-dot " + state;
}

// ============================================================
// Error chart
// ============================================================
function drawErrChart() {
  const canvas = $("ikl-err-chart");
  if (!canvas) return;
  const wrap = $("ikl-err-chart-wrap");
  const dpr = window.devicePixelRatio || 1;
  const W = wrap.clientWidth - 16, H = wrap.clientHeight - 16;
  canvas.width = W * dpr; canvas.height = H * dpr;
  canvas.style.width = W + "px"; canvas.style.height = H + "px";
  const ctx = canvas.getContext("2d");
  ctx.setTransform(1,0,0,1,0,0);
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, W, H);

  const padL = 50, padR = 14, padT = 14, padB = 24;
  const innerW = W - padL - padR;
  const innerH = H - padT - padB;

  // Griglia
  ctx.strokeStyle = "rgba(0,229,255,0.12)"; ctx.lineWidth = 1;
  for (let i = 0; i <= 10; i++) {
    const x = padL + i * innerW / 10;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + innerH); ctx.stroke();
  }

  if (STATE.errBuffer.length < 2) return;

  const tEnd = performance.now();
  const tStart = tEnd - STATE.errWindowMs;
  const xOf = t => padL + ((t - tStart) / STATE.errWindowMs) * innerW;

  // Auto-scale Y separato per pos e ori, ma stesso asse: normalizzo per max
  let maxPos = 0, maxImu = 0;
  for (const s of STATE.errBuffer) {
    if (s.errPos > maxPos) maxPos = s.errPos;
    if (s.errImu > maxImu) maxImu = s.errImu;
  }
  maxPos = Math.max(20, maxPos * 1.15);
  maxImu = Math.max(8,  maxImu * 1.15);

  const yPos = v => padT + innerH * (1 - v / maxPos);
  const yImu = v => padT + innerH * (1 - v / maxImu);

  // Marker eventi
  ctx.strokeStyle = "rgba(176,127,217,0.6)";
  ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
  for (const e of STATE.errEvents) {
    const x = xOf(e.t_ms);
    if (x < padL || x > padL + innerW) continue;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + innerH); ctx.stroke();
  }
  ctx.setLineDash([]);

  // err posizione (verde)
  ctx.strokeStyle = "#5dffa8"; ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  for (const s of STATE.errBuffer) {
    const x = xOf(s.t_ms), y = yPos(s.errPos);
    if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, y); }
  }
  ctx.stroke();

  // err IMU (blu)
  ctx.strokeStyle = "#5db8ff"; ctx.lineWidth = 1.7;
  ctx.beginPath();
  started = false;
  for (const s of STATE.errBuffer) {
    const x = xOf(s.t_ms), y = yImu(s.errImu);
    if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, y); }
  }
  ctx.stroke();

  // Labels Y
  ctx.fillStyle = "#5dffa8"; ctx.font = "11px system-ui, sans-serif";
  ctx.fillText(`${maxPos.toFixed(0)} mm`, 6, padT + 12);
  ctx.fillStyle = "#5db8ff";
  ctx.fillText(`${maxImu.toFixed(0)}°`,   6, padT + 28);
  ctx.fillStyle = "#9db1cc";
  ctx.fillText("0", 6, padT + innerH - 2);
  ctx.fillText("−30 s", padL,            H - 6);
  ctx.fillText("0 (now)", padL + innerW - 42, H - 6);
}

// ============================================================
// Init
// ============================================================
function initCollapse() {
  document.querySelectorAll(".ikl-section-header").forEach(h => {
    h.addEventListener("click", () => {
      const tid = h.dataset.target;
      const body = $(tid);
      if (!body) return;
      const collapsed = h.classList.toggle("collapsed");
      body.style.display = collapsed ? "none" : "";
      setTimeout(onResize, 50);
    });
  });
}

function setupSceneViews() {
  document.querySelectorAll("[data-view]").forEach(b => {
    b.addEventListener("click", () => setView(b.dataset.view));
  });
  $("btn-3d-clear-trail")?.addEventListener("click", () => {
    STATE.trailPoints = [];
    STATE.trailLine.geometry.setDrawRange(0, 0);
  });
  $("btn-3d-toggle-arrows")?.addEventListener("click", () => {
    STATE.arrowsVisible = !STATE.arrowsVisible;
    if (!STATE.arrowsVisible) {
      STATE.arrowFK.visible = false; STATE.arrowIMU.visible = false;
    }
  });
  $("btn-3d-toggle-smooth")?.addEventListener("click", (e) => {
    STATE.viz.enabled = !STATE.viz.enabled;
    e.currentTarget.textContent = STATE.viz.enabled ? "✦ Smooth ON" : "✦ Smooth OFF";
    addLog(`[IKL] visualizzazione 3D ${STATE.viz.enabled ? "smussata (EMA α=" + STATE.viz.alpha + ")" : "raw"}`);
  });
}

function setupPatterns() {
  document.querySelectorAll(".ikl-pattern-tab").forEach(b => {
    b.addEventListener("click", () => {
      document.querySelectorAll(".ikl-pattern-tab").forEach(x => x.classList.remove("active"));
      b.classList.add("active");
      STATE.demoPattern = b.dataset.pattern;
      addLog(`[IKL] pattern: ${STATE.demoPattern}`);
    });
  });
}

function setupDemo() {
  $("btn-demo-start")?.addEventListener("click", runDemo);
  $("btn-demo-stop")?.addEventListener("click", () => {
    STATE.demoAbort = true;
    sendCommand("uart", { cmd: "STOP" });
  });
  $("btn-demo-stats-reset")?.addEventListener("click", resetStats);
  $("btn-go-home")?.addEventListener("click", () => sendCommand("uart", { cmd: "HOME" }));
  $("btn-recover")?.addEventListener("click", () => {
    sendCommand("uart", { cmd: "ENABLE" });
    addLog("[IKL] Recovery (ENABLE → AUTO_HOME)");
  });
  $("btn-zero-ref")?.addEventListener("click", async () => {
    if (STATE.demoRunning) return;
    $("btn-zero-ref").disabled = true;
    await goHomeAndZeroReference({ silent: false });
    $("btn-zero-ref").disabled = false;
  });

  // Aggiorna indicatore stato riferimento ogni 500 ms
  setInterval(() => {
    const el = $("ref-status");
    if (!el) return;
    if (STATE.refHome) {
      const dt = ((Date.now() - STATE.refHome.ts) / 1000).toFixed(0);
      el.textContent = `riferimento: registrato (${dt}s fa)`;
      el.style.color = "var(--ikl-tcp)";
      el.style.fontStyle = "normal";
    } else {
      el.textContent = "riferimento: non registrato";
      el.style.color = "var(--muted)";
      el.style.fontStyle = "italic";
    }
  }, 500);
}

function setupManual() {
  $("btn-man-ik")?.addEventListener("click", () => {
    const x = parseFloat($("man-x").value), y = parseFloat($("man-y").value), z = parseFloat($("man-z").value);
    const r = parseFloat($("man-roll").value), p = parseFloat($("man-pitch").value), yw = parseFloat($("man-yaw").value);
    setTargetMarker({ x, y, z, roll: r, pitch: p, yaw: yw });
    sendCommand("uart", { cmd: `IK_SOLVE ${x} ${y} ${z} ${r} ${p} ${yw}` });
  });
  $("btn-man-exec")?.addEventListener("click", async () => {
    const x = parseFloat($("man-x").value), y = parseFloat($("man-y").value), z = parseFloat($("man-z").value);
    const r = parseFloat($("man-roll").value), p = parseFloat($("man-pitch").value), yw = parseFloat($("man-yaw").value);
    setTargetMarker({ x, y, z, roll: r, pitch: p, yaw: yw });
    const stepMs = parseInt($("demo-step-ms").value, 10) || 1500;
    const ok = await moveToTarget(x, y, z, r, p, yw, stepMs);
    showFeedback("fb-manual", ok, ok ? "Eseguito" : "Fallito");
  });
}

function init() {
  if (THREE) {
    build3DScene();
    setupSceneViews();
  }
  initCollapse();
  setupPatterns();
  setupDemo();
  setupManual();

  registerTelemetryHandler(applyTelemetry);
  registerIkResultHandler((msg) => {
    if (typeof STATE._ikResultOnce === "function") {
      const cb = STATE._ikResultOnce;
      STATE._ikResultOnce = null;
      try { cb(msg); } catch(_) {}
    }
  });
  registerSetposeDoneHandler(fireSetposeDone);
  registerSettingsHandler(msg => {
    if (msg.type === "settings") {
      STATE.settings = msg;
      addLog(`[IKL] settings caricati: offsets=${JSON.stringify(msg.offsets)} dirs=${JSON.stringify(msg.dirs)}`);
    }
  });

  connectJ5Dashboard();
  setTimeout(() => sendCommand("get_settings", {}), 500);

  // Loop disegno error chart
  setInterval(drawErrChart, 80);
  window.addEventListener("resize", drawErrChart);

  addLog("IK Live pronta — pattern attivo: cube");
}

document.addEventListener("DOMContentLoaded", init);
