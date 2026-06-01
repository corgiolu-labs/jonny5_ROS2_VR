/**
 * pickplace.js — Pagina Pick & Place Test Bench (JONNY5-4.0)
 *
 * Funzionalità:
 *   1. Vista 3D Three.js del braccio (FK live continuo + marker target IK + trail)
 *   2. Calcolo IK + esecuzione SETPOSE con profilo selezionabile
 *   3. Visualizzazione profili di moto RTR3/RTR5/BCB/BB (canvas 2D)
 *   4. Pannello attuatori PP1/PP2 (vacuum + valvola)
 *   5. Sequenza pick&place automatica (orchestrata via SETPOSE_T + setpose_done)
 *
 * Three.js è caricato via UMD globale (window.THREE) da pickplace.html.
 */

import {
  connectJ5Dashboard,
  sendCommand,
  addLog,
  registerTelemetryHandler,
  registerIkResultHandler,
  registerSetposeDoneHandler,
  registerUartResponseHandler,
  registerSettingsHandler,
} from "../../../shared/js/j5_common.js";

const THREE = window.THREE;
if (!THREE) {
  console.error("[PP] Three.js non caricato — controlla che three.min.js sia raggiungibile");
}

// ============================================================
// Stato globale
// ============================================================
const STATE = {
  // 3D scene
  scene: null, camera: null, renderer: null, controlsState: { autoIso: true },
  robot: { pivots: [], links: [] },        // pivot Group per ogni giunto + link mesh
  tcpLiveMarker: null, targetMarker: null,
  trailPoints: [], trailLine: null, trailMax: 600,

  // FK/IK live
  jointAngles: [90, 90, 90, 90, 90, 90],   // virtual deg (HOME=90)
  fkLive: { x: null, y: null, z: null, roll: null, pitch: null, yaw: null },

  // Settings da Pi (offsets, dirs, vel_max, profile)
  settings: null,
  poeM: null,                              // matrice M 4x4 da j5_poe_params.json (per geometria)

  // Profilo selezionato
  profile: "RTR5",

  // Sequenza
  seqRunning: false, seqAbort: false,

  // PP duty cache
  ppDuty: { 1: 0, 2: 0 },

  // setpose_done waiter
  _setposeDoneResolvers: [],
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
// 3D scene — costruzione braccio gerarchico a primitive
// ============================================================
// Parametri geometrici (dal POE M default in mm — saranno aggiornati via WS).
const GEOM = {
  baseRadius: 50, baseHeight: 30,
  shoulderZ: 94, shoulderRadius: 28,
  upperArmLen: 60, upperArmRadius: 18,    // gomito a Z=154 = 94+60
  forearmLen: 157, forearmRadius: 14,     // wrist a Z=311 = 154+157
  wristRadius: 14, wristLen: 24,
  toolOffsetX: 60, toolRadius: 7,
};

function makeMaterial(color, opts = {}) {
  return new THREE.MeshStandardMaterial({
    color, metalness: 0.4, roughness: 0.55,
    emissive: opts.emissive || 0x000000,
    emissiveIntensity: opts.emissiveIntensity || 0,
  });
}

function build3DScene() {
  const wrap = $("scene-3d-wrap");
  const canvas = $("scene-3d-canvas");
  const w = wrap.clientWidth, h = wrap.clientHeight;

  STATE.scene = new THREE.Scene();
  STATE.scene.background = null;       // trasparente sopra il gradient CSS
  STATE.scene.fog = new THREE.Fog(0x05070c, 800, 1600);

  STATE.camera = new THREE.PerspectiveCamera(38, w / h, 1, 4000);
  setView("iso");

  STATE.renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
  STATE.renderer.setPixelRatio(window.devicePixelRatio || 1);
  STATE.renderer.setSize(w, h, false);
  STATE.renderer.shadowMap.enabled = false;

  // Luci
  const amb = new THREE.AmbientLight(0xffffff, 0.55);
  STATE.scene.add(amb);
  const key = new THREE.DirectionalLight(0xc8e2ff, 0.85);
  key.position.set(300, 600, 400);
  STATE.scene.add(key);
  const fill = new THREE.DirectionalLight(0x5db8ff, 0.35);
  fill.position.set(-400, 200, -200);
  STATE.scene.add(fill);

  // Pavimento + griglia
  const grid = new THREE.GridHelper(800, 16, 0x3d9dff, 0x1a2a45);
  grid.material.opacity = 0.45; grid.material.transparent = true;
  STATE.scene.add(grid);
  // Assi
  const axes = new THREE.AxesHelper(120);
  axes.material.depthTest = false;
  STATE.scene.add(axes);

  // Costruisci braccio
  buildRobot();

  // Marker TCP live (verde) e target (magenta)
  STATE.tcpLiveMarker = new THREE.Mesh(
    new THREE.SphereGeometry(10, 24, 16),
    makeMaterial(0x5dffa8, { emissive: 0x0a4a2a, emissiveIntensity: 0.6 }),
  );
  STATE.tcpLiveMarker.visible = false;
  STATE.scene.add(STATE.tcpLiveMarker);

  STATE.targetMarker = new THREE.Mesh(
    new THREE.SphereGeometry(11, 24, 16),
    makeMaterial(0xff5dbb, { emissive: 0x551a3a, emissiveIntensity: 0.7 }),
  );
  STATE.targetMarker.visible = false;
  STATE.scene.add(STATE.targetMarker);

  // Trail line (positions buffer aggiornato dinamicamente)
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(STATE.trailMax * 3), 3));
  const trailMat = new THREE.LineBasicMaterial({ color: 0x5db8ff, transparent: true, opacity: 0.6 });
  STATE.trailLine = new THREE.Line(trailGeom, trailMat);
  STATE.trailLine.frustumCulled = false;
  STATE.scene.add(STATE.trailLine);

  // Resize listener
  window.addEventListener("resize", onResize);
  // Mouse drag = orbit semplice
  attachMouseOrbit(canvas);

  // Render loop
  animate();
}

function buildRobot() {
  // Piramide gerarchica: ogni pivot è un Group nested.
  // I pivot ruotano attorno a Z (BASE/YAW), Y (SPALLA/GOMITO/PITCH) o X (ROLL).
  const pivots = [];
  const links = [];

  // Basement
  const basement = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.baseRadius, GEOM.baseRadius * 1.05, GEOM.baseHeight, 32),
    makeMaterial(0x1f3354),
  );
  basement.rotation.x = Math.PI / 2;
  basement.position.z = GEOM.baseHeight / 2;
  STATE.scene.add(basement);

  // P0: BASE (Z rot)
  const p0 = new THREE.Group();   p0.position.set(0, 0, 0);
  STATE.scene.add(p0);
  pivots.push(p0);

  // Colonna fissa fino a SPALLA
  const column = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.shoulderRadius * 0.85, GEOM.shoulderRadius, GEOM.shoulderZ, 24),
    makeMaterial(0x2a4675),
  );
  column.rotation.x = Math.PI / 2;
  column.position.z = GEOM.shoulderZ / 2;
  p0.add(column);

  // P1: SPALLA (Y rot) at z=shoulderZ
  const p1 = new THREE.Group();   p1.position.set(0, 0, GEOM.shoulderZ);
  p0.add(p1);
  pivots.push(p1);

  // Joint sphere SPALLA
  const sJoint = new THREE.Mesh(new THREE.SphereGeometry(GEOM.shoulderRadius, 24, 16), makeMaterial(0x12c2b2));
  p1.add(sJoint);

  // Upper arm (link) lungo asse Z locale di P1
  const upperArm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.upperArmRadius, GEOM.upperArmRadius * 1.05, GEOM.upperArmLen, 20),
    makeMaterial(0x3d9dff),
  );
  upperArm.rotation.x = Math.PI / 2;
  upperArm.position.z = GEOM.upperArmLen / 2;
  p1.add(upperArm);
  links.push(upperArm);

  // P2: GOMITO (Y rot) at z=upperArmLen
  const p2 = new THREE.Group();   p2.position.set(0, 0, GEOM.upperArmLen);
  p1.add(p2);
  pivots.push(p2);

  const eJoint = new THREE.Mesh(new THREE.SphereGeometry(GEOM.upperArmRadius * 1.4, 22, 14), makeMaterial(0x12c2b2));
  p2.add(eJoint);

  // Forearm
  const forearm = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.forearmRadius, GEOM.forearmRadius * 1.05, GEOM.forearmLen, 18),
    makeMaterial(0x3d9dff),
  );
  forearm.rotation.x = Math.PI / 2;
  forearm.position.z = GEOM.forearmLen / 2;
  p2.add(forearm);
  links.push(forearm);

  // P3: YAW (Z rot) at z=forearmLen
  const p3 = new THREE.Group();   p3.position.set(0, 0, GEOM.forearmLen);
  p2.add(p3);
  pivots.push(p3);

  const wristJoint = new THREE.Mesh(new THREE.SphereGeometry(GEOM.wristRadius * 1.3, 22, 14), makeMaterial(0xb07fd9));
  p3.add(wristJoint);

  // P4: PITCH (Y rot)
  const p4 = new THREE.Group();   p4.position.set(0, 0, 0);
  p3.add(p4);
  pivots.push(p4);

  // P5: ROLL (X rot)
  const p5 = new THREE.Group();   p5.position.set(0, 0, 0);
  p4.add(p5);
  pivots.push(p5);

  // Tool: corto link X + sferetta TCP
  const toolLink = new THREE.Mesh(
    new THREE.CylinderGeometry(GEOM.toolRadius, GEOM.toolRadius * 1.1, GEOM.toolOffsetX, 14),
    makeMaterial(0xff9d3d),
  );
  toolLink.rotation.z = Math.PI / 2;
  toolLink.position.x = GEOM.toolOffsetX / 2;
  p5.add(toolLink);

  const tcp = new THREE.Mesh(new THREE.SphereGeometry(GEOM.toolRadius * 1.5, 18, 12), makeMaterial(0xffb84d, {emissive:0x4a3300,emissiveIntensity:0.4}));
  tcp.position.x = GEOM.toolOffsetX;
  p5.add(tcp);

  STATE.robot.pivots = pivots;
  STATE.robot.links = links;
  STATE.robot.tcpLocal = tcp;
}

function updateRobotPose(anglesVirtualDeg) {
  // anglesVirtualDeg: [B, S, G, Y, P, R] in spazio "virtuale" (90 = HOME).
  // Convenzione assi: pivot Three.js gerarchici allineati al solver POE,
  // tutti i giunti con segno positivo.
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

// Vista preset
function setView(name) {
  if (!STATE.camera) return;
  const r = 700, h = 450;
  switch (name) {
    case "iso":   STATE.camera.position.set( r,  r,  h); break;
    case "top":   STATE.camera.position.set( 0.01, 0.01, 1100); break;
    case "front": STATE.camera.position.set( 0.01, -1100, 350); break;
    case "side":  STATE.camera.position.set( 1100, 0.01, 350); break;
  }
  STATE.camera.up.set(0, 0, 1);
  STATE.camera.lookAt(0, 0, 200);
}

function attachMouseOrbit(canvas) {
  let dragging = false, lx = 0, ly = 0, az = Math.PI/4, el = Math.PI/3.5, dist = 1100;
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
    dist = clamp(dist * (1 + e.deltaY * 0.001), 350, 2500);
    apply();
  }, { passive: false });
  apply();
}

function onResize() {
  if (!STATE.renderer) return;
  const wrap = $("scene-3d-wrap");
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
  if (STATE.renderer) STATE.renderer.render(STATE.scene, STATE.camera);
}

// ============================================================
// Telemetria → aggiorna scene + readout
// ============================================================
function applyTelemetry(t) {
  // FK live (mm e gradi)
  const fx = t.fk_live_x_mm, fy = t.fk_live_y_mm, fz = t.fk_live_z_mm;
  const valid = t.fk_live_valid !== false && fx != null && fz != null;
  if (valid) {
    // Buffer live per il grafico esecuzione (vel/acc TCP).
    pushLiveSample(performance.now(), fx, fy, fz);

    STATE.fkLive = {
      x: fx, y: fy, z: fz,
      roll: t.fk_live_roll, pitch: t.fk_live_pitch, yaw: t.fk_live_yaw,
    };
    if (STATE.tcpLiveMarker) {
      STATE.tcpLiveMarker.position.set(fx, fy, fz);
      STATE.tcpLiveMarker.visible = true;
    }
    // Aggiorna trail solo se posizione cambia significativamente
    const last = STATE.trailPoints[STATE.trailPoints.length - 1];
    if (!last || Math.hypot(last[0]-fx, last[1]-fy, last[2]-fz) > 2.5) {
      pushTrailPoint(fx, fy, fz);
    }
    $("tcp-live-x").textContent = fmt(fx, 0);
    $("tcp-live-y").textContent = fmt(fy, 0);
    $("tcp-live-z").textContent = fmt(fz, 0);
    $("t-fk-x").textContent = fmt(fx, 1);
    $("t-fk-y").textContent = fmt(fy, 1);
    $("t-fk-z").textContent = fmt(fz, 1);
    $("t-fk-roll").textContent  = fmt(t.fk_live_roll, 1);
    $("t-fk-pitch").textContent = fmt(t.fk_live_pitch, 1);
    $("t-fk-yaw").textContent   = fmt(t.fk_live_yaw, 1);
    $("pill-tcp").textContent = `${fmt(fx,0)} · ${fmt(fy,0)} · ${fmt(fz,0)}`;
    $("pill-tcp").className = "diag-value state-pill state-on";

    // Δ vs target marker (se visibile)
    if (STATE.targetMarker.visible) {
      const tp = STATE.targetMarker.position;
      const d = Math.hypot(tp.x - fx, tp.y - fy, tp.z - fz);
      $("tcp-target-delta").textContent = `${d.toFixed(1)} mm`;
    } else {
      $("tcp-target-delta").textContent = "–";
    }
  }

  // Angoli giunti FISICI dalla telemetria → li uso anche per ruotare il braccio 3D.
  // I pivot vogliono valori "virtuali" (90 = HOME); applico inversione per dirs/offsets se note.
  // Per semplicità: se settings ancora non caricate, considera physical == virtual.
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
    STATE.jointAngles = angV;
    updateRobotPose(angV);
    const ids = ["t-deg-b","t-deg-s","t-deg-g","t-deg-y","t-deg-p","t-deg-r"];
    angV.forEach((v,i) => $(ids[i]).textContent = fmt(v, 0));
  }

  // Stato robot pill
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
}

// ============================================================
// Persistenza target cartesiano (localStorage)
// ============================================================
const TARGET_LS_KEY = "j5_pickplace_target_v1";
const TARGET_DEFAULT = { x: 60, y: 0, z: 321, roll: 0, pitch: 0, yaw: 0 };

function _writeTargetUi(t) {
  $("ik-in-x").value = t.x;
  $("ik-in-y").value = t.y;
  $("ik-in-z").value = t.z;
  $("ik-in-roll").value  = t.roll;
  $("ik-in-pitch").value = t.pitch;
  $("ik-in-yaw").value   = t.yaw;
}

function _readTargetUi() {
  return {
    x: parseFloat($("ik-in-x").value) || 0,
    y: parseFloat($("ik-in-y").value) || 0,
    z: parseFloat($("ik-in-z").value) || 0,
    roll:  parseFloat($("ik-in-roll").value)  || 0,
    pitch: parseFloat($("ik-in-pitch").value) || 0,
    yaw:   parseFloat($("ik-in-yaw").value)   || 0,
  };
}

function _renderTargetSavedInfo(savedAt) {
  const wrap = $("target-saved-info");
  if (!wrap) return;
  const em = wrap.querySelector("em");
  if (savedAt) {
    const dt = new Date(savedAt);
    em.textContent = `Target persistito (browser) — salvato ${dt.toLocaleString()}`;
    wrap.style.display = "";
  } else {
    em.textContent = "";
    wrap.style.display = "none";
  }
}

function saveTargetToLocal() {
  const t = _readTargetUi();
  const payload = { ...t, savedAt: Date.now() };
  try {
    localStorage.setItem(TARGET_LS_KEY, JSON.stringify(payload));
    _renderTargetSavedInfo(payload.savedAt);
    showFeedback("fb-ik", true, "Target salvato ✓");
    addLog(`[PP] Target salvato: X=${t.x} Y=${t.y} Z=${t.z} RPY=(${t.roll},${t.pitch},${t.yaw})`);
    showTargetMarker();
  } catch (e) {
    showFeedback("fb-ik", false, "Salvataggio fallito");
    addLog(`[PP] Salvataggio target fallito: ${e}`);
  }
}

function loadTargetFromLocal() {
  try {
    const raw = localStorage.getItem(TARGET_LS_KEY);
    if (!raw) return false;
    const t = JSON.parse(raw);
    if (typeof t.x !== "number" || typeof t.z !== "number") return false;
    _writeTargetUi(t);
    _renderTargetSavedInfo(t.savedAt);
    addLog(`[PP] Target caricato dal browser: X=${t.x} Y=${t.y} Z=${t.z} RPY=(${t.roll},${t.pitch},${t.yaw})`);
    return true;
  } catch (_) { return false; }
}

function resetTargetToDefault() {
  _writeTargetUi(TARGET_DEFAULT);
  try { localStorage.removeItem(TARGET_LS_KEY); } catch(_){}
  _renderTargetSavedInfo(null);
  showFeedback("fb-ik", true, "Default ripristinato");
  showTargetMarker();
  addLog("[PP] Target reimpostato ai default");
}

// ============================================================
// IK / SETPOSE
// ============================================================
function readTargetCart() {
  return {
    x: parseFloat($("ik-in-x").value),
    y: parseFloat($("ik-in-y").value),
    z: parseFloat($("ik-in-z").value),
    roll:  parseFloat($("ik-in-roll").value),
    pitch: parseFloat($("ik-in-pitch").value),
    yaw:   parseFloat($("ik-in-yaw").value),
  };
}

function showTargetMarker() {
  const t = readTargetCart();
  if ([t.x,t.y,t.z].some(v => isNaN(v))) { STATE.targetMarker.visible = false; return; }
  STATE.targetMarker.position.set(t.x, t.y, t.z);
  STATE.targetMarker.visible = true;
}

function ikSolve() {
  const t = readTargetCart();
  const cmd = `IK_SOLVE ${t.x} ${t.y} ${t.z} ${t.roll} ${t.pitch} ${t.yaw}`;
  sendCommand("uart", { cmd });
  showTargetMarker();
  addLog(`[PP] IK_SOLVE → ${t.x},${t.y},${t.z} | ${t.roll},${t.pitch},${t.yaw}`);
}

function applyIkResult(msg) {
  const ok = msg.reachable === true;
  $("ik-r-reachable").textContent = ok ? "✓ SI" : "✗ NO";
  $("ik-r-reachable").className = "r-value " + (ok ? "ok" : "err");
  $("ik-r-iter").textContent = msg.iterations ?? "–";
  const ep = Number(msg.error_pos ?? 0);
  const eo = Number(msg.error_ori ?? 0);
  $("ik-r-epos").textContent = fmt(ep, 2);
  $("ik-r-epos").className = "r-value " + (ep < 5 ? "ok" : ep < 20 ? "warn" : "err");
  $("ik-r-eori").textContent = fmt(eo, 2);
  $("ik-r-eori").className = "r-value " + (eo < 2 ? "ok" : eo < 8 ? "warn" : "err");

  const ang = msg.angles_deg || [];
  const ids = ["ik-out-b","ik-out-s","ik-out-g","ik-out-y","ik-out-p","ik-out-r"];
  ang.forEach((v,i) => { if(ids[i]) $(ids[i]).value = fmt(v, 1); });
}

function executeSetpose() {
  // Usa angoli virtuali ottenuti da IK (output cells); altrimenti li ricalcola.
  const ids = ["ik-out-b","ik-out-s","ik-out-g","ik-out-y","ik-out-p","ik-out-r"];
  const angles = ids.map(id => parseFloat($(id).value));
  if (angles.some(v => isNaN(v))) {
    showFeedback("fb-ik", false, "Calcola prima IK");
    return;
  }
  const vel = parseInt($("vel-slider").value, 10) || 40;
  const prof = STATE.profile;   // BB ora supportato nativamente dal firmware
  const cmd = `SETPOSE ${angles.map(a => Math.round(a)).join(" ")} ${vel} ${prof}`;
  sendCommand("uart", { cmd });
  addLog(`[PP] SETPOSE → ${angles.map(a=>a.toFixed(0)).join(",")} vel=${vel}°/s prof=${prof}`);

  // Stima durata movimento dal max delta giunto / vel
  const curAng = STATE.jointAngles || [90,90,90,90,90,90];
  let maxDelta = 0;
  for (let i = 0; i < 6; i++) {
    const d = Math.abs(angles[i] - curAng[i]);
    if (d > maxDelta) maxDelta = d;
  }
  const durationMs = Math.max(200, Math.round(maxDelta / vel * 1000));
  registerMotionPrediction({
    profile: prof,
    durationMs,
    targetXYZ: { x: parseFloat($("ik-in-x").value), y: parseFloat($("ik-in-y").value), z: parseFloat($("ik-in-z").value) },
  });

  showFeedback("fb-ik", true, "Inviato");
}

// ============================================================
// Esecuzione live: buffer scorrevole + derivata vel/acc TCP
// ============================================================
const LIVE = {
  buffer: [],          // [{ t_ms, x, y, z, v, a }]
  events: [],          // [{ t_ms, label }]  marker verticali
  paused: false,
  windowMs: 10000,     // 10 s di storia visibile
  emaAlpha: 0.4,       // smoothing low-pass
  lastSample: null,    // {t,x,y,z}
  lastV: 0,
  lastT: null,
  rateHz: 0,
  _rateAcc: 0, _rateCount: 0, _rateLastReport: 0,
  vPeak: 0, aPeakAbs: 0,

  // Predizione del movimento atteso, registrata a ogni invio SETPOSE/SETPOSE_T.
  // Usata per disegnare il profilo teorico scalato sovrapposto alla traccia reale.
  // { startMs, endMs, profile, distMm, vPeakMmS, aPeakMmS2 }
  prediction: null,
  // Quanto tempo dopo endMs continuiamo a mostrare il profilo teorico nel chart
  // (utile per confronto visivo a movimento concluso).
  predictionRetainMs: 4000,
};

/** Registra una previsione di movimento dato il TCP target e durata.
 * Distanza cartesiana = |TCP_target - TCP_corrente| (richiede fk_live valido).
 * Se distMm passato esplicito, lo usa. */
function registerMotionPrediction({ profile, durationMs, targetXYZ, distMmOverride }) {
  if (!durationMs || durationMs <= 0) return;
  let dist = 0;
  if (distMmOverride != null) {
    dist = distMmOverride;
  } else if (STATE.fkLive.x != null && targetXYZ) {
    const dx = targetXYZ.x - STATE.fkLive.x;
    const dy = targetXYZ.y - STATE.fkLive.y;
    const dz = targetXYZ.z - STATE.fkLive.z;
    dist = Math.hypot(dx, dy, dz);
  }
  if (dist < 1) return;   // movimento troppo piccolo
  const T_s = durationMs / 1000.0;
  const def = PROFILE_DEFS[profile] || PROFILE_DEFS.RTR5;
  // Picco normalizzato di sd e |sdd|
  let vN = 0, aN = 0;
  for (let i = 0; i <= 200; i++) {
    const tt = i / 200;
    vN = Math.max(vN, Math.abs(def.sd(tt)));
    aN = Math.max(aN, Math.abs(def.sdd(tt)));
  }
  // Conversione: τ = t/T → ds/dτ = ds/dt × T → v_real = sd_norm × dist / T
  const vPeak = vN * dist / T_s;
  const aPeak = aN * dist / (T_s * T_s);
  const startMs = performance.now();
  LIVE.prediction = {
    startMs,
    endMs: startMs + durationMs,
    profile,
    distMm: dist,
    vPeakMmS: vPeak,
    aPeakMmS2: aPeak,
  };
  addLog(`[PP-PRED] ${profile} dist=${dist.toFixed(0)}mm T=${durationMs}ms v_max=${vPeak.toFixed(0)}mm/s a_max=${aPeak.toFixed(0)}mm/s²`);
}

function _liveResetPeaks() {
  LIVE.vPeak = 0;
  LIVE.aPeakAbs = 0;
}

function pushLiveSample(t_ms, x, y, z) {
  if (LIVE.paused) return;
  if (x == null || y == null || z == null) return;
  // Velocità per central-difference: serve campione precedente.
  let v = LIVE.lastV, a = 0;
  if (LIVE.lastSample) {
    const dt = (t_ms - LIVE.lastSample.t_ms) / 1000.0;
    if (dt > 1e-4) {
      const dx = x - LIVE.lastSample.x;
      const dy = y - LIVE.lastSample.y;
      const dz = z - LIVE.lastSample.z;
      const vRaw = Math.hypot(dx, dy, dz) / dt;
      v = LIVE.emaAlpha * vRaw + (1 - LIVE.emaAlpha) * LIVE.lastV;
      a = (v - LIVE.lastV) / dt;
      // Stima rate per UI
      LIVE._rateAcc += 1.0 / dt;
      LIVE._rateCount += 1;
      const now = performance.now();
      if (now - LIVE._rateLastReport > 750) {
        LIVE.rateHz = LIVE._rateAcc / Math.max(1, LIVE._rateCount);
        LIVE._rateAcc = 0; LIVE._rateCount = 0; LIVE._rateLastReport = now;
        const el = $("live-rate-text");
        if (el) el.textContent = `${LIVE.rateHz.toFixed(1)} Hz · α=${LIVE.emaAlpha.toFixed(2)}`;
      }
    }
  }
  LIVE.buffer.push({ t_ms, x, y, z, v, a });
  LIVE.lastSample = { t_ms, x, y, z };
  LIVE.lastV = v;
  LIVE.lastT = t_ms;

  // Aggiorna picchi (solo dentro la finestra mobile)
  const cutoff = t_ms - LIVE.windowMs;
  while (LIVE.buffer.length && LIVE.buffer[0].t_ms < cutoff) LIVE.buffer.shift();
  while (LIVE.events.length && LIVE.events[0].t_ms < cutoff) LIVE.events.shift();
  // Ricomputa picchi sulla finestra (cheap perché ~300 campioni)
  let vp = 0, ap = 0;
  for (const s of LIVE.buffer) {
    if (s.v > vp) vp = s.v;
    const aa = Math.abs(s.a);
    if (aa > ap) ap = aa;
  }
  LIVE.vPeak = vp; LIVE.aPeakAbs = ap;

  // UI istantanee
  $("live-v-now").textContent  = v.toFixed(1) + " mm/s";
  $("live-v-peak").textContent = vp.toFixed(0) + " mm/s";
  $("live-a-now").textContent  = a.toFixed(0) + " mm/s²";
  $("live-a-peak").textContent = ap.toFixed(0) + " mm/s²";
  $("live-n-samples").textContent = LIVE.buffer.length;
}

function pushLiveEvent(label) {
  if (!LIVE.lastT) return;
  LIVE.events.push({ t_ms: LIVE.lastT, label: String(label || "") });
}

function drawLiveChart() {
  const canvas = $("live-chart");
  if (!canvas) return;
  const wrap = $("live-chart-wrap");
  const dpr = window.devicePixelRatio || 1;
  const W = wrap.clientWidth - 16, H = wrap.clientHeight - 16;
  canvas.width = W * dpr; canvas.height = H * dpr;
  canvas.style.width = W + "px"; canvas.style.height = H + "px";
  const ctx = canvas.getContext("2d");
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, W, H);

  const padL = 50, padR = 14, padT = 16, padB = 26;
  const innerW = W - padL - padR;
  const innerH = H - padT - padB;
  const yMidV = padT + innerH * 0.45;          // zona velocità (alto)
  const yMidA = padT + innerH * 0.85;          // zona accelerazione (basso, con zero al centro)
  const halfH_v = innerH * 0.42;
  const halfH_a = innerH * 0.18;

  // Sfondo griglia
  ctx.strokeStyle = "rgba(61,157,255,0.12)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 10; i++) {
    const x = padL + i * innerW / 10;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + innerH); ctx.stroke();
  }
  // Linea zero accelerazione
  ctx.strokeStyle = "rgba(61,157,255,0.35)";
  ctx.beginPath(); ctx.moveTo(padL, yMidA); ctx.lineTo(padL + innerW, yMidA); ctx.stroke();

  // Etichette
  ctx.fillStyle = "#9db1cc";
  ctx.font = "11px system-ui, sans-serif";
  ctx.fillText("v(t) mm/s", padL + 4, padT + 12);
  ctx.fillText("a(t) mm/s²", padL + 4, yMidA - halfH_a + 12);
  ctx.fillText("−10 s",   padL,             H - 7);
  ctx.fillText("0 (now)", padL + innerW - 42, H - 7);

  if (LIVE.buffer.length < 2 && !LIVE.prediction) return;

  // Mappa tempo → x. Finestra mobile [now-windowMs, now].
  // Se non ci sono ancora campioni reali, usa performance.now() come "now".
  const tEnd = LIVE.lastT || performance.now();
  const tStart = tEnd - LIVE.windowMs;
  const xOf = t => padL + ((t - tStart) / LIVE.windowMs) * innerW;

  // Auto-scale Y considerando ANCHE i picchi teorici della prediction.
  let vScale = LIVE.vPeak, aScale = LIVE.aPeakAbs;
  const pred = LIVE.prediction;
  // Mantieni in vita la prediction per qualche secondo dopo endMs
  const predAlive = pred && (tEnd <= pred.endMs + LIVE.predictionRetainMs);
  if (predAlive) {
    vScale = Math.max(vScale, pred.vPeakMmS);
    aScale = Math.max(aScale, pred.aPeakMmS2);
  } else if (pred && tEnd > pred.endMs + LIVE.predictionRetainMs) {
    LIVE.prediction = null;
  }
  const vMax = Math.max(50, vScale * 1.15);
  const aMax = Math.max(100, aScale * 1.15);
  const yV = v => yMidV - (v / vMax) * halfH_v;
  const yA = a => yMidA - (a / aMax) * halfH_a;

  // ─── Banda esecuzione attesa (tra startMs e endMs) ─────────────────────
  if (predAlive) {
    const x0 = Math.max(padL, xOf(pred.startMs));
    const x1 = Math.min(padL + innerW, xOf(pred.endMs));
    if (x1 > x0) {
      ctx.fillStyle = "rgba(255,255,255,0.04)";
      ctx.fillRect(x0, padT, x1 - x0, innerH);
    }
  }

  // Marker eventi (linee tratteggiate verticali, dietro alle curve)
  ctx.strokeStyle = "rgba(176,127,217,0.55)";
  ctx.setLineDash([4, 4]);
  ctx.lineWidth = 1;
  for (const e of LIVE.events) {
    const x = xOf(e.t_ms);
    if (x < padL || x > padL + innerW) continue;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + innerH); ctx.stroke();
  }
  ctx.setLineDash([]);

  // ─── Profilo teorico SCALATO (tratteggio bianco) ──────────────────────
  if (predAlive) {
    const def = PROFILE_DEFS[pred.profile] || PROFILE_DEFS.RTR5;
    const T_s = (pred.endMs - pred.startMs) / 1000.0;
    const distMm = pred.distMm;
    const N = 200;
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 1.7;

    // velocità teorica
    ctx.strokeStyle = "rgba(93,255,168,0.85)";
    ctx.beginPath();
    let started = false;
    for (let i = 0; i <= N; i++) {
      const tau = i / N;
      const tMs = pred.startMs + tau * (pred.endMs - pred.startMs);
      const vTh = def.sd(tau) * distMm / T_s;
      const x = xOf(tMs);
      if (x < padL - 1 || x > padL + innerW + 1) continue;
      const y = yV(vTh);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else          { ctx.lineTo(x, y); }
    }
    ctx.stroke();

    // accelerazione teorica
    ctx.strokeStyle = "rgba(255,184,77,0.85)";
    ctx.beginPath();
    started = false;
    for (let i = 0; i <= N; i++) {
      const tau = i / N;
      const tMs = pred.startMs + tau * (pred.endMs - pred.startMs);
      const aTh = def.sdd(tau) * distMm / (T_s * T_s);
      const x = xOf(tMs);
      if (x < padL - 1 || x > padL + innerW + 1) continue;
      const y = yA(aTh);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else          { ctx.lineTo(x, y); }
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // ─── Curve REALI misurate (linea continua spessa) ─────────────────────
  if (LIVE.buffer.length >= 2) {
    // velocità reale (verde)
    ctx.strokeStyle = "#5dffa8";
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    let started = false;
    for (const s of LIVE.buffer) {
      const x = xOf(s.t_ms), y = yV(s.v);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else          { ctx.lineTo(x, y); }
    }
    ctx.stroke();

    // accelerazione reale (arancione)
    ctx.strokeStyle = "#ffb84d";
    ctx.lineWidth = 1.7;
    ctx.beginPath();
    started = false;
    for (const s of LIVE.buffer) {
      const x = xOf(s.t_ms), y = yA(s.a);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else          { ctx.lineTo(x, y); }
    }
    ctx.stroke();
  }

  // Etichette scala Y
  ctx.fillStyle = "#5dffa8";
  ctx.fillText(`${vMax.toFixed(0)}`, 6, yMidV - halfH_v + 12);
  ctx.fillText("0", 6, yMidV + 4);
  ctx.fillStyle = "#ffb84d";
  ctx.fillText(`+${aMax.toFixed(0)}`, 6, yMidA - halfH_a + 10);
  ctx.fillText(`−${aMax.toFixed(0)}`, 6, yMidA + halfH_a + 4);

  // Cursore "now" (linea verticale a destra)
  ctx.strokeStyle = "rgba(93,255,168,0.4)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL + innerW, padT);
  ctx.lineTo(padL + innerW, padT + innerH);
  ctx.stroke();
}

let _liveRafScheduled = false;
function scheduleLiveRedraw() {
  if (_liveRafScheduled) return;
  _liveRafScheduled = true;
  requestAnimationFrame(() => {
    _liveRafScheduled = false;
    drawLiveChart();
  });
}

function exportLiveCSV() {
  if (!LIVE.buffer.length) {
    showFeedback("fb-pickplace", false, "Buffer vuoto");
    return;
  }
  const t0 = LIVE.buffer[0].t_ms;
  const lines = ["t_ms,x_mm,y_mm,z_mm,v_mm_s,a_mm_s2"];
  for (const s of LIVE.buffer) {
    lines.push(`${(s.t_ms - t0).toFixed(1)},${s.x.toFixed(2)},${s.y.toFixed(2)},${s.z.toFixed(2)},${s.v.toFixed(2)},${s.a.toFixed(2)}`);
  }
  // Eventi come header commentato
  if (LIVE.events.length) {
    lines.unshift(`# events (t_ms_rel,label):`);
    LIVE.events.forEach(e => lines.unshift(`# ${(e.t_ms - t0).toFixed(1)},${e.label}`));
  }
  const blob = new Blob([lines.join("\n")], { type: "text/csv" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  const ts = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
  a.href = url; a.download = `pickplace_live_${ts}.csv`;
  document.body.appendChild(a); a.click(); a.remove();
  URL.revokeObjectURL(url);
  addLog(`[PP-LIVE] CSV esportato (${LIVE.buffer.length} campioni, ${LIVE.events.length} eventi)`);
}

function setupLiveChart() {
  $("btn-live-pause")?.addEventListener("click", (e) => {
    LIVE.paused = !LIVE.paused;
    e.currentTarget.textContent = LIVE.paused ? "▶ Riprendi" : "⏸ Pausa";
  });
  $("btn-live-clear")?.addEventListener("click", () => {
    LIVE.buffer = []; LIVE.events = [];
    LIVE.lastSample = null; LIVE.lastV = 0; LIVE.lastT = null;
    _liveResetPeaks();
    $("live-v-now").textContent = $("live-v-peak").textContent =
      $("live-a-now").textContent = $("live-a-peak").textContent = "–";
    $("live-n-samples").textContent = "0";
    drawLiveChart();
  });
  $("btn-live-csv")?.addEventListener("click", exportLiveCSV);

  // Loop di disegno @20Hz indipendente dalla telemetria.
  setInterval(drawLiveChart, 50);
  window.addEventListener("resize", drawLiveChart);
}

// ============================================================
// Profili velocità / accelerazione (canvas 2D)
// ============================================================
const PROFILE_DEFS = {
  RTR3: { color: "#5db8ff", visible: true,
    s: t => 3*t*t - 2*t*t*t,
    sd:  t => 6*t - 6*t*t,
    sdd: t => 6 - 12*t },
  RTR5: { color: "#5dffa8", visible: true,
    s:   t => 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t,
    sd:  t => 30*t*t - 60*t*t*t + 30*t*t*t*t,
    sdd: t => 60*t - 180*t*t + 120*t*t*t },
  BCB:  { color: "#b07fd9", visible: true,
    s:   t => (1 - Math.cos(Math.PI*t)) / 2,
    sd:  t => (Math.PI/2) * Math.sin(Math.PI*t),
    sdd: t => (Math.PI*Math.PI/2) * Math.cos(Math.PI*t) },
  BB:   { color: "#ffb84d", visible: true,
    // Bang-Bang: τ in [0, 0.5] accelerazione costante = +4, da 0.5 a 1 = -4
    // → v(τ) triangolare picco a 0.5 = 2; s(τ) = 2τ² (per τ≤0.5), 1 - 2(1-τ)² (per τ>0.5)
    s:   t => t <= 0.5 ? 2*t*t : 1 - 2*(1-t)*(1-t),
    sd:  t => t <= 0.5 ? 4*t : 4*(1-t),
    sdd: t => t <= 0.5 ? 4 : -4 },
};

// Le funzioni drawProfileChart / buildProfileToggles sono state rimosse:
// l'overlay del profilo teorico è ora disegnato direttamente in drawLiveChart()
// usando LIVE.prediction (registrato a ogni invio di SETPOSE/SETPOSE_T).

// ============================================================
// Pannello attuatori PP1/PP2
// ============================================================
function ppSendDuty(channel, duty) {
  const d = clamp(parseInt(duty, 10) || 0, 0, 100);
  sendCommand("uart", { cmd: `PP${channel} ${d}` });
  addLog(`[PP] PP${channel} duty=${d}%`);
}

function setupActuators() {
  for (const ch of [1, 2]) {
    const slider = $(`pp${ch}-slider`);
    const text = $(`pp${ch}-duty-text`);
    const led = $(`led-pp${ch}`);
    const btnOn = $(`btn-pp${ch}-on`);
    const btnOff = $(`btn-pp${ch}-off`);
    let lastSent = -1;
    let timer = null;

    const updUI = (v) => {
      text.textContent = `${v} %`;
      led.classList.toggle("on", v > 0);
    };
    slider?.addEventListener("input", () => {
      updUI(slider.value);
      clearTimeout(timer);
      timer = setTimeout(() => {
        const v = parseInt(slider.value, 10);
        if (v !== lastSent) { ppSendDuty(ch, v); lastSent = v; }
      }, 110);
    });
    slider?.addEventListener("change", () => {
      const v = parseInt(slider.value, 10);
      if (v !== lastSent) { ppSendDuty(ch, v); lastSent = v; }
    });
    btnOn?.addEventListener("click", () => { slider.value = 100; updUI(100); ppSendDuty(ch, 100); lastSent=100; });
    btnOff?.addEventListener("click", () => { slider.value = 0;   updUI(0);   ppSendDuty(ch, 0);   lastSent=0;   });
  }
  $("btn-pp-stop-all")?.addEventListener("click", () => {
    [1,2].forEach(ch => {
      $(`pp${ch}-slider`).value = 0;
      $(`pp${ch}-duty-text`).textContent = "0 %";
      $(`led-pp${ch}`).classList.remove("on");
      sendCommand("uart", { cmd: `PP${ch} 0` });
    });
    showFeedback("fb-actuators", true, "Entrambi a 0");
  });
  $("btn-pp-status")?.addEventListener("click", () => {
    sendCommand("uart", { cmd: "PP?" });
  });
}

function applyUartResponse(msg) {
  const cmd = String(msg?.cmd || "").trim().toUpperCase();
  // Risposta ENABLE durante recovery → aggiorna indicatore sequenza
  if (cmd === "ENABLE" && STATE._enableRecoveryPending) {
    STATE._enableRecoveryPending = false;
    const ok = msg.ok === true;
    const resp = String(msg.response || "").trim();
    seqStep(ok ? `Ripristinato ✓ (${resp})` : `Ripristino fallito: ${resp}`, ok ? "done" : "fail");
    addLog(`[PP-SEQ] ENABLE recovery → ok=${ok} ${resp}`);
    return;
  }
  if (!cmd.startsWith("PP")) return;
  const ok = msg.ok === true;
  const resp = String(msg.response || "").trim();
  // PP? → "OK PP <d1> <d2>"
  if (cmd === "PP?" && ok) {
    const m = resp.match(/OK PP\s+(\d+)\s+(\d+)/i);
    if (m) {
      const d1 = parseInt(m[1],10), d2 = parseInt(m[2],10);
      STATE.ppDuty[1] = d1; STATE.ppDuty[2] = d2;
      $("pp1-slider").value = d1; $("pp1-duty-text").textContent = `${d1} %`; $("led-pp1").classList.toggle("on", d1>0);
      $("pp2-slider").value = d2; $("pp2-duty-text").textContent = `${d2} %`; $("led-pp2").classList.toggle("on", d2>0);
      $("pill-pp1").textContent = `${d1}%`; $("pill-pp1").className = "diag-value state-pill " + (d1>0?"state-on":"state-off");
      $("pill-pp2").textContent = `${d2}%`; $("pill-pp2").className = "diag-value state-pill " + (d2>0?"state-on":"state-off");
      showFeedback("fb-actuators", true, `← ${resp}`);
      return;
    }
  }
  // PP1/PP2 X → "OK PP1 X"
  const mset = cmd.match(/^PP([12])\s+(\d+)$/);
  if (mset && ok) {
    const ch = parseInt(mset[1],10);
    const d  = parseInt(mset[2],10);
    STATE.ppDuty[ch] = d;
    $(`pill-pp${ch}`).textContent = `${d}%`;
    $(`pill-pp${ch}`).className = "diag-value state-pill " + (d>0?"state-on":"state-off");
  }
  if (!ok) showFeedback("fb-actuators", false, `ERR: ${resp}`);
}

// ============================================================
// SETPOSE_DONE waiter (per sequenza pick&place)
// ============================================================
function waitSetposeDone(timeoutMs = 8000) {
  return new Promise((resolve) => {
    let done = false;
    const t = setTimeout(() => { if (!done) { done = true; resolve({ ok: false, reason: "timeout" }); } }, timeoutMs);
    STATE._setposeDoneResolvers.push((payload) => {
      if (done) return;
      done = true;
      clearTimeout(t);
      resolve({ ok: true, payload });
    });
  });
}

function fireSetposeDone(payload) {
  // Marker verticale nel grafico live (visibile come linea tratteggiata).
  const lbl = payload && payload.time_ms ? `done t=${payload.time_ms}ms` : "done";
  pushLiveEvent(lbl);
  const resolvers = STATE._setposeDoneResolvers.splice(0);
  resolvers.forEach(r => { try { r(payload); } catch(_){} });
}

// ============================================================
// Sequenza Pick → Place automatica
// ============================================================
async function _runIkAtPoint(x, y, z, roll = 0, pitch = 0, yaw = 0) {
  // RPY default = (0, 0, 0): coerente col default UI (#ik-in-pitch value="0").
  // La sequenza passa esplicitamente i valori RPY letti dal pannello UI
  // così la pose validata in "Calcola IK" è la stessa eseguita.
  return new Promise((resolve) => {
    const cmd = `IK_SOLVE ${x} ${y} ${z} ${roll} ${pitch} ${yaw}`;
    let resolved = false;
    const handler = (msg) => {
      if (resolved) return;
      resolved = true;
      _ikResultOnce = null;
      resolve(msg);
    };
    _ikResultOnce = handler;
    sendCommand("uart", { cmd });
    setTimeout(() => { if (!resolved) { resolved = true; _ikResultOnce = null; resolve(null); } }, 2500);
  });
}

let _ikResultOnce = null;   // intercetta UN ik_result per la sequenza

/** Legge RPY dai campi UI del pannello "Target cartesiano" — usato dalla sequenza
 *  per garantire stessa orientazione validata manualmente con "Calcola IK". */
function _readRpyFromUi() {
  return {
    roll:  parseFloat($("ik-in-roll").value)  || 0,
    pitch: parseFloat($("ik-in-pitch").value) || 0,
    yaw:   parseFloat($("ik-in-yaw").value)   || 0,
  };
}

/** Calcola la durata dello step in base al modo selezionato:
 *  - "fixed": usa il campo Time/step
 *  - "velocity": calcola T = max(|target_ang - cur_ang|) / vel_slider × 1000
 */
function _computeStepDurationMs(targetAngles) {
  const mode = (document.querySelector("input[name='seq-dur-mode']:checked") || {}).value || "fixed";
  if (mode === "velocity") {
    const vel = parseInt($("vel-slider").value, 10) || 40;
    const cur = STATE.jointAngles || [90,90,90,90,90,90];
    let maxDelta = 0;
    for (let i = 0; i < 6; i++) {
      const d = Math.abs(targetAngles[i] - cur[i]);
      if (d > maxDelta) maxDelta = d;
    }
    return Math.max(200, Math.round(maxDelta / vel * 1000));
  }
  // default: fixed
  return parseInt($("seq-step-ms").value, 10) || 1500;
}

/** Aggiorna l'HUD nel grafico live con i parametri dello step in corso. */
function _updateLiveHud({ profile, durationMs, distMm }) {
  const hud = $("live-hud");
  if (!hud) return;
  hud.style.display = "";
  $("hud-prof").textContent = profile || "–";
  $("hud-t").textContent    = durationMs != null ? Math.round(durationMs) : "–";
  $("hud-d").textContent    = distMm != null ? distMm.toFixed(0) : "–";
  // Auto-hide dopo 4s di inattività (gestito da timer rinnovato a ogni chiamata)
  clearTimeout(_updateLiveHud._t);
  _updateLiveHud._t = setTimeout(() => { hud.style.display = "none"; }, 4500);
}

async function _moveToCart(x, y, z, timeMsOverride = null, rpy = null) {
  const o = rpy || _readRpyFromUi();
  const r = await _runIkAtPoint(x, y, z, o.roll, o.pitch, o.yaw);
  if (!r || !r.reachable) {
    seqStep(`✗ IK non risolta per (${x},${y},${z}) RPY=(${o.roll},${o.pitch},${o.yaw})`, "fail");
    return false;
  }
  const epos = Number(r.error_pos || 0);
  if (epos > 10) {
    seqStep(`✗ IK errore posizione ${epos.toFixed(1)} mm > 10 mm — abort`, "fail");
    addLog(`[PP-SEQ] IK errore ${epos.toFixed(1)} mm: rifiuto`);
    return false;
  }
  const angRaw = r.angles_deg || [];
  if (angRaw.length !== 6) return false;
  const angInt  = angRaw.map(v => Math.round(v));        // per _computeStepDurationMs
  const angX10  = angRaw.map(v => Math.round(v * 10));   // per comando HR
  // Durata per-step: o override esplicito, o calcolata in base al modo
  const timeMs = timeMsOverride != null ? timeMsOverride : _computeStepDurationMs(angInt);
  // Profilo letto AL MOMENTO dell'invio (così il cambio tab è immediato)
  const prof = STATE.profile;
  // High-resolution: setpoint sub-degree (eliminata la quantizzazione 1°)
  const cmd = `SETPOSE_T_HR ${angX10.join(" ")} ${timeMs} ${prof}`;
  sendCommand("uart", { cmd });
  registerMotionPrediction({
    profile: prof,
    durationMs: timeMs,
    targetXYZ: { x, y, z },
  });
  // HUD: mostra profilo e T effettivi dello step in corso + distanza
  let distMm = null;
  if (STATE.fkLive.x != null) {
    distMm = Math.hypot(x - STATE.fkLive.x, y - STATE.fkLive.y, z - STATE.fkLive.z);
  }
  _updateLiveHud({ profile: prof, durationMs: timeMs, distMm });
  const done = await waitSetposeDone(timeMs + 4000);
  return done.ok;
}

function seqStep(text, dotClass = "active") {
  $("seq-step-text").textContent = text;
  const dot = $("seq-step-dot");
  dot.className = "seq-step-dot " + dotClass;
}

/**
 * runTransfer — esegue una catena pick→place generica fra due punti cartesiani.
 *
 * I parametri vengono RILETTI dai campi UI a OGNI step (non snapshot all'inizio):
 * puoi modificare PICK/PLACE/H/DWELL/profilo/T/vel mentre la sequenza è in corso.
 *
 * @param {object} cfg
 *   @param {string} fromKey,toKey        — "pick" o "place" (riletti per step)
 *   @param {boolean} returnHome          — se true, alla fine va a HOME
 *   @param {string} fromLabel,toLabel    — etichette per log/UI
 *   @param {string} prefix               — prefisso step (es "[3↗]")
 * @returns {Promise<boolean>}
 */
async function runTransfer(cfg) {
  const {
    fromKey = "pick", toKey = "place",
    returnHome,
    fromLabel = "FROM", toLabel = "TO", prefix = "",
  } = cfg;

  const totalSteps = returnHome ? 8 : 7;
  const tag = (i, txt) => `${prefix ? prefix + " " : ""}${i}/${totalSteps} ${txt}`;

  // Helper per rileggere coordinate + RPY del punto FROM/TO ad ogni step
  const _pt = (key) => {
    const p = readSeqParams();
    if (key === "pick")  return { x: p.PX, y: p.PY, z: p.PZ, roll: p.PROLL, pitch: p.PPITCH, yaw: p.PYAW };
    if (key === "place") return { x: p.QX, y: p.QY, z: p.QZ, roll: p.QROLL, pitch: p.QPITCH, yaw: p.QYAW };
    return { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
  };
  const _approachH = () => readSeqParams().H;
  const _dwell     = () => readSeqParams().D;
  // T è gestito per-step da _computeStepDurationMs (in _moveToCart) → null = auto

  const steps = [
    [tag(1, `Approach ${fromLabel}`), () => { const f = _pt(fromKey); return _moveToCart(f.x, f.y, f.z + _approachH(), null, f); }],
    [tag(2, `Discesa ${fromLabel}`),  () => { const f = _pt(fromKey); return _moveToCart(f.x, f.y, f.z, null, f); }],
    [tag(3, "Vuoto ON"), async () => {
      sendCommand("uart", { cmd: "PP2 100" });
      await new Promise(r => setTimeout(r, 200));
      sendCommand("uart", { cmd: "PP1 100" });
      await new Promise(r => setTimeout(r, _dwell()));
      return true;
    }],
    [tag(4, `Risalita ${fromLabel}`), () => { const f = _pt(fromKey); return _moveToCart(f.x, f.y, f.z + _approachH(), null, f); }],
    [tag(5, `Approach ${toLabel}`),   () => { const t = _pt(toKey);   return _moveToCart(t.x, t.y, t.z + _approachH(), null, t); }],
    [tag(6, `Discesa ${toLabel}`),    () => { const t = _pt(toKey);   return _moveToCart(t.x, t.y, t.z, null, t); }],
    [tag(7, "Rilascio"), async () => {
      sendCommand("uart", { cmd: "PP1 0" });
      await new Promise(r => setTimeout(r, 150));
      sendCommand("uart", { cmd: "PP2 0" });
      await new Promise(r => setTimeout(r, _dwell()));
      return true;
    }],
  ];
  if (returnHome) {
    steps.push([tag(8, `Risalita ${toLabel} → HOME`), async () => {
      const t = _pt(toKey);
      const ok1 = await _moveToCart(t.x, t.y, t.z + _approachH(), null, t);
      if (!ok1) return false;
      sendCommand("uart", { cmd: "HOME" });
      await waitSetposeDone((parseInt($("seq-step-ms").value, 10) || 1500) + 4000);
      return true;
    }]);
  } else {
    steps.push([tag(8, `Risalita ${toLabel}`), () => { const t = _pt(toKey); return _moveToCart(t.x, t.y, t.z + _approachH(), null, t); }]);
  }

  for (const [label, fn] of steps) {
    if (STATE.seqAbort) { seqStep(`Interrotta a: ${label}`, "fail"); return false; }
    seqStep(label, "active");
    addLog(`[PP-SEQ] ${label}`);
    let ok = false;
    try { ok = await fn(); } catch (e) { addLog(`[PP-SEQ] ${label} EXC ${e}`); ok = false; }
    if (!ok) { seqStep(`✗ Fallito: ${label}`, "fail"); return false; }
  }
  return true;
}

/** Legge i parametri della sequenza dai campi UI */
function readSeqParams() {
  return {
    PX: parseFloat($("seq-pick-x").value),
    PY: parseFloat($("seq-pick-y").value),
    PZ: parseFloat($("seq-pick-z").value),
    PROLL:  parseFloat($("seq-pick-roll").value)  || 0,
    PPITCH: parseFloat($("seq-pick-pitch").value) || 0,
    PYAW:   parseFloat($("seq-pick-yaw").value)   || 0,
    QX: parseFloat($("seq-place-x").value),
    QY: parseFloat($("seq-place-y").value),
    QZ: parseFloat($("seq-place-z").value),
    QROLL:  parseFloat($("seq-place-roll").value)  || 0,
    QPITCH: parseFloat($("seq-place-pitch").value) || 0,
    QYAW:   parseFloat($("seq-place-yaw").value)   || 0,
    H:  parseFloat($("seq-app-h").value),
    T:  parseInt($("seq-step-ms").value, 10),
    D:  parseInt($("seq-dwell-ms").value, 10),
  };
}

function _seqLockUI(locked) {
  ["btn-seq-run", "btn-seq-reverse", "btn-seq-loop", "btn-seq-demo"].forEach(id => {
    const b = $(id); if (b) b.disabled = locked;
  });
}

// ============================================================
// Persistenza parametri sequenza (localStorage)
// ============================================================
const SEQ_LS_KEY = "j5_pickplace_seq_v1";
const SEQ_DEFAULT = {
  PX: 120, PY: 80, PZ: 40,   PROLL: 0, PPITCH: 0, PYAW: 0,
  QX: 120, QY: -80, QZ: 40,  QROLL: 0, QPITCH: 0, QYAW: 0,
  H: 80, T: 1500, D: 700,
  durMode: "fixed",
};

function _writeSeqUi(s) {
  $("seq-pick-x").value = s.PX;
  $("seq-pick-y").value = s.PY;
  $("seq-pick-z").value = s.PZ;
  $("seq-pick-roll").value  = s.PROLL  ?? 0;
  $("seq-pick-pitch").value = s.PPITCH ?? 0;
  $("seq-pick-yaw").value   = s.PYAW   ?? 0;
  $("seq-place-x").value = s.QX;
  $("seq-place-y").value = s.QY;
  $("seq-place-z").value = s.QZ;
  $("seq-place-roll").value  = s.QROLL  ?? 0;
  $("seq-place-pitch").value = s.QPITCH ?? 0;
  $("seq-place-yaw").value   = s.QYAW   ?? 0;
  $("seq-app-h").value   = s.H;
  $("seq-step-ms").value = s.T;
  $("seq-dwell-ms").value = s.D;
  const radios = document.querySelectorAll("input[name='seq-dur-mode']");
  radios.forEach(r => { r.checked = (r.value === (s.durMode || "fixed")); });
}

function _renderSeqSavedInfo(savedAt) {
  const wrap = $("seq-saved-info");
  if (!wrap) return;
  const em = wrap.querySelector("em");
  if (savedAt) {
    em.textContent = `Sequenza persistita (browser) — salvata ${new Date(savedAt).toLocaleString()}`;
    wrap.style.display = "";
  } else {
    em.textContent = "";
    wrap.style.display = "none";
  }
}

function saveSeqToLocal() {
  const p = readSeqParams();
  const durMode = (document.querySelector("input[name='seq-dur-mode']:checked") || {}).value || "fixed";
  const payload = { ...p, durMode, savedAt: Date.now() };
  try {
    localStorage.setItem(SEQ_LS_KEY, JSON.stringify(payload));
    _renderSeqSavedInfo(payload.savedAt);
    seqStep(`Sequenza salvata ✓ (PICK ${p.PX},${p.PY},${p.PZ} · PLACE ${p.QX},${p.QY},${p.QZ})`, "done");
    addLog(`[PP] Sequenza salvata: PICK(${p.PX},${p.PY},${p.PZ}) PLACE(${p.QX},${p.QY},${p.QZ}) H=${p.H} T=${p.T} D=${p.D} mode=${durMode}`);
  } catch (e) {
    seqStep(`Salvataggio fallito: ${e}`, "fail");
  }
}

function loadSeqFromLocal() {
  try {
    const raw = localStorage.getItem(SEQ_LS_KEY);
    if (!raw) return false;
    const s = JSON.parse(raw);
    if (typeof s.PX !== "number" || typeof s.QX !== "number") return false;
    _writeSeqUi(s);
    _renderSeqSavedInfo(s.savedAt);
    addLog(`[PP] Sequenza caricata dal browser: PICK(${s.PX},${s.PY},${s.PZ}) PLACE(${s.QX},${s.QY},${s.QZ}) mode=${s.durMode || 'fixed'}`);
    return true;
  } catch (_) { return false; }
}

function resetSeqToDefault() {
  _writeSeqUi(SEQ_DEFAULT);
  try { localStorage.removeItem(SEQ_LS_KEY); } catch(_){}
  _renderSeqSavedInfo(null);
  seqStep("Sequenza ripristinata ai default", "done");
  addLog("[PP] Sequenza reimpostata ai default");
}

/** PICK → PLACE (forward) */
async function runPickPlaceSequence() {
  if (STATE.seqRunning) return;
  STATE.seqRunning = true; STATE.seqAbort = false;
  _seqLockUI(true);
  const ok = await runTransfer({
    fromKey: "pick", toKey: "place",
    returnHome: true,
    fromLabel: "PICK", toLabel: "PLACE",
  });
  if (ok && !STATE.seqAbort) seqStep("Sequenza PICK → PLACE completata ✓", "done");
  STATE.seqRunning = false;
  _seqLockUI(false);
}

/** PLACE → PICK (reverse) — riprende il pezzo da PLACE e lo rimette in PICK */
async function runPlacePickSequence() {
  if (STATE.seqRunning) return;
  STATE.seqRunning = true; STATE.seqAbort = false;
  _seqLockUI(true);
  const ok = await runTransfer({
    fromKey: "place", toKey: "pick",
    returnHome: true,
    fromLabel: "PLACE", toLabel: "PICK",
  });
  if (ok && !STATE.seqAbort) seqStep("Sequenza PLACE → PICK completata ✓", "done");
  STATE.seqRunning = false;
  _seqLockUI(false);
}

/** LOOP: alterna PICK→PLACE e PLACE→PICK fino a STOP.
 *  Tutti i parametri (PICK, PLACE, H, T, D, profilo, vel) sono riletti
 *  per ogni step → puoi modificarli in volo senza fermare il loop. */
async function runLoopSequence() {
  if (STATE.seqRunning) return;
  STATE.seqRunning = true; STATE.seqAbort = false;
  _seqLockUI(true);
  let cycle = 0;
  while (!STATE.seqAbort) {
    cycle++;
    const fwd = await runTransfer({
      fromKey: "pick", toKey: "place", returnHome: false,
      fromLabel: "PICK", toLabel: "PLACE", prefix: `[${cycle}↗]`,
    });
    if (!fwd || STATE.seqAbort) break;
    const rev = await runTransfer({
      fromKey: "place", toKey: "pick", returnHome: false,
      fromLabel: "PLACE", toLabel: "PICK", prefix: `[${cycle}↘]`,
    });
    if (!rev || STATE.seqAbort) break;
  }
  if (STATE.seqAbort) {
    sendCommand("uart", { cmd: "PP1 0" });
    sendCommand("uart", { cmd: "PP2 0" });
    sendCommand("uart", { cmd: "HOME" });
    seqStep(`Loop interrotto dopo ${cycle} cicli — HOME`, "fail");
  } else {
    seqStep(`Loop completato — ${cycle} cicli`, "done");
  }
  STATE.seqRunning = false;
  _seqLockUI(false);
}

/** DEMO TESI: PICK→PLACE poi PLACE→PICK poi HOME (ciclo singolo completo) */
async function runDemoSequence() {
  if (STATE.seqRunning) return;
  STATE.seqRunning = true; STATE.seqAbort = false;
  _seqLockUI(true);
  seqStep("DEMO — fase 1/2: PICK → PLACE …", "active");
  const fwd = await runTransfer({
    fromKey: "pick", toKey: "place",
    returnHome: false,
    fromLabel: "PICK", toLabel: "PLACE",
  });
  if (!fwd || STATE.seqAbort) {
    STATE.seqRunning = false;
    _seqLockUI(false);
    return;
  }
  seqStep("DEMO — fase 2/2: PLACE → PICK → HOME …", "active");
  const rev = await runTransfer({
    fromKey: "place", toKey: "pick",
    returnHome: true,
    fromLabel: "PLACE", toLabel: "PICK",
  });
  if (rev && !STATE.seqAbort) seqStep("DEMO completata ✓  (HOME raggiunta)", "done");
  STATE.seqRunning = false;
  _seqLockUI(false);
}

// ============================================================
// Init
// ============================================================
function initCollapse() {
  document.querySelectorAll(".pp-section-header").forEach(h => {
    h.addEventListener("click", () => {
      const tid = h.dataset.target;
      const body = $(tid);
      if (!body) return;
      const collapsed = h.classList.toggle("collapsed");
      body.style.display = collapsed ? "none" : "";
      // resize 3D dopo che il container cambia dimensione
      setTimeout(onResize, 50);
    });
  });
}

function setupProfiles() {
  // Tab click — aggiorna profilo attivo + label nel pannello live
  document.querySelectorAll(".pp-profile-tab").forEach(b => {
    b.addEventListener("click", () => {
      document.querySelectorAll(".pp-profile-tab").forEach(x => x.classList.remove("active"));
      b.classList.add("active");
      STATE.profile = b.dataset.prof;
      $("bb-warn-banner").classList.toggle("show", STATE.profile === "BB");
      const lbl = $("live-prof-name");
      if (lbl) {
        lbl.textContent = STATE.profile;
        lbl.style.color = (PROFILE_DEFS[STATE.profile] || {}).color || "var(--text)";
      }
      addLog(`[PP] profilo selezionato: ${STATE.profile}`);
    });
  });
  // Inizializza label profilo
  const lbl = $("live-prof-name");
  if (lbl) {
    lbl.textContent = STATE.profile;
    lbl.style.color = (PROFILE_DEFS[STATE.profile] || {}).color || "var(--text)";
  }
}

function setupIkControls() {
  $("btn-ik-solve")?.addEventListener("click", ikSolve);
  $("btn-execute-setpose")?.addEventListener("click", executeSetpose);
  $("btn-stop-motion")?.addEventListener("click", () => sendCommand("uart", { cmd: "STOP" }));
  $("btn-go-home")?.addEventListener("click", () => sendCommand("uart", { cmd: "HOME" }));
  $("btn-pull-from-live")?.addEventListener("click", () => {
    if (STATE.fkLive.x == null) { showFeedback("fb-ik", false, "FK non disponibile"); return; }
    $("ik-in-x").value = Math.round(STATE.fkLive.x);
    $("ik-in-y").value = Math.round(STATE.fkLive.y);
    $("ik-in-z").value = Math.round(STATE.fkLive.z);
    $("ik-in-roll").value  = Math.round(STATE.fkLive.roll);
    $("ik-in-pitch").value = Math.round(STATE.fkLive.pitch);
    $("ik-in-yaw").value   = Math.round(STATE.fkLive.yaw);
    showTargetMarker();
  });
  ["ik-in-x","ik-in-y","ik-in-z","ik-in-roll","ik-in-pitch","ik-in-yaw"].forEach(id => {
    $(id)?.addEventListener("input", showTargetMarker);
  });
  const vs = $("vel-slider");
  vs?.addEventListener("input", () => $("vel-text").textContent = `${vs.value} °/s`);

  // Persistenza target (localStorage)
  $("btn-target-save")?.addEventListener("click", saveTargetToLocal);
  $("btn-target-reset")?.addEventListener("click", resetTargetToDefault);
  // Autoload all'avvio (sovrascrive i value="..." dell'HTML solo se c'è un salvataggio)
  loadTargetFromLocal();
}

function setupSceneViews() {
  document.querySelectorAll(".scene-overlay [data-view]").forEach(b => {
    b.addEventListener("click", () => setView(b.dataset.view));
  });
  $("btn-3d-clear-trail")?.addEventListener("click", () => {
    STATE.trailPoints = [];
    STATE.trailLine.geometry.setDrawRange(0, 0);
  });
}

function setupSequence() {
  $("btn-seq-run")?.addEventListener("click", runPickPlaceSequence);
  $("btn-seq-reverse")?.addEventListener("click", runPlacePickSequence);
  $("btn-seq-loop")?.addEventListener("click", runLoopSequence);
  $("btn-seq-demo")?.addEventListener("click", runDemoSequence);
  $("btn-seq-save")?.addEventListener("click", saveSeqToLocal);
  $("btn-seq-reset")?.addEventListener("click", resetSeqToDefault);
  $("btn-seq-abort")?.addEventListener("click", () => {
    STATE.seqAbort = true;
    sendCommand("uart", { cmd: "STOP" });
    seqStep("STOP inviato — premi ↻ Ripristina per riabilitare", "fail");
  });
  $("btn-seq-recover")?.addEventListener("click", () => {
    if (STATE.seqRunning) {
      seqStep("Sequenza in corso: prima Interrompi", "fail");
      return;
    }
    // Sicurezza: spegni vuoto prima di rimuovere il blocco state machine.
    sendCommand("uart", { cmd: "PP1 0" });
    sendCommand("uart", { cmd: "PP2 0" });
    // Backend Pi: ENABLE da stato STOPPED esegue automaticamente
    // SAFE → ENABLE → AUTO_HOME (vedi ws_handlers_uart.py).
    STATE._enableRecoveryPending = true;
    sendCommand("uart", { cmd: "ENABLE" });
    seqStep("Ripristino in corso (SAFE → ENABLE → HOME)…", "active");
    addLog("[PP-SEQ] Ripristino richiesto");
  });

  // Autoload parametri sequenza dal browser (sovrascrive i value="..." HTML
  // solo se c'è un salvataggio valido).
  loadSeqFromLocal();
}

function init() {
  if (!THREE) {
    addLog("ERR: Three.js non caricato — vista 3D disabilitata");
  } else {
    build3DScene();
    setupSceneViews();
  }

  initCollapse();
  setupProfiles();
  setupIkControls();
  setupActuators();
  setupSequence();
  setupLiveChart();

  // Handlers WS
  registerTelemetryHandler(applyTelemetry);
  registerIkResultHandler((msg) => {
    if (typeof _ikResultOnce === "function") {
      const cb = _ikResultOnce;
      _ikResultOnce = null;
      try { cb(msg); } catch(_) {}
      return;
    }
    applyIkResult(msg);
  });
  registerSetposeDoneHandler(fireSetposeDone);
  registerUartResponseHandler(applyUartResponse);
  registerSettingsHandler(msg => {
    if (msg.type === "settings") {
      STATE.settings = msg;
    }
  });

  // Connessione WS + boot requests
  connectJ5Dashboard();
  setTimeout(() => {
    sendCommand("get_settings", {});
    sendCommand("uart", { cmd: "PP?" });    // legge stato attuatori al boot
  }, 600);

  addLog("Pick & Place Test Bench pronto.");
}

document.addEventListener("DOMContentLoaded", init);
