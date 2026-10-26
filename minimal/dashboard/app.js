// app.js — dashboard minimale di JONNY5: diagnostica, comandi, giunti con cinematica, impostazioni, test.
// Parla solo con j5mini (WebSocket, stessi messaggi dell'app Quest), che inoltra le righe UART allo STM32.
import * as K from "./kin.js";
import { View3D } from "./view3d.js";
import { CamTest } from "./camtest.js";

const $ = id => document.getElementById(id);
const JN = ["Base", "Spalla", "Gomito", "Yaw polso", "Pitch polso", "Roll polso"];
const SK = ["servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R"];
const MOTION_CMDS = /^(HOME|PARK|TELEOPPOSE|SETPOSE)/;
const S = {
  ws: null, connected: false, cfg: null, kin: null, tm: null, col: null, colAt: 0,
  target: [90, 90, 90, 90, 90, 90], pending: [], waiters: [], mine: new Set(), abort: false,
  samples: [], hist: [], tmCount: 0, tmRate: 0, uiQueued: false, poses: [],
};

// ------------------------------------------------------------------ WebSocket
function wsUrl() {
  const q = new URLSearchParams(location.search).get("ws");
  if (q) return q;
  const host = location.hostname || "localhost";
  return `${location.protocol === "https:" ? "wss" : "ws"}://${host}:8557`;
}

function connect() {
  const url = wsUrl();
  let ws;
  try { ws = new WebSocket(url); } catch (e) { return setTimeout(connect, 2000); }
  S.ws = ws;
  ws.onopen = () => { S.connected = true; send({ type: "get_config" }); badges(); log(`collegato a ${url}`, "uns"); };
  ws.onclose = () => {
    if (S.connected) log("collegamento perso, riprovo...", "err");
    S.connected = false; setMotion(false); badges();
    if (location.protocol === "https:" && !S.certHint) {
      S.certHint = true;
      toast(`Se non si collega: apri https://${location.hostname}:8557 e accetta il certificato`, true, 8000);
    }
    setTimeout(connect, 2000);
  };
  ws.onmessage = ev => { try { onMessage(JSON.parse(ev.data)); } catch (e) { console.warn(e); } };
}

function send(o) { if (S.ws && S.ws.readyState === 1) { S.ws.send(JSON.stringify(o)); return true; } return false; }

/** One UART line; resolves with {ok, response} (j5mini broadcasts every reply as uart_response). */
function uart(cmd, timeoutMs = 3000) {
  cmd = cmd.trim();
  S.mine.add(cmd);
  return new Promise(resolve => {
    const p = { cmd, resolve, timer: setTimeout(() => { S.pending.splice(S.pending.indexOf(p), 1); resolve({ ok: false, response: "nessuna risposta" }); }, timeoutMs) };
    S.pending.push(p);
    if (!send({ type: "uart", cmd })) { clearTimeout(p.timer); S.pending.pop(); resolve({ ok: false, response: "non collegato" }); return; }
    log("→ " + cmd, "tx");
  });
}

/** Waits for an unsolicited line matching re (e.g. SETPOSE_DONE), or a telemetry condition. */
function waitLine(re, timeoutMs) {
  return new Promise(resolve => {
    const w = { re, resolve, timer: setTimeout(() => { S.waiters.splice(S.waiters.indexOf(w), 1); resolve(null); }, timeoutMs) };
    S.waiters.push(w);
  });
}
async function waitFor(pred, timeoutMs, stepMs = 100) {
  const end = Date.now() + timeoutMs;
  while (Date.now() < end) { if (S.abort) return false; if (pred()) return true; await sleep(stepMs); }
  return false;
}
const sleep = ms => new Promise(r => setTimeout(r, ms));

function onMessage(m) {
  switch (m.type) {
    case "telemetry":
      S.tm = m; S.tmCount++;
      if (!S.targetInit && S.jointsBuilt && measuredVirtual()) { S.targetInit = true; S.target = measuredVirtual(); buildJoints(); }
      S.samples.push({ t: performance.now(), link: m.link });
      while (S.samples.length > 2 && S.samples[0].t < performance.now() - 1500) S.samples.shift();
      queueUi();
      break;
    case "config":
      S.cfg = m;
      $("b-sim").classList.toggle("hidden", !m.sim);
      buildJoints(); fillSettings(); queueUi();
      break;
    case "uart_response": {
      const i = S.pending.findIndex(p => p.cmd === m.cmd);
      const mine = i >= 0 || S.mine.has(m.cmd);
      if (i >= 0) { const p = S.pending.splice(i, 1)[0]; clearTimeout(p.timer); p.resolve(m); }
      if (mine || $("log-poll").checked || m.cmd !== "STATUS?") log(`← ${m.cmd}: ${m.response}`, m.ok ? "resp" : "err");
      if (m.response && m.response.startsWith("STATUS:") && S.tm) { S.tm.robot_state = m.response.slice(7).trim(); queueUi(); }
      break;
    }
    case "uart_unsolicited":
      log("⇠ " + m.line, "uns");
      for (const w of S.waiters.slice()) if (w.re.test(m.line)) {
        S.waiters.splice(S.waiters.indexOf(w), 1); clearTimeout(w.timer); w.resolve(m.line);
      }
      break;
    case "stereo_collimation": S.col = m; S.colAt = Date.now(); break;
    case "ws_pong": if (S.onPong) S.onPong(m); break;
    case "config_saved":
      $("set-out").textContent = m.ok
        ? `salvato ${m.saved_to_file ? "in j5mini.json (copia precedente in .bak)" : "solo in memoria (modalità simulata)"}:\n` + m.boot_commands.join("\n")
        : "errore: " + m.error;
      toast(m.ok ? "impostazioni salvate" : "errore: " + m.error, !m.ok);
      if (m.ok) send({ type: "get_config" });
      break;
  }
}

// ------------------------------------------------------------------ UI helpers
function toast(msg, bad = false, ms = 3000) {
  const t = $("toast");
  t.textContent = msg; t.className = "show" + (bad ? " bad" : "");
  clearTimeout(t._h); t._h = setTimeout(() => { t.className = ""; }, ms);
}

function log(line, cls = "") {
  const el = $("log");
  if ((cls === "resp" || cls === "err") && !$("log-resp").checked) return;
  if (cls === "uns" && !$("log-unsol").checked) return;
  const d = new Date();
  const span = document.createElement("span");
  span.className = cls;
  span.textContent = `${d.toTimeString().slice(0, 8)}.${String(d.getMilliseconds()).padStart(3, "0")}  ${line}\n`;
  el.appendChild(span);
  while (el.childNodes.length > 600) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}

function dl(el, rows) {
  el.innerHTML = rows.map(([k, v, cls]) => `<dt>${k}</dt><dd class="${cls || ""}">${v ?? "–"}</dd>`).join("");
}
const yes = (v, alarm) => v === true ? [alarm ? "SÌ" : "sì", alarm ? "alarm" : "on"] : v === false ? ["no", "offv"] : ["?", "offv"];
const fx = (v, d = 1) => (typeof v === "number" ? v.toFixed(d) : "–");

function teleopLive() { return !!(S.tm && S.tm.teleop_mode >= 0); }

function setMotion(on) {
  $("motion").checked = on;
  document.body.classList.toggle("motion-on", on);
}

function motionAllowed() {
  if (!S.connected) { toast("non collegato", true); return false; }
  if (!$("motion").checked) { toast("attiva prima 'comandi di movimento' in alto", true); return false; }
  if (teleopLive()) { toast("il visore sta comandando il braccio: comandi di movimento disattivati", true); return false; }
  return true;
}

function queueUi() {
  if (S.uiQueued) return;
  S.uiQueued = true;
  requestAnimationFrame(() => { S.uiQueued = false; updateUi(); });
}

// ------------------------------------------------------------------ angles
function offDir(i) {
  const c = S.cfg;
  return [c ? c.offsets[i] : 90, c ? c.dirs[i] : 1];
}
function measuredVirtual() {
  if (!S.tm || !(SK[0] in S.tm)) return null;
  return SK.map((k, i) => { const [o, d] = offDir(i); return K.physicalToVirtual(S.tm[k], o, d); });
}
/** Virtual range of joint i from its physical limits (and the firmware's 5..175 for SETPOSE). */
function virtualRange(i) {
  const [o, d] = offDir(i);
  const lim = S.cfg ? S.cfg.limits[i] : [0, 180];
  const a = K.physicalToVirtual(Math.max(5, lim[0]), o, d), b = K.physicalToVirtual(Math.min(175, lim[1]), o, d);
  return [Math.min(a, b), Math.max(a, b)];
}
function clampTarget() {
  S.target = S.target.map((v, i) => { const [lo, hi] = virtualRange(i); return Math.max(lo, Math.min(hi, v)); });
}
function setposeCmd(virt, ms, prof) {
  const phys = virt.map((v, i) => { const [o, d] = offDir(i); return K.virtualToPhysical(v, o, d); });
  return `SETPOSE_T_HR ${phys.map(p => Math.round(p * 10)).join(" ")} ${Math.round(ms)} ${prof}`;
}

// ------------------------------------------------------------------ status tab + badges
function badges() {
  const b = (id, text, cls) => { const e = $(id); e.textContent = text; e.className = "badge " + (cls || ""); };
  b("b-link", S.connected ? "collegato" : "non collegato", S.connected ? "ok" : "off");
  const tm = S.tm || {};
  const st = S.connected ? (tm.robot_state || "?") : "?";
  b("b-state", st, { IDLE: "ok", SAFE: "warn", STOPPED: "bad", UNKNOWN: "dim" }[st] || "dim");
  if (tm.estop_bypass) b("b-estop", "E-STOP BYPASS", "warn");
  else if (tm.estop_active === true) b("b-estop", "E-STOP PREMUTO", "bad");
  else if (tm.estop_active === false) b("b-estop", "E-STOP ok", "ok");
  else b("b-estop", "E-STOP ?", "dim");
  const live = S.connected && teleopLive();
  b("b-teleop", live ? `visore: modo ${tm.teleop_mode}` : "visore: no", live ? "warn" : "dim");
  document.body.classList.toggle("teleop-live", live);
}

function updateUi() {
  badges();
  const tm = S.tm;
  if (!tm) return;
  const now = performance.now();
  let rate = {};
  if (S.samples.length >= 2) {
    const a = S.samples[0], z = S.samples[S.samples.length - 1], dt = (z.t - a.t) / 1000;
    if (dt > 0.3) for (const k of Object.keys(z.link)) rate[k] = (z.link[k] - a.link[k]) / dt;
  }
  dl($("st-robot"), [
    ["stato", tm.robot_state, tm.robot_state === "IDLE" ? "on" : tm.robot_state === "STOPPED" ? "alarm" : ""],
    ["E-STOP premuto", ...yes(tm.estop_active, true)], ["bypass E-STOP", ...yes(tm.estop_bypass, true)],
    ["deadman (2 grip)", ...yes(tm.deadman_active)], ["armato VR", ...yes(tm.vr_armed)],
    ["input attivo", ...yes(tm.input_active)], ["freeze", ...yes(tm.vr_freeze)], ["guard (latch)", ...yes(tm.vr_guard)],
    ["periodo RT loop", tm.rt_loop_period_us ? tm.rt_loop_period_us + " µs" : "–"],
  ]);
  const tot = tm.link || {};
  dl($("st-link"), [
    ["trasferimenti SPI", `${fx(rate.spi_tx, 0)}/s`], ["risposte valide", `${fx(rate.spi_rx_ok, 0)}/s`,
      rate.spi_rx_ok < 80 ? "alarm" : "on"],
    ["frame non validi", `${tot.spi_rx_bad ?? "–"} (${fx(rate.spi_rx_bad, 1)}/s)`, rate.spi_rx_bad > 1 ? "alarm" : ""],
    ["payload neutri", `${fx(rate.neutral && rate.spi_tx ? 100 * rate.neutral / rate.spi_tx : 0, 0)} %`],
    ["UART tx / rx", `${tot.uart_tx ?? "–"} / ${tot.uart_rx ?? "–"}`],
    ["timeout UART", tot.uart_timeouts ?? "–", tot.uart_timeouts ? "alarm" : ""],
    ["telemetria", `${S.tmRate} msg/s`], ["client collegati", tm.clients ?? "–"],
  ]);
  const modes = { 0: "PREP", 1: "POSE", 2: "MANUALE", 3: "HEAD", 4: "HYBRID", 5: "ASSIST" };
  dl($("st-teleop"), [
    ["visore", tm.teleop_mode >= 0 ? `attivo, modo ${tm.teleop_mode} ${modes[tm.teleop_mode] || ""}` : "nessuno"],
    ["età ultimo intent", tm.intent_age_ms != null ? tm.intent_age_ms + " ms" : "mai"],
    ["zoom richiesto", tm.vr_zoom0 != null ? fx(tm.vr_zoom0, 2) + "×" : "–"],
  ]);
  let rpy = null;
  if (typeof tm.imu_q_w === "number") {
    const [w, x, y, z] = [tm.imu_q_w, tm.imu_q_x, tm.imu_q_y, tm.imu_q_z];
    rpy = [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
      Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))].map(a => a / K.DEG);
  }
  dl($("st-imu"), [
    ["valida", ...yes(tm.imu_valid)],
    ["quaternione", typeof tm.imu_q_w === "number" ? [tm.imu_q_w, tm.imu_q_x, tm.imu_q_y, tm.imu_q_z].map(v => v.toFixed(3)).join(" ") : "–"],
    ["roll / pitch / yaw", rpy ? rpy.map(a => a.toFixed(1) + "°").join("  ") : "–"],
  ]);
  const colFresh = S.col && Date.now() - S.colAt < 2000;
  dl($("st-cam"), [
    ["modo sensore", tm.cam_mode ? `${tm.cam_mode} (${fx(tm.cam_hfov, 1)}°, ${tm.cam_fps ?? "–"} fps)` : "nessun dato"],
    ["zoom applicato", tm.cam_zoom != null ? fx(tm.cam_zoom, 2) + "×" : "–"], ["luminosità", tm.cam_ev != null ? fx(tm.cam_ev, 2) + " EV" : "–"],
    ["collimatore", colFresh ? (S.col.lock ? "agganciato" : "cerca...") : "nessun dato", colFresh && S.col.lock ? "on" : ""],
    ["distanza al mirino", colFresh && S.col.dist_mm ? (S.col.dist_mm / 10).toFixed(1) + " cm" : "–"],
  ]);
  const mv = measuredVirtual();
  dl($("st-joints"), SK.map((k, i) => [JN[i], `${tm[k] ?? "–"}°  (virtuale ${mv ? mv[i].toFixed(0) : "–"}°)`]));
  updateJointsMeas(mv);
  draw3d(mv);
}

function drawChart() {
  const c = $("chart"), g = c.getContext("2d"), dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = c.clientHeight;
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const H = S.hist, max = 120;
  g.strokeStyle = "#232a36"; g.lineWidth = 1; g.fillStyle = "#5c667a"; g.font = "11px system-ui";
  for (const v of [0, 50, 100]) { const y = h - 6 - (v / max) * (h - 12); g.beginPath(); g.moveTo(30, y); g.lineTo(w, y); g.stroke(); g.fillText(v, 4, y + 4); }
  const series = [["ok", "#2fa863", 1], ["tm", "#2f7fd8", 5], ["bad", "#d9443a", 10]];
  for (const [k, col, mul] of series) {
    g.strokeStyle = col; g.lineWidth = 2; g.beginPath();
    H.forEach((s, i) => {
      const x = 30 + (w - 30) * (i + 60 - H.length) / 59, y = h - 6 - Math.min(max, s[k] * mul) / max * (h - 12);
      i ? g.lineTo(x, y) : g.moveTo(x, y);
    });
    g.stroke();
  }
}

setInterval(() => {   // once a second: telemetry rate, history for the chart
  S.tmRate = S.tmCount; S.tmCount = 0;
  const tm = S.tm;
  if (tm && S.prevLink) {
    S.hist.push({ ok: tm.link.spi_rx_ok - S.prevLink.spi_rx_ok, bad: tm.link.spi_rx_bad - S.prevLink.spi_rx_bad, tm: S.tmRate });
    if (S.hist.length > 60) S.hist.shift();
  }
  S.prevLink = tm ? { ...tm.link } : null;
  if ($("stato").classList.contains("active")) drawChart();
  queueUi();
}, 1000);

// ------------------------------------------------------------------ arm tab: joints, poses, 3D
const view = new View3D($("view3d"));
fetch("kinematics.json").then(r => r.json()).then(k => { S.kin = k; queueUi(); draw3d(measuredVirtual()); })
  .catch(() => toast("kinematics.json non trovato: vista 3D disattivata", true));

function buildJoints() {
  const box = $("joints");
  box.innerHTML = "";
  const firstLoad = !S.jointsBuilt;
  S.jointsBuilt = true;
  JN.forEach((name, i) => {
    const [lo, hi] = virtualRange(i);
    const row = document.createElement("div");
    row.className = "joint";
    row.innerHTML = `<span class="name">${name}</span>
      <input type="range" min="${lo.toFixed(1)}" max="${hi.toFixed(1)}" step="0.5" id="j-r${i}">
      <input type="number" min="${lo.toFixed(1)}" max="${hi.toFixed(1)}" step="0.5" id="j-n${i}">
      <span class="meas" id="j-m${i}">–</span>`;
    box.appendChild(row);
    const r = row.querySelector("input[type=range]"), n = row.querySelector("input[type=number]");
    const set = v => { S.target[i] = Number(v); clampTarget(); r.value = n.value = S.target[i]; targetChanged(); };
    r.addEventListener("input", () => set(r.value));
    n.addEventListener("change", () => set(n.value));
  });
  if (firstLoad) { const mv = measuredVirtual(); if (mv) { S.target = mv.slice(); S.targetInit = true; } }
  clampTarget();
  S.target.forEach((v, i) => { $("j-r" + i).value = $("j-n" + i).value = v; });
  $("t-sweep-j").innerHTML = JN.map((n, i) => `<option value="${i}">${n}</option>`).join("");
  targetChanged();
}

function updateJointsMeas(mv) {
  if (!mv || !S.jointsBuilt) return;
  mv.forEach((v, i) => {
    const phys = S.tm[SK[i]];
    $("j-m" + i).textContent = `mis. ${v.toFixed(0)}°\nfisico ${phys}°`;
  });
}

let liveTimer = null;
function targetChanged() {
  $("go-cmd").textContent = setposeCmd(S.target, Number($("move-ms").value), $("move-prof").value);
  draw3d(measuredVirtual());
  if ($("live").checked && $("motion").checked && !teleopLive()) {
    clearTimeout(liveTimer);
    liveTimer = setTimeout(() => {
      const mv = measuredVirtual() || S.target;
      const delta = Math.max(...S.target.map((v, i) => Math.abs(v - mv[i])));
      uart(setposeCmd(S.target, Math.max(250, delta * 1000 / 45), "RTR5"));   // ~45 deg/s
    }, 150);
  }
}

function setPoseRow(prefix, p) {
  $(prefix + "x").textContent = p ? p.x_mm.toFixed(1) : "–"; $(prefix + "y").textContent = p ? p.y_mm.toFixed(1) : "–";
  $(prefix + "z").textContent = p ? p.z_mm.toFixed(1) : "–"; $(prefix + "r").textContent = p ? p.roll_deg.toFixed(1) : "–";
  $(prefix + "p").textContent = p ? p.pitch_deg.toFixed(1) : "–"; $(prefix + "w").textContent = p ? p.yaw_deg.toFixed(1) : "–";
}

function draw3d(mv) {
  if (!S.kin) return;
  const { S: Sc, M } = S.kin;
  const scenes = [];
  const dp = S.kin.draw_points;
  if (mv) { scenes.push({ chain: K.chain(Sc, M, K.qFromVirtual(mv), dp), style: "solid" }); setPoseRow("m-", K.poseFromVirtualDeg(Sc, M, mv)); }
  else setPoseRow("m-", null);
  scenes.push({ chain: K.chain(Sc, M, K.qFromVirtual(S.target), dp), style: "ghost" });
  setPoseRow("t-", K.poseFromVirtualDeg(Sc, M, S.target));
  const m = K.manipulability(Sc, K.qFromVirtual(S.target));
  const ref = K.manipulability(Sc, K.qFromVirtual([90, 120, 60, 90, 120, 90]));   // a well-conditioned pose
  $("manip").value = Math.min(1, m / ref);
  $("manip-v").textContent = m < ref * 0.08 ? "vicino a una singolarità" : m.toExponential(2);
  view.set(scenes);
}

async function goTarget(ms = Number($("move-ms").value), prof = $("move-prof").value) {
  if (!motionAllowed()) return null;
  if (S.tm && S.tm.robot_state !== "IDLE") { toast(`robot in ${S.tm.robot_state}: serve IDLE (ENABLE)`, true); return null; }
  const done = waitLine(/^SETPOSE_DONE/, ms + 4000);
  const r = await uart(setposeCmd(S.target, ms, prof));
  if (!r.ok) { toast("rifiutato: " + r.response, true); return null; }
  return done;
}

$("go").onclick = () => goTarget();
$("tgt-meas").onclick = () => { const mv = measuredVirtual(); if (mv) { S.target = mv.slice(); buildJoints(); } };
$("tgt-home").onclick = () => { S.target = [90, 90, 90, 90, 90, 90]; buildJoints(); };
$("move-ms").onchange = $("move-prof").onchange = targetChanged;

function loadPoses() { try { S.poses = JSON.parse(localStorage.getItem("j5.poses") || "[]"); } catch { S.poses = []; } }
function savePoses() { try { localStorage.setItem("j5.poses", JSON.stringify(S.poses)); } catch { /* private mode */ } renderPoses(); }
function renderPoses() {
  $("poses").innerHTML = "";
  S.poses.forEach((p, i) => {
    const li = document.createElement("li");
    li.textContent = `${p.name}  [${p.v.map(v => v.toFixed(0)).join(" ")}]  ${p.ms} ms ${p.prof}`;
    const b = (t, f, cls) => { const e = document.createElement("button"); e.textContent = t; e.onclick = f; if (cls) e.className = cls; li.appendChild(e); };
    b("carica", () => { S.target = p.v.slice(); buildJoints(); });
    b("vai", () => { S.target = p.v.slice(); buildJoints(); goTarget(p.ms, p.prof); }, "motion");
    b("↑", () => { if (i) { [S.poses[i - 1], S.poses[i]] = [S.poses[i], S.poses[i - 1]]; savePoses(); } });
    b("✕", () => { S.poses.splice(i, 1); savePoses(); });
    $("poses").appendChild(li);
  });
}
$("pose-save").onclick = () => {
  S.poses.push({ name: $("pose-name").value || `posa ${S.poses.length + 1}`, v: S.target.slice(),
    ms: Number($("move-ms").value), prof: $("move-prof").value });
  $("pose-name").value = ""; savePoses();
};
$("pose-play").onclick = async () => {
  if (!S.poses.length) return toast("nessuna posa salvata", true);
  S.abort = false;
  for (const p of S.poses) {
    if (S.abort) return;
    S.target = p.v.slice(); buildJoints();
    const done = await goTarget(p.ms, p.prof);
    if (!done || !(await done)) { toast(`sequenza interrotta a "${p.name}"`, true); return; }
  }
  toast("sequenza completata");
};
$("pose-export").onclick = () => {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([JSON.stringify(S.poses, null, 1)], { type: "application/json" }));
  a.download = "jonny5-pose.json"; a.click();
};
$("pose-import").onchange = async e => {
  try {
    const list = JSON.parse(await e.target.files[0].text());
    if (!Array.isArray(list) || !list.every(p => Array.isArray(p.v) && p.v.length === 6)) throw new Error("formato");
    S.poses = list; savePoses(); toast(`${list.length} pose importate`);
  } catch (err) { toast("file non valido", true); }
};

// ------------------------------------------------------------------ commands tab
const CMDS = [
  ["ENABLE", "SAFE/STOPPED → IDLE (non muove)"], ["SAFE", "servo spenti, si riarma da solo con un flusso VR"],
  ["STOP", "STOPPED: serve ENABLE"], ["RESET", "azzera lo stato del firmware"],
  ["HOME", "va agli offset (90 virtuali)", true], ["PARK", "posa di parcheggio", true],
  ["TELEOPPOSE", "posa di partenza della teleoperazione", true], ["RELAX_DIGITAL", "rilascia i servo digitali del polso"],
  ["HEADZERO", "azzera il riferimento della testa"],
  ["STATUS?", "stato del firmware"], ["ESTOP?", "stato della linea di emergenza"], ["VR?", "parametri VR attivi"],
];
for (const [c, tip, motion] of CMDS) {
  const b = document.createElement("button");
  b.textContent = c; b.title = tip;
  if (motion) b.className = "motion";
  b.onclick = () => { if (motion && !motionAllowed()) return; uart(c); };
  $("cmd-buttons").appendChild(b);
}
const hist = []; let histI = 0;
$("console").onsubmit = e => {
  e.preventDefault();
  const v = $("console-in").value.trim();
  if (!v) return;
  if (MOTION_CMDS.test(v) && !motionAllowed()) return;
  uart(v); hist.push(v); histI = hist.length; $("console-in").value = "";
};
$("console-in").onkeydown = e => {
  if (e.key === "ArrowUp" && histI > 0) { $("console-in").value = hist[--histI]; e.preventDefault(); }
  if (e.key === "ArrowDown") { histI = Math.min(hist.length, histI + 1); $("console-in").value = hist[histI] || ""; e.preventDefault(); }
};
$("log-clear").onclick = () => { $("log").textContent = ""; };

// ------------------------------------------------------------------ settings tab
function fillSettings() {
  const c = S.cfg;
  const rows = JN.map((n, i) => {
    const [lo, hi] = virtualRange(i);
    return `<tr><td>${n}</td>
      <td><input type="number" id="s-off${i}" min="0" max="180" value="${c.offsets[i]}"></td>
      <td><select id="s-dir${i}"><option value="1"${c.dirs[i] === 1 ? " selected" : ""}>+1</option><option value="-1"${c.dirs[i] === -1 ? " selected" : ""}>−1</option></select></td>
      <td><input type="number" id="s-min${i}" min="0" max="180" value="${c.limits[i][0]}"></td>
      <td><input type="number" id="s-max${i}" min="0" max="180" value="${c.limits[i][1]}"></td>
      <td>${lo.toFixed(0)}..${hi.toFixed(0)}</td></tr>`;
  });
  $("set-joints").innerHTML = `<tr><th></th><th>offset (HOME)</th><th>dir</th><th>min fisico</th><th>max fisico</th><th>virtuale</th></tr>` + rows.join("");
  $("set-vr").value = c.vr_params || "";
  $("set-pwm").value = c.pwm_config || "";
}
function readSettings() {
  const num = id => Number($(id).value);
  return {
    offsets: JN.map((_, i) => num("s-off" + i)), dirs: JN.map((_, i) => num("s-dir" + i)),
    limits: JN.map((_, i) => [num("s-min" + i), num("s-max" + i)]),
    vr_params: $("set-vr").value.trim().replace(/\s+/g, " "), pwm_config: $("set-pwm").value.trim().replace(/\s+/g, " "),
  };
}
$("set-reload").onclick = () => send({ type: "get_config" });
$("set-save").onclick = () => send({ type: "save_config", ...readSettings() });
$("set-apply").onclick = async () => {
  if (!motionAllowed()) return;
  const s = readSettings();
  const lines = [`SET_OFFSETS ${s.offsets.join(" ")}`];
  if (s.vr_params) lines.push(s.vr_params);
  s.limits.forEach(([a, b], i) => lines.push(`SET_JOINT_LIMITS ${i} ${a} ${b}`));
  if (s.pwm_config) lines.push(s.pwm_config);
  const out = [];
  for (const l of lines) { const r = await uart(l); out.push(`${r.ok ? "OK " : "ERR"}  ${l}  →  ${r.response}`); }
  $("set-out").textContent = out.join("\n") + "\n\nApplicato solo al firmware acceso: per tenerlo, Salva in j5mini.json.";
};

// ------------------------------------------------------------------ tests tab
$("t-ping").onclick = async () => {
  const out = $("t-ping-out"), rtt = [];
  out.textContent = "...";
  for (let i = 0; i < 50; i++) {
    const t0 = performance.now();
    const ok = await new Promise(res => {
      const timer = setTimeout(() => res(false), 2000);
      S.onPong = m => { if (m.id === i) { clearTimeout(timer); res(true); } };
      send({ type: "ws_ping", id: i, ts_client: t0 });
    });
    if (ok) rtt.push(performance.now() - t0);
    await sleep(20);
  }
  S.onPong = null;
  if (!rtt.length) { out.textContent = "nessuna risposta"; return; }
  rtt.sort((a, b) => a - b);
  const q = p => rtt[Math.min(rtt.length - 1, Math.floor(p * rtt.length))].toFixed(1);
  out.textContent = `risposte ${rtt.length}/50\nmin ${q(0)} ms  mediana ${q(0.5)} ms  p95 ${q(0.95)} ms  max ${q(1)} ms`;
};

$("t-link").onclick = async () => {
  const out = $("t-link-out");
  if (!S.tm) { out.textContent = "nessuna telemetria"; return; }
  const a = { ...S.tm.link }, t0 = performance.now();
  for (let s = 10; s > 0; s--) { out.textContent = `misuro... ${s} s`; await sleep(1000); }
  const z = S.tm.link, dt = (performance.now() - t0) / 1000;
  const d = k => z[k] - a[k];
  const okRate = d("spi_rx_ok") / dt, badPct = d("spi_tx") ? 100 * d("spi_rx_bad") / d("spi_tx") : 0;
  const verdict = okRate >= 90 && badPct < 1 && d("uart_timeouts") === 0 ? "BUONO" : "DA CONTROLLARE";
  out.textContent = `trasferimenti ${d("spi_tx")} in ${dt.toFixed(1)} s (${(d("spi_tx") / dt).toFixed(1)}/s)\n` +
    `risposte valide ${okRate.toFixed(1)}/s, non valide ${d("spi_rx_bad")} (${badPct.toFixed(2)} %)\n` +
    `payload neutri ${d("neutral")}, righe UART ${d("uart_tx")} tx / ${d("uart_rx")} rx, timeout ${d("uart_timeouts")}\n` +
    `esito: ${verdict} (atteso ~100/s, < 1 % non validi, 0 timeout)`;
};

$("t-sweep").onclick = async () => {
  const out = $("t-sweep-out");
  const j = Number($("t-sweep-j").value), A = Number($("t-sweep-a").value), ms = Number($("t-sweep-ms").value);
  if (!motionAllowed()) return;
  S.abort = false;
  const [lo, hi] = virtualRange(j), base = S.target.slice();
  const legs = [Math.min(hi, base[j] + A), Math.max(lo, base[j] - A), base[j]];
  const lines = [`${JN[j]}: ${base[j].toFixed(1)}° → ${legs.map(v => v.toFixed(1)).join(" → ")} (virtuali), ${ms} ms, RTR5`];
  out.textContent = lines.join("\n");
  for (const v of legs) {
    if (S.abort) break;
    S.target[j] = v; buildJoints();
    const t0 = performance.now();
    const done = await goTarget(ms, "RTR5");
    const line = done ? await done : null;
    if (!line) { lines.push("  nessun SETPOSE_DONE: interrotto"); break; }
    const elapsed = performance.now() - t0;
    await sleep(300);                                   // let the last telemetry arrive
    const mv = measuredVirtual();
    const m = Object.fromEntries(line.split(" ").slice(1).map(kv => kv.split("=")));
    lines.push(`  ${v.toFixed(1)}°: ${elapsed.toFixed(0)} ms dal comando (firmware ${m.time_ms} ms), ` +
      `vel max ${m.vel_max} °/s, acc max ${m.acc_max} °/s², misurato ${mv ? mv[j].toFixed(0) : "–"}° ` +
      `(errore ${mv ? (mv[j] - v).toFixed(1) : "–"}°, risoluzione telemetria 1°)`);
    out.textContent = lines.join("\n");
  }
};

$("t-estop").onclick = async () => {
  const out = $("t-estop-out"), lines = [];
  const say = s => { lines.push(s); out.textContent = lines.join("\n"); };
  S.abort = false;
  if (!S.tm) return say("nessuna telemetria");
  say("1. premi il fungo di emergenza (60 s)...");
  if (!(await waitFor(() => S.tm.estop_active === true, 60000))) return say("   non rilevato: controlla cablaggio / bit diag. ESITO: FALLITO");
  await uart("STATUS?");
  await sleep(300);
  say(`   rilevato, stato ${S.tm.robot_state} ${S.tm.robot_state === "STOPPED" ? "✓" : "✗ (atteso STOPPED)"}`);
  const r = await uart("ENABLE");
  say(`2. ENABLE con il fungo premuto → ${r.response} ${!r.ok && /ESTOP/.test(r.response) ? "✓ rifiutato" : "✗ doveva essere rifiutato"}`);
  say("3. rilascia il fungo (60 s)...");
  if (!(await waitFor(() => S.tm.estop_active === false, 60000))) return say("   non rilasciato. ESITO: FALLITO");
  const r2 = await uart("ENABLE");
  say(`4. ENABLE dopo il rilascio → ${r2.response} ${r2.ok ? "✓" : "✗"}`);
  say(lines.every(l => !l.includes("✗")) ? "ESITO: SUPERATO" : "ESITO: CONTROLLARE i punti ✗");
};

// ------------------------------------------------------------------ header, tabs, keyboard
$("motion").onchange = e => setMotion(e.target.checked);
$("stop").onclick = () => { S.abort = true; uart("STOP"); toast("STOP inviato"); };
document.addEventListener("keydown", e => {
  const typing = /INPUT|TEXTAREA|SELECT/.test(document.activeElement.tagName);
  if (e.key === "Escape" || (e.key === " " && !typing)) { e.preventDefault(); $("stop").click(); }
});
document.querySelectorAll("#tabs button").forEach(b => b.onclick = () => {
  document.querySelectorAll("#tabs button, .tab").forEach(x => x.classList.remove("active"));
  b.classList.add("active"); $(b.dataset.tab).classList.add("active");
  if (b.dataset.tab === "braccio") view.draw();
  if (b.dataset.tab === "stato") drawChart();
  if (b.dataset.tab !== "camere" && camTest.pcs.some(Boolean)) camTest.stop();   // the video costs CPU on the Pi
});

const camTest = new CamTest({ send, getTelemetry: () => S.tm, toast,
  getCollimation: () => (S.col && Date.now() - S.colAt < 2000 ? S.col : null) });
$("guide").innerHTML = GUIDE();
loadPoses(); renderPoses(); connect();

function GUIDE() {
  return `
<h2>Come è fatto il sistema</h2>
<p>Il braccio ha 6 giunti rotoidali: <b>base</b>, <b>spalla</b>, <b>gomito</b> e un polso sferico <b>yaw / pitch / roll</b>.
Lo STM32 esegue il controllo in tempo reale a 1 kHz (traiettorie, limiti, PWM dei servo, fungo di emergenza); il Pi
con <code>j5mini</code> gli parla via SPI (frame da 64 byte a 100 Hz) e UART (righe di testo come <code>HOME</code>).
Questa pagina passa da <code>j5mini</code> come l'app del visore.</p>
<h2>Angoli: fisici e virtuali</h2>
<p><b>Fisico</b> = gradi del servo (0..180), quello che il firmware usa. <b>Virtuale</b> =
<code>(fisico − offset) · dir + 90</code>: con tutti i giunti a 90 virtuali il braccio è in HOME, qualunque sia il
montaggio dei servo. Gli slider sono in gradi virtuali; la riga <code>SETPOSE_T_HR</code> sotto mostra i fisici
(in decimi di grado) che arrivano al firmware.</p>
<h2>Stati del firmware</h2>
<ul><li><b>SAFE</b>: servo spenti. Diventa IDLE da solo quando arriva un flusso dal visore, oppure con ENABLE.</li>
<li><b>IDLE</b>: pronto; accetta HOME/PARK/SETPOSE e la teleoperazione.</li>
<li><b>STOPPED</b>: dopo STOP o il fungo; serve ENABLE (a fungo rilasciato).</li>
<li>Se lo SPI tace per 500 ms il firmware va in SAFE da solo (watchdog).</li></ul>
<h2>Cinematica (Product of Exponentials)</h2>
<p><code>T(q) = e<sup>[S₁]q₁</sup> · … · e<sup>[S₆]q₆</sup> · M</code>, con gli assi di vite <code>S</code> e la posa
<code>M</code> in <code>kinematics.json</code> e <code>q = virtuale − 90</code> in radianti. La vista 3D mostra la posa
misurata (piena) e il target (tratteggiato); la barra di manipolabilità è <code>|det J|</code> dello Jacobiano spaziale:
vicino a zero il braccio perde un grado di libertà (singolarità). Il codice è in <code>kin.js</code>, senza dipendenze.</p>
<h2>Profili di moto (SETPOSE)</h2>
<p>RTR5 quintico a jerk minimo, RTR3 cubico, BCB trapezio di velocità, BB bang-bang. Alla fine il firmware scrive
<code>SETPOSE_DONE time_ms=… vel_max=… acc_max=…</code>: il test "Escursione" li raccoglie.</p>
<h2>Esercizi</h2>
<ol>
<li><b>Verifica della cinematica diretta</b>: porta il braccio in 3 pose, misura con un metro la punta rispetto alla
base e confrontala con x, y, z della tabella. Da dove vengono le differenze (offset, giochi, risoluzione di 1°)?</li>
<li><b>Singolarità</b>: trova una posa in cui la manipolabilità va a zero (suggerimento: yaw e roll del polso
allineati, o il braccio tutto disteso). Perché lì lo Jacobiano perde rango?</li>
<li><b>Profili</b>: con il test "Escursione" confronta RTR3, RTR5, BCB e BB a parità di tempo: velocità e
accelerazione massime, rumore, vibrazioni.</li>
<li><b>Calibrazione</b>: trova gli offset che mettono il braccio esattamente dritto in HOME, applicali, salvali.</li>
<li><b>Cinematica inversa</b>: scrivi in un file tuo una funzione che, data una posizione x, y, z, trova gli angoli
(Newton-Raphson con lo Jacobiano di <code>kin.js</code>, Modern Robotics cap. 6). Provala prima sul simulatore.</li>
<li><b>Sequenze</b>: insegna al braccio una sequenza di pose (salva, riordina, esegui) e misurane la ripetibilità.</li>
</ol>
<h2>Telecamere</h2>
<p>Due Camera Module 3 (IMX708) a 63 mm, sincronizzate fra loro, pubblicate da MediaMTX: le app le ricevono in
RTSP, la scheda Camere in WebRTC. Per provarle: avvia il video, controlla l'esito di ogni occhio, inquadra un
oggetto con dettagli e confronta nitidezza e luminosità; con il collimatore attivo la distanza al mirino dice
se le due immagini si sovrappongono.</p>
<h2>Sicurezza</h2>
<ul><li><b>STOP</b> (in alto, o Esc / barra spaziatrice) ferma tutto in ogni momento.</li>
<li>I comandi di movimento sono spenti finché non attivi "comandi di movimento", e si spengono da soli se il
collegamento cade o se il visore sta comandando.</li>
<li>Il firmware rispetta sempre i limiti dei giunti e il fungo di emergenza, qualunque cosa mandi questa pagina.</li>
<li>Senza robot: <code>python3 minimal/j5mini/j5mini.py --sim</code> fa girare il firmware vero simulato sul PC,
con questa pagina su <code>http://localhost:8558</code>.</li></ul>`;
}
