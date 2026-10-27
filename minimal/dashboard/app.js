// app.js — JONNY5 control room: stato e comandi principali, braccio con cinematica, camere, teleoperazione,
// console, impostazioni, test. Parla solo con j5mini (WebSocket, stessi messaggi dell'app Quest), che inoltra le
// righe UART allo STM32. Nessuna libreria: la pagina deve funzionare sull'hotspot del robot, senza internet.
import * as K from "./kin.js";
import { View3D } from "./view3d.js";
import { CamTest } from "./camtest.js";

const $ = id => document.getElementById(id);
const JN = ["Base", "Spalla", "Gomito", "Yaw polso", "Pitch polso", "Roll polso"];
const SK = ["servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R"];
const MOTION_CMDS = /^(HOME|PARK|TELEOPPOSE|SETPOSE)/;
const MODES = { 0: "PREPARAZIONE", 1: "POSE", 2: "MANUALE", 3: "HEAD", 4: "HYBRID", 5: "ASSIST" };
const QS = new URLSearchParams(location.search);
const APP = QS.get("app");                       // ?app=phone: dentro il WebView dell'app telefono (niente riga del marchio)
const S = {
  ws: null, connected: false, cfg: null, kin: null, tm: null, col: null, colAt: 0,
  target: [90, 90, 90, 90, 90, 90], pending: [], waiters: [], mine: new Set(), abort: false,
  samples: [], hist: [], tmCount: 0, tmRate: 0, uiQueued: false, poses: [],
  imuRef: null, imuRefLabel: "", homePending: false, horizon: false, camp: false, visionReq: 0, wsAlt: false,
};
if (APP) document.body.classList.add("app-" + APP);

// ------------------------------------------------------------------ WebSocket
/** Stessa origine della pagina (wss://robot/ws, un solo certificato); se il tunnel non risponde, la porta 8557 diretta. */
function wsUrl() {
  const q = QS.get("ws");
  if (q) return q;
  const proto = location.protocol === "https:" ? "wss" : "ws";
  if (!S.wsAlt) return `${proto}://${location.host}/ws`;
  return `${proto}://${location.hostname || "localhost"}:8557`;
}

function connect() {
  const url = wsUrl();
  let ws, opened = false;
  try { ws = new WebSocket(url); } catch (e) { return setTimeout(connect, 2000); }
  S.ws = ws;
  ws.onopen = () => { opened = true; S.connected = true; send({ type: "get_config" }); badges(); log(`collegato a ${url}`, "sys"); };
  ws.onclose = () => {
    if (S.connected) log("collegamento perso, riprovo...", "err");
    S.connected = false; setMotion(false); badges();
    if (!opened && !QS.get("ws")) S.wsAlt = !S.wsAlt;             // alterna /ws e :8557 finché uno risponde
    if (!opened && S.wsAlt && location.protocol === "https:" && !S.certHint) {
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

/** Waits for an unsolicited line matching re (e.g. SETPOSE_DONE). */
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
      if (typeof m.horizon_lock === "boolean" && m.horizon_lock !== S.horizon) { S.horizon = m.horizon_lock; horizonButtons(); }
      if (!S.targetInit && S.jointsBuilt && measuredVirtual()) { S.targetInit = true; S.target = measuredVirtual(); buildJoints(); }
      S.samples.push({ t: performance.now(), link: m.link });
      while (S.samples.length > 2 && S.samples[0].t < performance.now() - 1500) S.samples.shift();
      queueUi();
      break;
    case "config":
      S.cfg = m;
      $("b-sim").classList.toggle("hidden", !m.sim);
      if (typeof m.horizon_lock === "boolean") { S.horizon = m.horizon_lock; horizonButtons(); }
      buildJoints(); fillSettings(); buildVrp(); queueUi();
      send({ type: "get_kinematics" });
      break;
    case "kinematics":
      if (!Array.isArray(m.S)) { toast("cinematica non disponibile: vista 3D disattivata", true); break; }
      S.kin = { S: m.S, M: m.M, draw_points: m.draw_points };
      fillKin(); draw3d(measuredVirtual());
      break;
    case "kinematics_saved":
      $("kin-out").textContent = m.ok ? `salvata ${m.saved_to_file ? "in kinematics.json (copia precedente in .bak)" : "solo in memoria (modalità simulata)"} e passata al gimbal ASSIST` : "errore: " + m.error;
      toast(m.ok ? "cinematica salvata" : "errore: " + m.error, !m.ok);
      break;
    case "uart_response": {
      const i = S.pending.findIndex(p => p.cmd === m.cmd);
      const mine = i >= 0 || S.mine.has(m.cmd);
      if (i >= 0) { const p = S.pending.splice(i, 1)[0]; clearTimeout(p.timer); p.resolve(m); }
      if (mine || $("log-poll").checked || m.cmd !== "STATUS?") log(`← ${m.cmd}: ${m.response}`, m.ok ? "resp" : "err");
      if (m.response && m.response.startsWith("STATUS:") && S.tm) { S.tm.robot_state = m.response.slice(7).trim(); queueUi(); }
      if (m.cmd === "HOME" && m.ok) S.homePending = true;      // l'IMU prende HOME come riferimento a fine corsa
      break;
    }
    case "uart_unsolicited":
      log("⇠ " + m.line, "uns");
      for (const w of S.waiters.slice()) if (w.re.test(m.line)) {
        S.waiters.splice(S.waiters.indexOf(w), 1); clearTimeout(w.timer); w.resolve(m.line);
      }
      if (/^SETPOSE_DONE/.test(m.line) && S.homePending) { S.homePending = false; setTimeout(() => imuReference("HOME"), 500); }
      break;
    case "stereo_collimation": S.col = m; S.colAt = Date.now(); break;
    case "ws_pong": if (S.onPong) S.onPong(m); break;
    case "horizon_lock": S.horizon = !!m.on; horizonButtons(); toast(S.horizon ? "Orizzonte acceso" : "Orizzonte spento"); break;
    case "error": toast("j5mini: " + m.error, true); log("errore: " + m.error, "err"); break;
    case "imu_campaign":
      S.camp = !!m.on; $("tp-camp").classList.toggle("on", S.camp);
      $("tp-camp").textContent = S.camp ? "ferma registrazione" : "avvia registrazione";
      $("tp-camp-out").textContent = (S.camp ? "registro in " : "ferma") + (m.path ? `: ${m.path}` : "") +
        (m.why ? `\n${m.why}` : "") + (m.n != null ? `\ncampioni: ${m.n}` : "");
      break;
    case "vision_result": showVision(m); break;
    case "vision_point":
      $("tp-vision-out").textContent += `\npuntamento ${m.label || ""}: ${m.ok ? (m.done ? "arrivato" : "in corso") : "no"}` +
        (m.why ? ` (${m.why})` : "") + (m.T_s ? `, ${Number(m.T_s).toFixed(1)} s` : "");
      break;
    case "config_saved":
      $("set-out").textContent = m.ok
        ? `salvato ${m.saved_to_file ? "in j5mini.json (copia precedente in .bak)" : "solo in memoria (modalità simulata)"}:\n` + m.boot_commands.join("\n")
        : "errore: " + m.error;
      $("vrp-out").textContent = m.ok ? "salvato" : "errore: " + m.error;
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

// ------------------------------------------------------------------ angles, quaternions
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
const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
const qconj = q => [q[0], -q[1], -q[2], -q[3]];
function rpyDeg([w, x, y, z]) {
  return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))].map(a => a / K.DEG);
}
const rpyText = q => q ? rpyDeg(q).map(a => a.toFixed(1) + "°").join("  ") : "–";
const quatText = q => q ? q.map(v => v.toFixed(3)).join(" ") : "–";
function imuQuat() { return S.tm && typeof S.tm.imu_q_w === "number" ? [S.tm.imu_q_w, S.tm.imu_q_x, S.tm.imu_q_y, S.tm.imu_q_z] : null; }
function imuReference(label) {
  const q = imuQuat();
  if (!q) return toast("IMU non disponibile", true);
  S.imuRef = q; S.imuRefLabel = `${label} ${new Date().toTimeString().slice(0, 5)}`;
  $("imu-ref-lbl").textContent = "riferita a " + S.imuRefLabel;
  queueUi();
}
$("imu-zero").onclick = () => imuReference("manuale");
$("imu-raw").onclick = () => { S.imuRef = null; $("imu-ref-lbl").textContent = "grezza"; queueUi(); };

// ------------------------------------------------------------------ status tab, pills, hero
function badges() {
  const b = (id, text, cls) => { const e = $(id); e.textContent = text; e.className = "pill " + (cls || ""); };
  b("b-link", S.connected ? "collegato" : "non collegato", S.connected ? "ok" : "off");
  const tm = S.tm || {};
  const st = S.connected ? (tm.robot_state || "?") : "?";
  b("b-state", st, { IDLE: "ok", SAFE: "warn", STOPPED: "bad", UNKNOWN: "dim" }[st] || "dim");
  if (tm.estop_bypass) b("b-estop", "E-STOP BYPASS", "warn");
  else if (tm.estop_active === true) b("b-estop", "E-STOP PREMUTO", "bad");
  else if (tm.estop_active === false) b("b-estop", "E-STOP ok", "ok");
  else b("b-estop", "E-STOP ?", "dim");
  const live = S.connected && teleopLive();
  b("b-teleop", live ? `visore: ${MODES[tm.teleop_mode] || "modo " + tm.teleop_mode}` : "visore: no", live ? "warn" : "dim");
  b("b-horizon", S.horizon ? "orizzonte ON" : "orizzonte off", S.horizon ? "teal" : "dim");
  document.body.classList.toggle("teleop-live", live);
  $("hero-sub").textContent = !S.connected ? "in attesa del collegamento con j5mini…" :
    `${st === "IDLE" ? "pronto (IDLE)" : st === "SAFE" ? "servo spenti (SAFE)" : st === "STOPPED" ? "fermo (STOPPED): SAFE poi ENABLE per ripartire" : "stato " + st}` +
    ` · fungo ${tm.estop_bypass ? "in bypass" : tm.estop_active ? "PREMUTO" : "ok"}` +
    ` · ${live ? "visore in " + (MODES[tm.teleop_mode] || tm.teleop_mode) : "visore assente"}` +
    (tm.link && S.tmRate ? ` · SPI ${S.lastSpiRate ?? "–"}/s` : "");
}

function horizonButtons() {
  for (const id of ["h-horizon", "tp-horizon"]) $(id).classList.toggle("on", S.horizon);
  badges();
}

function updateUi() {
  badges();
  const tm = S.tm;
  if (!tm) return;
  let rate = {};
  if (S.samples.length >= 2) {
    const a = S.samples[0], z = S.samples[S.samples.length - 1], dt = (z.t - a.t) / 1000;
    if (dt > 0.3) for (const k of Object.keys(z.link)) rate[k] = (z.link[k] - a.link[k]) / dt;
  }
  S.lastSpiRate = rate.spi_rx_ok != null ? rate.spi_rx_ok.toFixed(0) : null;
  dl($("st-robot"), [
    ["stato", tm.robot_state, tm.robot_state === "IDLE" ? "on" : tm.robot_state === "STOPPED" ? "alarm" : tm.robot_state === "SAFE" ? "warnv" : ""],
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
    ["fase DMA (byte)", tot.spi_shift ?? "–", tot.spi_shift ? "warnv" : ""],
    ["payload neutri", `${fx(rate.neutral && rate.spi_tx ? 100 * rate.neutral / rate.spi_tx : 0, 0)} %`],
    ["frame interpolati", `${fx(rate.smoothed, 0)}/s`],
    ["UART tx / rx", `${tot.uart_tx ?? "–"} / ${tot.uart_rx ?? "–"}`],
    ["timeout UART", tot.uart_timeouts ?? "–", tot.uart_timeouts ? "alarm" : ""],
    ["telemetria", `${S.tmRate} msg/s`], ["client collegati", tm.clients ?? "–"],
  ]);
  const vrRows = [
    ["visore", tm.teleop_mode >= 0 ? `attivo, modo ${tm.teleop_mode} ${MODES[tm.teleop_mode] || ""}` : "nessuno", tm.teleop_mode >= 0 ? "warnv" : ""],
    ["età ultimo intent", tm.intent_age_ms != null ? tm.intent_age_ms + " ms" : "mai"],
    ["heartbeat", tm.vr_heartbeat ?? "–"], ["modo visto dal firmware", tm.mode_echo != null ? `${tm.mode_echo} ${MODES[tm.mode_echo] || ""}` : "–"],
    ["testa roll / pitch / yaw", rpyText(tm.vr_q)],
    ["zoom richiesto", tm.vr_zoom0 != null ? fx(tm.vr_zoom0, 2) + "×" : "–"],
  ];
  dl($("st-teleop"), vrRows);
  dl($("tp-vr"), [...vrRows, ["quaternione testa", quatText(tm.vr_q)]]);
  const q = imuQuat();
  const qr = q && S.imuRef ? qmul(qconj(S.imuRef), q) : q;
  dl($("st-imu"), [
    ["valida", ...yes(tm.imu_valid)],
    ["roll / pitch / yaw", rpyText(qr)],
    ["quaternione", quatText(q)],
    ["riferimento", S.imuRef ? S.imuRefLabel : "grezza (nessuno)"],
  ]);
  dl($("tp-horizon-dl"), [
    ["stato", S.horizon ? "acceso" : "spento", S.horizon ? "on" : "offv"],
    ["trim dall'IMU", tm.horizon_trim != null ? fx(tm.horizon_trim, 2) + "°" : "–"],
    ["corregge", S.horizon && teleopLive() && tm.deadman_active ? "sì (grip premuti)" : "no: serve un modo attivo con i grip premuti", S.horizon && teleopLive() && tm.deadman_active ? "on" : "offv"],
  ]);
  const colFresh = S.col && Date.now() - S.colAt < 2000;
  dl($("st-cam"), [
    ["modo sensore", tm.cam_mode ? `${tm.cam_mode} (${fx(tm.cam_hfov, 1)}°, ${tm.cam_fps ?? "–"} fps)` : "nessun dato"],
    ["zoom applicato", tm.cam_zoom != null ? fx(tm.cam_zoom, 2) + "×" : "–"], ["luminosità", tm.cam_ev != null ? fx(tm.cam_ev, 2) + " EV" : "–"],
    ["collimatore", colFresh ? (S.col.lock ? "agganciato" : "cerca...") : "nessun dato", colFresh && S.col.lock ? "on" : ""],
    ["distanza al mirino", colFresh && S.col.dist_mm ? (S.col.dist_mm / 10).toFixed(1) + " cm" : "–"],
  ]);
  $("cal-now").textContent = colFresh ? `collimatore: ${S.col.lock ? "agganciato" : "cerca"}, distanza ${S.col.dist_mm ? (S.col.dist_mm / 10).toFixed(1) + " cm" : "–"}` : "collimatore: nessun dato";
  const mv = measuredVirtual();
  dl($("st-joints"), SK.map((k, i) => [JN[i], `${tm[k] ?? "–"}°  (virtuale ${mv ? mv[i].toFixed(0) : "–"}°)`]));
  updateJointsMeas(mv);
  if ($("impostazioni").classList.contains("active")) SK.forEach((k, i) => { const e = $("s-cur" + i); if (e) e.textContent = tm[k] != null ? tm[k] + "°" : "–"; });
  draw3d(mv);
}

function drawChart() {
  const c = $("chart"), g = c.getContext("2d"), dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = c.clientHeight;
  if (!w) return;
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const H = S.hist, max = 120;
  g.strokeStyle = "rgba(61,157,255,.15)"; g.lineWidth = 1; g.fillStyle = "#5c6f8a"; g.font = "11px system-ui";
  for (const v of [0, 50, 100]) { const y = h - 6 - (v / max) * (h - 12); g.beginPath(); g.moveTo(30, y); g.lineTo(w, y); g.stroke(); g.fillText(v, 4, y + 4); }
  const series = [["ok", "#69f0ae", 1], ["tm", "#3d9dff", 5], ["bad", "#ff6e66", 10]];
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

// ------------------------------------------------------------------ hero: main commands
$("h-enable").onclick = async () => {
  if (!S.connected) return toast("non collegato", true);
  const a = await uart("SAFE"), b = await uart("ENABLE");
  toast(b.ok ? "robot in IDLE" : `ENABLE rifiutato: ${b.response}`, !b.ok);
  if (!a.ok) toast(`SAFE: ${a.response}`, true);
  uart("STATUS?");
};
$("h-safe").onclick = async () => { const r = await uart("SAFE"); toast(r.ok ? "SAFE: servo spenti" : r.response, !r.ok); uart("STATUS?"); };
for (const [id, cmd] of [["h-home", "HOME"], ["h-park", "PARK"], ["h-vr", "TELEOPPOSE"]]) {
  $(id).onclick = async () => {
    if (!motionAllowed()) return;
    if (S.tm && S.tm.robot_state !== "IDLE") return toast(`robot in ${S.tm.robot_state}: serve IDLE (ENABLE)`, true);
    const r = await uart(cmd);
    if (!r.ok) toast(`${cmd} rifiutato: ${r.response}`, true);
  };
}
$("h-horizon").onclick = $("tp-horizon").onclick = () => {
  if (!send({ type: "horizon_lock", on: !S.horizon })) toast("non collegato", true);
};

// ------------------------------------------------------------------ arm tab: joints, poses, 3D
const view = new View3D($("view3d"));

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
      <input type="number" min="${lo.toFixed(1)}" max="${hi.toFixed(1)}" step="0.5" id="j-n${i}" inputmode="decimal">
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

// ------------------------------------------------------------------ camera tab extras (the video test is camtest.js)
$("cal-go").onclick = () => {
  const mm = Number($("cal-mm").value);
  if (!(mm >= 100 && mm <= 5000)) return toast("distanza fra 100 e 5000 mm", true);
  if (!S.col || Date.now() - S.colAt > 2000) return toast("collimatore non attivo: nessun dato stereo", true);
  if (send({ type: "stereo_calibrate", distance_mm: mm })) toast(`calibrazione a ${mm} mm richiesta al collimatore`);
};

// ------------------------------------------------------------------ teleop tab: vision, campaign, VR params
function showVision(m) {
  const lines = [`analisi ${m.action || ""} ${m.ok ? "ok" : "fallita"}${m.why ? ": " + m.why : ""}` + (m.zoom ? ` (zoom ${m.zoom}×)` : "")];
  const t = m.target;
  if (t) lines.push(`sotto il mirino: ${t.label_it || t.label || "?"}` + (t.dist_mm ? ` a ${(t.dist_mm / 10).toFixed(1)} cm` : "") + (t.conf ? ` (conf. ${Number(t.conf).toFixed(2)})` : ""));
  if (Array.isArray(m.objects) && m.objects.length)
    lines.push("oggetti: " + m.objects.slice(0, 8).map(o => (o.label_it || o.label || "?") + (o.dist_mm ? ` ${(o.dist_mm / 10).toFixed(0)} cm` : "")).join(", "));
  if (Array.isArray(m.texts) && m.texts.length) lines.push("testo: " + m.texts.map(x => x.text || x).join(" "));
  $("tp-vision-out").textContent = lines.join("\n");
}
for (const [id, action] of [["tp-analyze", "analyze"], ["tp-point", "point"]]) {
  $(id).onclick = () => {
    if (!send({ type: "vision_request", action, req: "dash" + (++S.visionReq) })) return toast("non collegato", true);
    $("tp-vision-out").textContent = action === "point" ? "cerco l'oggetto sotto il mirino..." : "analizzo...";
  };
}
$("tp-headzero").onclick = () => uart("HEADZERO");
$("tp-camp").onclick = () => {
  if (!send({ type: "imu_campaign", on: !S.camp, label: $("tp-camp-label").value.trim() })) toast("non collegato", true);
};

/** The 34 fields of SET_VR_PARAMS in the firmware's order (j5vr_set_vr_params): 16 floats, then 18 integers. */
const VRP = [
  ["g", "Guadagni e filtri della testa (float)"],
  ["yaw_gain", "guadagno yaw"], ["pitch_gain", "guadagno pitch"], ["roll_gain", "guadagno roll"],
  ["alpha_small", "filtro EMA, movimenti piccoli"], ["alpha_large", "filtro EMA, movimenti grandi"],
  ["deadzone_deg", "zona morta (°)"], ["max_step_deg_s", "passo massimo (°/s)"],
  ["vel_max", "velocità massima (°/s)"], ["vel_digital", "velocità servo digitali (°/s)"],
  ["lpf_pitch", "filtro pitch (ignorato dal fw)"], ["lpf_roll", "filtro roll (ignorato dal fw)"],
  ["joy_dz", "zona morta stick"], ["head_sensitivity", "sensibilità testa (0.25..4)"],
  ["sign_yaw", "segno yaw (±1)"], ["sign_pitch", "segno pitch (±1)"], ["sign_roll", "segno roll (±1)"],
  ["g", "Instradamento visore → polso (sorgente: 0 roll, 1 pitch, 2 yaw del visore; abilitato 0/1)"],
  ["src_roll", "sorgente del roll"], ["src_pitch", "sorgente del pitch"], ["src_yaw", "sorgente dello yaw"],
  ["en_roll", "roll abilitato"], ["en_pitch", "pitch abilitato"], ["en_yaw", "yaw abilitato"],
  ["g", "Velocità massime in MANUALE (°/s; 0 = quella dei servo digitali)"],
  ["vel_base", "base"], ["vel_spalla", "spalla"], ["vel_gomito", "gomito"], ["vel_yaw", "yaw"], ["vel_pitch", "pitch"], ["vel_roll", "roll"],
  ["g", "Velocità massime in HEAD / HYBRID / ASSIST (°/s)"],
  ["vel_yaw_head", "yaw"], ["vel_pitch_head", "pitch"], ["vel_roll_head", "roll"], ["vel_base_head", "base"], ["vel_spalla_head", "spalla"], ["vel_gomito_head", "gomito"],
];
function vrpValues() { return $("set-vr").value.trim().split(/\s+/).slice(1); }
function buildVrp() {
  const vals = vrpValues();
  let n = 0;
  $("vrp").innerHTML = VRP.map(([k, lbl]) => k === "g" ? `<div class="grp"><h4>${lbl}</h4></div>` :
    `<label>${lbl}<input id="vrp-${k}" data-i="${n}" value="${vals[n++] ?? ""}" inputmode="decimal"></label>`).join("");
  $("vrp").querySelectorAll("input").forEach(inp => inp.onchange = () => {
    const v = vrpValues();
    while (v.length < 34) v.push("0");
    v[Number(inp.dataset.i)] = inp.value.trim();
    $("set-vr").value = "SET_VR_PARAMS " + v.join(" ");
    $("vrp-out").textContent = $("set-vr").value;
  });
}
$("vrp-apply").onclick = async () => {
  if (!motionAllowed()) return;
  const line = $("set-vr").value.trim().replace(/\s+/g, " ");
  const r = await uart(line);
  $("vrp-out").textContent = `${r.ok ? "OK " : "ERR"} ${line} → ${r.response}\n\nApplicato solo al firmware acceso: per tenerlo, Salva in j5mini.json.`;
};
$("vrp-save").onclick = () => send({ type: "save_config", ...readSettings() });
$("vrp-reset").onclick = () => send({ type: "get_config" });

// ------------------------------------------------------------------ commands tab
const CMDS = [
  ["ENABLE", "SAFE → IDLE (non muove)"], ["SAFE", "servo spenti, si riarma da solo con un flusso VR"],
  ["STOP", "STOPPED: per uscire SAFE poi ENABLE"],
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
      <td><input type="number" id="s-off${i}" min="0" max="180" value="${c.offsets[i]}" inputmode="numeric"></td>
      <td><select id="s-dir${i}"><option value="1"${c.dirs[i] === 1 ? " selected" : ""}>+1</option><option value="-1"${c.dirs[i] === -1 ? " selected" : ""}>−1</option></select></td>
      <td><input type="number" id="s-min${i}" min="0" max="180" value="${c.limits[i][0]}" inputmode="numeric"></td>
      <td><input type="number" id="s-max${i}" min="0" max="180" value="${c.limits[i][1]}" inputmode="numeric"></td>
      <td>${lo.toFixed(0)}..${hi.toFixed(0)}</td>
      <td class="live" id="s-cur${i}">–</td>
      <td class="jog"><button type="button" class="tiny motion" data-jog="${i}" data-d="-1" title="muove solo questo giunto, di un passo, verso il basso dell'angolo fisico">−</button><button type="button" class="tiny motion" data-jog="${i}" data-d="1" title="muove solo questo giunto, di un passo, verso l'alto dell'angolo fisico">+</button><button type="button" class="tiny motion" data-wig="${i}" title="prova: +passo, −passo, ritorno (per vedere il verso e il gioco)">⇄</button><button type="button" class="tiny" data-off="${i}" title="prende l'angolo fisico misurato adesso come offset (porta prima il giunto dritto con − e +)">← fisico</button></td></tr>`;
  });
  $("set-joints").innerHTML = `<tr><th></th><th>offset (HOME)</th><th>dir</th><th>min fisico</th><th>max fisico</th><th>virtuale</th><th>fisico ora</th><th>muovi sul robot</th></tr>` + rows.join("");
  $("set-joints").querySelectorAll("button[data-jog]").forEach(b => b.onclick = () => jogJoint(Number(b.dataset.jog), Number(b.dataset.d) * Number($("set-step").value)));
  $("set-joints").querySelectorAll("button[data-wig]").forEach(b => b.onclick = () => wiggleJoint(Number(b.dataset.wig), Number($("set-step").value)));
  $("set-joints").querySelectorAll("button[data-off]").forEach(b => b.onclick = () => {
    const i = Number(b.dataset.off), v = S.tm && S.tm[SK[i]];
    if (typeof v !== "number") return toast("nessuna telemetria", true);
    $("s-off" + i).value = v;
    toast(`${JN[i]}: offset = ${v}° (fisico misurato). Poi Applica, controlla in HOME, Salva`);
  });
  $("set-vr").value = c.vr_params || "";
  $("set-pwm").value = c.pwm_config || "";
  const pose = (key, name) => `<tr><td>${name}</td>` +
    (c[key] || [90, 90, 90, 90, 90, 90]).map((v, i) => `<td><input type="number" class="pose" id="s-${key}${i}" min="0" max="180" step="0.5" value="${v}" inputmode="decimal"></td>`).join("") +
    `<td><button type="button" class="tiny" data-pose="${key}">← target</button></td></tr>`;
  $("set-poses").innerHTML = `<tr><th></th>${JN.map(n => `<th>${n.replace(" polso", "")}</th>`).join("")}<th></th></tr>` + pose("park", "PARK") + pose("vr", "VR pose");
  $("set-poses").querySelectorAll("button[data-pose]").forEach(b => b.onclick = () => {
    S.target.forEach((v, i) => { $(`s-${b.dataset.pose}${i}`).value = v; });
  });
  $("set-speed").value = c.speed_pct ?? 40;
  $("set-prof").value = c.profile || "RTR5";
}
/** SETPOSE_T_HR from PHYSICAL degrees (the settings page reasons in servo angles). */
function setposePhysCmd(phys, ms, prof = "RTR5") {
  return `SETPOSE_T_HR ${phys.map(p => Math.round(p * 10)).join(" ")} ${Math.round(ms)} ${prof}`;
}
function measuredPhysical() { return S.tm && SK[0] in S.tm ? SK.map(k => S.tm[k]) : null; }
/** Moves one joint on the robot by delta physical degrees from where it is now; resolves with the SETPOSE_DONE line. */
async function jogJoint(i, delta) {
  if (!motionAllowed()) return null;
  if (S.tm && S.tm.robot_state !== "IDLE") { toast(`robot in ${S.tm.robot_state}: serve IDLE (ENABLE)`, true); return null; }
  const phys = measuredPhysical();
  if (!phys) { toast("nessuna telemetria", true); return null; }
  const lim = S.cfg ? S.cfg.limits[i] : [0, 180];
  const want = Math.max(5, Math.min(175, Math.max(lim[0], Math.min(lim[1], phys[i] + delta))));
  if (want === phys[i]) { toast(`${JN[i]}: già al limite (${phys[i]}°)`, true); return null; }
  phys[i] = want;
  const ms = Math.max(300, Math.abs(delta) * 1000 / 30);           // ~30 deg/s: slow enough to watch
  const done = waitLine(/^SETPOSE_DONE/, ms + 3000);
  const r = await uart(setposePhysCmd(phys, ms));
  if (!r.ok) { toast("rifiutato: " + r.response, true); return null; }
  return done;
}
/** +A, -A, back: shows the direction of the joint and its play. */
async function wiggleJoint(i, A) {
  const start = measuredPhysical();
  if (!start) return toast("nessuna telemetria", true);
  S.abort = false;
  for (const target of [start[i] + A, start[i] - A, start[i]]) {
    if (S.abort) return;
    const phys = measuredPhysical() || start;
    const d = await jogJoint(i, target - phys[i]);
    if (!d || !(await d)) return;
    await sleep(250);
  }
  toast(`${JN[i]}: prova finita, tornato a ${start[i]}°`);
}

function readSettings() {
  const num = id => Number($(id).value);
  return {
    offsets: JN.map((_, i) => num("s-off" + i)), dirs: JN.map((_, i) => num("s-dir" + i)),
    limits: JN.map((_, i) => [num("s-min" + i), num("s-max" + i)]),
    vr_params: $("set-vr").value.trim().replace(/\s+/g, " "), pwm_config: $("set-pwm").value.trim().replace(/\s+/g, " "),
    park: JN.map((_, i) => num("s-park" + i)), vr: JN.map((_, i) => num("s-vr" + i)),
    speed_pct: num("set-speed"), profile: $("set-prof").value,
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
  if ($("set-home").checked) {                  // the arm goes to the new HOME at once: you see the offsets live
    if (S.tm && S.tm.robot_state !== "IDLE") out.push(`HOME non inviato: robot in ${S.tm.robot_state} (serve IDLE)`);
    else { const r = await uart("HOME"); out.push(`${r.ok ? "OK " : "ERR"}  HOME  →  ${r.response}`); }
  }
  $("set-out").textContent = out.join("\n") + "\n\nApplicato solo al firmware acceso: per tenerlo, Salva in j5mini.json (le pose si salvano soltanto).";
};

// ------------------------------------------------------------------ settings: POE editor
// shown in millimetres (v of the screw axes, position of M, draw points); the file and kin.js stay in metres
const KIN_COLS = { s: ["wx", "wy", "wz", "vx (mm)", "vy (mm)", "vz (mm)"], m: ["r1", "r2", "r3", "pos (mm)"], dp: ["x (mm)", "y (mm)", "z (mm)"] };
const KIN_MM = { "kin-s": j => j >= 3, "kin-m": j => j === 3, "kin-dp": () => true };   // which columns are lengths
const r3 = v => Math.round(v * 1000) / 1000, r6 = v => Math.round(v * 1e6) / 1e6;   // 0.001 mm shown, 1 um kept
function kinTable(id, cols, rowNames, rows, fixedLast = false) {
  const head = `<tr><th></th>${cols.map(c => `<th>${c}</th>`).join("")}</tr>`;
  const body = rows.map((r, i) => `<tr><td>${rowNames[i]}</td>` + r.map((v, j) => {
    const mm = KIN_MM[id](j), shown = mm ? r3(v * 1000) : v;
    return fixedLast && i === rows.length - 1 ? `<td class="fixed">${v}</td>` :
      `<td><input type="number" step="${mm ? 0.1 : 0.0001}" id="${id}-${i}-${j}" value="${shown}" inputmode="decimal"></td>`;
  }).join("") + "</tr>").join("");
  $(id).innerHTML = head + body;
}
function fillKin() {
  const k = S.kin;
  if (!k) return;
  kinTable("kin-s", KIN_COLS.s, JN, k.S);
  kinTable("kin-m", KIN_COLS.m, ["x", "y", "z", ""], k.M, true);
  kinTable("kin-dp", KIN_COLS.dp, JN, k.draw_points);
  for (const id of ["kin-s", "kin-m", "kin-dp"]) $(id).querySelectorAll("input").forEach(inp => inp.onchange = () => {
    S.kin = readKin(); draw3d(measuredVirtual()); $("kin-out").textContent = "modificata (anteprima nella vista 3D): non ancora salvata";
  });
}
function readKin() {
  const num = (id, j) => { const v = Number($(`${id}-${j}`).value); return KIN_MM[id.split("-").slice(0, 2).join("-")](Number(j.split("-")[1])) ? r6(v / 1000) : v; };
  const cell = (id, i, j) => num(id, `${i}-${j}`);
  return {
    S: JN.map((_, i) => KIN_COLS.s.map((_, j) => cell("kin-s", i, j))),
    M: [0, 1, 2].map(i => KIN_COLS.m.map((_, j) => cell("kin-m", i, j))).concat([[0, 0, 0, 1]]),
    draw_points: JN.map((_, i) => KIN_COLS.dp.map((_, j) => cell("kin-dp", i, j))),
  };
}
$("kin-save").onclick = () => { if (!send({ type: "save_kinematics", ...readKin() })) toast("non collegato", true); };
$("kin-reload").onclick = () => send({ type: "get_kinematics" });

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
  say(`4. ENABLE da STOPPED → ${r2.response} ${!r2.ok ? "✓ rifiutato (lo STOP resta finché non si passa da SAFE)" : "✗ doveva essere rifiutato"}`);
  const r3 = await uart("SAFE"), r4 = await uart("ENABLE");
  say(`5. SAFE → ${r3.response}, ENABLE → ${r4.response} ${r3.ok && r4.ok ? "✓ di nuovo IDLE" : "✗"}`);
  say(lines.every(l => !l.includes("✗")) ? "ESITO: SUPERATO" : "ESITO: CONTROLLARE i punti ✗");
};

// ------------------------------------------------------------------ header, tabs, keyboard
$("motion").onchange = e => setMotion(e.target.checked);
$("stop").onclick = () => { S.abort = true; uart("STOP"); toast("STOP inviato"); };
document.addEventListener("keydown", e => {
  const typing = /INPUT|TEXTAREA|SELECT/.test(document.activeElement.tagName);
  if (e.key === "Escape" || (e.key === " " && !typing)) { e.preventDefault(); $("stop").click(); }
});
function showTab(name) {
  document.querySelectorAll("#tabs button, .tab").forEach(x => x.classList.remove("active"));
  document.querySelector(`#tabs button[data-tab="${name}"]`).classList.add("active"); $(name).classList.add("active");
  window.scrollTo(0, 0);
  if (name === "braccio") view.draw();
  if (name === "stato") drawChart();
  if (name !== "camere" && camTest.pcs.some(Boolean)) camTest.stop();   // the video costs CPU on the Pi
  try { localStorage.setItem("j5.tab", name); } catch { /* private mode */ }
}
document.querySelectorAll("#tabs button").forEach(b => b.onclick = () => showTab(b.dataset.tab));
const headerH = () => document.documentElement.style.setProperty("--header-h", document.querySelector("header").offsetHeight + "px");
new ResizeObserver(headerH).observe(document.querySelector("header"));
headerH();

const camTest = new CamTest({ send, getTelemetry: () => S.tm, toast,
  getCollimation: () => (S.col && Date.now() - S.colAt < 2000 ? S.col : null) });
loadPoses(); renderPoses();
try { const t = localStorage.getItem("j5.tab"); if (t && $(t)) showTab(t); } catch { /* private mode */ }
connect();
