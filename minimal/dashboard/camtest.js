// camtest.js — prova delle telecamere nella dashboard: le due camere dal vivo via WebRTC (WHEP di MediaMTX,
// passando da j5mini sulla stessa porta HTTPS della pagina), statistiche di ricezione per occhio, nitidezza
// (varianza del laplaciano, per la messa a fuoco) e luminosità media a confronto, comandi delle camere.

const CAMS = ["cam0", "cam1"];
const MIN_FPS = 20;

/** Varianza del laplaciano 4-vicini su un'immagine in scala di grigi: cresce con i dettagli a fuoco. */
export function sharpness(gray, w, h) {
  let n = 0, sum = 0, sum2 = 0;
  for (let y = 1; y < h - 1; y++)
    for (let x = 1; x < w - 1; x++) {
      const i = y * w + x;
      const l = 4 * gray[i] - gray[i - 1] - gray[i + 1] - gray[i - w] - gray[i + w];
      sum += l; sum2 += l * l; n++;
    }
  if (!n) return 0;
  const m = sum / n;
  return sum2 / n - m * m;
}

/** RGBA -> grigio (luma BT.601) e media. */
export function toGray(rgba, w, h) {
  const g = new Float32Array(w * h);
  let mean = 0;
  for (let i = 0, j = 0; i < g.length; i++, j += 4) {
    g[i] = 0.299 * rgba[j] + 0.587 * rgba[j + 1] + 0.114 * rgba[j + 2];
    mean += g[i];
  }
  return { gray: g, mean: g.length ? mean / g.length : 0 };
}

/** Esito di un occhio da statistiche di un secondo. */
export function verdict(s) {
  if (!s || !s.connected) return { ok: false, why: "nessun video" };
  if (s.fps < MIN_FPS) return { ok: false, why: `${s.fps.toFixed(0)} fps (< ${MIN_FPS})` };
  if (s.lossPct > 1) return { ok: false, why: `perdite ${s.lossPct.toFixed(1)} %` };
  return { ok: true, why: "ok" };
}

export class CamTest {
  constructor({ send, getTelemetry, getCollimation, toast }) {
    this.send = send; this.getTm = getTelemetry; this.getCol = getCollimation; this.toast = toast;
    this.pcs = [null, null]; this.prev = [null, null]; this.stats = [null, null];
    this.scratch = document.createElement("canvas");
    this.scratch.width = 320; this.scratch.height = 180;
    this.timer = null;
    const $ = id => document.getElementById(id);
    $("cam-start").onclick = () => (this.pcs.some(Boolean) ? this.stop() : this.start());
    $("cam-af").onclick = () => { this.send({ type: "cam_autofocus" }); this.toast("autofocus su tutte e due le camere"); };
    $("cam-mode").onchange = e => this.send({ type: "cam_mode", mode: e.target.value });
    $("cam-zoom").onchange = e => { const z = Number(e.target.value); this.send({ type: "vr_zoom_state", camera: true, zoom0: z, zoom1: z }); };
    $("cam-ev").oninput = e => { $("cam-ev-v").textContent = Number(e.target.value).toFixed(1) + " EV"; };
    $("cam-ev").onchange = e => this.send({ type: "cam_exposure", ev: Number(e.target.value) });
    $("cam-focus").onchange = e => {
      const v = e.target.value;
      this.send(v === "auto" ? { type: "cam_focus", auto: true } : { type: "cam_focus", dioptres: Number(v) });
    };
    CAMS.forEach((c, i) => { $(`cam-shot${i}`).onclick = () => this.snapshot(i); });
  }

  async start() {
    document.getElementById("cam-start").textContent = "Ferma video";
    CAMS.forEach((c, i) => this.connect(i));
    clearInterval(this.timer);
    this.timer = setInterval(() => this.tick(), 1000);
  }

  stop() {
    this.pcs.forEach(pc => pc && pc.close());
    this.pcs = [null, null]; this.prev = [null, null]; this.stats = [null, null];
    CAMS.forEach((c, i) => { document.getElementById(`cam-v${i}`).srcObject = null; });
    clearInterval(this.timer); this.timer = null;
    document.getElementById("cam-start").textContent = "Avvia video";
    this.render();
  }

  /** WHEP senza trickle: offerta con tutti i candidati, una POST, la risposta di MediaMTX. */
  async connect(i) {
    const out = document.getElementById(`cam-s${i}`);
    out.textContent = "collegamento...";
    const pc = new RTCPeerConnection({ iceServers: [] });
    this.pcs[i] = pc;
    pc.addTransceiver("video", { direction: "recvonly" });
    pc.ontrack = ev => { document.getElementById(`cam-v${i}`).srcObject = ev.streams[0] || new MediaStream([ev.track]); };
    try {
      await pc.setLocalDescription(await pc.createOffer());
      await new Promise(res => {
        if (pc.iceGatheringState === "complete") return res();
        const t = setTimeout(res, 2000);
        pc.onicegatheringstatechange = () => { if (pc.iceGatheringState === "complete") { clearTimeout(t); res(); } };
      });
      const r = await fetch(`whep/${CAMS[i]}`, { method: "POST", headers: { "Content-Type": "application/sdp" }, body: pc.localDescription.sdp });
      const body = await r.text();
      if (!r.ok) {
        let msg = body;
        try { msg = JSON.parse(body).error || body; } catch { /* MediaMTX answers in plain text */ }
        if (r.status === 502) msg = "MediaMTX non raggiungibile (servizio video spento; in simulazione è normale)";
        else if (r.status === 404) msg = "camera non pubblicata (controlla jonny5-video-capture)";
        throw new Error(msg.slice(0, 200) || `HTTP ${r.status}`);
      }
      await pc.setRemoteDescription({ type: "answer", sdp: body });
    } catch (e) {
      out.textContent = "errore: " + (e.message || e);
      pc.close(); this.pcs[i] = null;
      if (!this.pcs.some(Boolean)) { clearInterval(this.timer); this.timer = null; document.getElementById("cam-start").textContent = "Avvia video"; }
    }
  }

  async tick() {
    for (let i = 0; i < 2; i++) {
      const pc = this.pcs[i];
      if (!pc) continue;
      let inb = null;
      (await pc.getStats()).forEach(r => { if (r.type === "inbound-rtp" && r.kind === "video") inb = r; });
      const now = performance.now(), p = this.prev[i];
      if (inb && p) {
        const dt = (now - p.t) / 1000, dPk = inb.packetsReceived - p.packetsReceived, dLost = (inb.packetsLost || 0) - p.packetsLost;
        const dDec = (inb.framesDecoded || 0) - p.framesDecoded;
        this.stats[i] = {
          connected: pc.connectionState === "connected" || dDec > 0,
          fps: dDec / dt, kbps: (inb.bytesReceived - p.bytesReceived) * 8 / dt / 1000,
          lossPct: dPk + dLost > 0 ? 100 * Math.max(0, dLost) / (dPk + Math.max(0, dLost)) : 0,
          w: inb.frameWidth || 0, h: inb.frameHeight || 0,
          decMs: dDec > 0 ? 1000 * ((inb.totalDecodeTime || 0) - p.totalDecodeTime) / dDec : 0,
          jitterMs: (inb.jitter || 0) * 1000, dropped: inb.framesDropped || 0, state: pc.connectionState,
        };
      }
      if (inb) this.prev[i] = { t: now, packetsReceived: inb.packetsReceived, packetsLost: inb.packetsLost || 0,
        framesDecoded: inb.framesDecoded || 0, bytesReceived: inb.bytesReceived, totalDecodeTime: inb.totalDecodeTime || 0 };
      this.measure(i);
    }
    this.render();
  }

  /** Nitidezza e luminosità del fotogramma corrente (su una copia 320x180). */
  measure(i) {
    const v = document.getElementById(`cam-v${i}`);
    if (!this.stats[i] || !v.videoWidth) return;
    const c = this.scratch, g = c.getContext("2d", { willReadFrequently: true });
    g.drawImage(v, 0, 0, c.width, c.height);
    const { gray, mean } = toGray(g.getImageData(0, 0, c.width, c.height).data, c.width, c.height);
    this.stats[i].sharp = sharpness(gray, c.width, c.height);
    this.stats[i].luma = mean;
  }

  render() {
    const tm = this.getTm() || {};
    CAMS.forEach((c, i) => {
      const s = this.stats[i], v = verdict(s), out = document.getElementById(`cam-s${i}`);
      if (!this.pcs[i]) { if (!out.textContent.startsWith("errore")) out.textContent = "fermo"; return; }
      if (!s) return;
      out.innerHTML = `<b class="${v.ok ? "on" : "alarm"}">${v.ok ? "OK" : "KO"}: ${v.why}</b>\n` +
        `${s.w}x${s.h}  ${s.fps.toFixed(1)} fps  ${s.kbps.toFixed(0)} kbit/s\n` +
        `perdite ${s.lossPct.toFixed(2)} %  jitter ${s.jitterMs.toFixed(1)} ms  decodifica ${s.decMs.toFixed(1)} ms\n` +
        `nitidezza ${s.sharp != null ? s.sharp.toFixed(0) : "–"}  luminosità ${s.luma != null ? s.luma.toFixed(0) : "–"}/255`;
    });
    const [a, b] = this.stats, col = this.getCol();
    const cmp = [];
    if (a && b && a.sharp != null && b.sharp != null) {
      const r = Math.min(a.sharp, b.sharp) / Math.max(a.sharp, b.sharp, 1e-9);
      cmp.push(`nitidezza destra/sinistra ${(100 * r).toFixed(0)} % ${r < 0.6 ? "→ una camera è sfocata: prova Autofocus" : ""}`);
      cmp.push(`differenza di luminosità ${Math.abs(a.luma - b.luma).toFixed(0)}/255 ${Math.abs(a.luma - b.luma) > 25 ? "→ esposizioni diverse" : ""}`);
    }
    cmp.push(`sensore: ${tm.cam_mode ? `${tm.cam_mode}, ${tm.cam_fps ?? "–"} fps, ${tm.cam_hfov ?? "–"}°, zoom ${tm.cam_zoom ?? "–"}×` : "nessun dato da cam_stream"}`);
    cmp.push(`collimatore: ${col ? (col.lock ? `agganciato, ${col.dist_mm ? (col.dist_mm / 10).toFixed(1) + " cm" : "distanza –"}` : "cerca...") : "nessun dato"}`);
    document.getElementById("cam-cmp").textContent = cmp.join("\n");
  }

  snapshot(i) {
    const v = document.getElementById(`cam-v${i}`);
    if (!v.videoWidth) { this.toast("nessun fotogramma", true); return; }
    const c = document.createElement("canvas");
    c.width = v.videoWidth; c.height = v.videoHeight;
    c.getContext("2d").drawImage(v, 0, 0);
    const a = document.createElement("a");
    a.href = c.toDataURL("image/png");
    a.download = `jonny5-${CAMS[i]}-${new Date().toISOString().replace(/[:.]/g, "-")}.png`;
    a.click();
  }
}
