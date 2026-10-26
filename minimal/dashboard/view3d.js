// view3d.js — vista 3D minima del braccio su canvas 2D (niente librerie: il robot non ha internet).
// Prospettiva semplice con orbita attorno all'origine. Mondo: X avanti (verso le camere), Y a sinistra, Z su.

export class View3D {
  constructor(canvas) {
    this.c = canvas;
    this.g = canvas.getContext("2d");
    this.yaw = -0.9; this.pitch = 0.35; this.dist = 0.85;   // camera
    this.target = [0.06, 0, 0.17];                          // punto guardato [m]
    this.scenes = [];
    let drag = null;
    canvas.addEventListener("pointerdown", e => { drag = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); });
    canvas.addEventListener("pointerup", () => { drag = null; });
    canvas.addEventListener("pointermove", e => {
      if (!drag) return;
      this.yaw -= (e.clientX - drag[0]) * 0.008;
      this.pitch = Math.max(-0.2, Math.min(1.45, this.pitch + (e.clientY - drag[1]) * 0.008));
      drag = [e.clientX, e.clientY];
      this.draw();
    });
    canvas.addEventListener("wheel", e => {
      e.preventDefault();
      this.dist = Math.max(0.4, Math.min(3, this.dist * (e.deltaY > 0 ? 1.1 : 0.9)));
      this.draw();
    }, { passive: false });
    new ResizeObserver(() => this.draw()).observe(canvas);
  }

  /** scenes: [{chain (kin.chain), style: "solid" | "ghost"}] */
  set(scenes) { this.scenes = scenes; this.draw(); }

  project(p) {
    const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw), cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
    const x = p[0] - this.target[0], y = p[1] - this.target[1], z = p[2] - this.target[2];
    // camera basis: right, up, forward (looking at target from yaw/pitch)
    const xr = -sy * x + cy * y;
    const xf = cy * cp * x + sy * cp * y + sp * z;
    const xu = -cy * sp * x - sy * sp * y + cp * z;
    const depth = this.dist - xf;
    const f = this.scale / Math.max(depth, 0.05);
    return [this.w / 2 - xr * f, this.h / 2 - xu * f, depth];
  }

  line(a, b, color, width = 2, dash = null) {
    const g = this.g, p = this.project(a), q = this.project(b);
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash || []);
    g.beginPath(); g.moveTo(p[0], p[1]); g.lineTo(q[0], q[1]); g.stroke();
  }

  dot(a, color, r) {
    const p = this.project(a);
    this.g.fillStyle = color; this.g.beginPath(); this.g.arc(p[0], p[1], r, 0, 2 * Math.PI); this.g.fill();
  }

  text(a, s, color) {
    const p = this.project(a);
    this.g.fillStyle = color; this.g.font = "12px system-ui"; this.g.fillText(s, p[0] + 6, p[1] - 6);
  }

  draw() {
    const c = this.c, dpr = window.devicePixelRatio || 1;
    const w = c.clientWidth, h = c.clientHeight || 420;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
      c.width = Math.round(w * dpr); c.height = Math.round(h * dpr);
    }
    const g = this.g;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = w; this.h = h; this.scale = Math.min(w, h) * 0.9;
    g.clearRect(0, 0, w, h);
    // floor grid 10 cm, 60 x 60 cm
    for (let i = -3; i <= 3; i++) {
      this.line([i * 0.1, -0.3, 0], [i * 0.1, 0.3, 0], "#232a36", 1);
      this.line([-0.3, i * 0.1, 0], [0.3, i * 0.1, 0], "#232a36", 1);
    }
    this.line([0, 0, 0], [0.12, 0, 0], "#c04040", 2); this.text([0.12, 0, 0], "X", "#c04040");
    this.line([0, 0, 0], [0, 0.12, 0], "#40a050", 2); this.text([0, 0.12, 0], "Y", "#40a050");
    this.line([0, 0, 0], [0, 0, 0.12], "#4070d0", 2); this.text([0, 0, 0.12], "Z", "#4070d0");
    for (const s of this.scenes) if (s.chain) this.drawChain(s.chain, s.style);
  }

  drawChain(ch, style) {
    const ghost = style === "ghost";
    const link = ghost ? "rgba(240,190,80,.75)" : "#d8dee9";
    const dash = ghost ? [6, 5] : null;
    const pts = [[0, 0, 0], ...ch.joints.map(j => j.p), ch.ee];
    for (let i = 0; i + 1 < pts.length; i++) this.line(pts[i], pts[i + 1], link, ghost ? 2 : 5, dash);
    const cols = ["#6aa8ff", "#5fd08a", "#f0a050", "#c08aff", "#ff7aa8", "#7ae0e0"];
    ch.joints.forEach((j, i) => {
      if (!ghost) {
        const a = j.axis.map(v => v * 0.035);
        this.line([j.p[0] - a[0], j.p[1] - a[1], j.p[2] - a[2]], [j.p[0] + a[0], j.p[1] + a[1], j.p[2] + a[2]], cols[i], 3);
      }
      this.dot(j.p, ghost ? "rgba(240,190,80,.8)" : cols[i], ghost ? 3 : 5);
    });
    // end-effector frame (x red, y green, z blue)
    const T = ch.T, L = 0.05;
    [["#ff5050", 0], ["#50d070", 1], ["#5090ff", 2]].forEach(([col, k]) =>
      this.line(ch.ee, [ch.ee[0] + T[0][k] * L, ch.ee[1] + T[1][k] * L, ch.ee[2] + T[2][k] * L], col, ghost ? 1.5 : 2.5, dash));
    this.dot(ch.ee, ghost ? "rgba(240,190,80,.9)" : "#ffffff", ghost ? 3 : 4);
  }
}
