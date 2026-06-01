/*
 * VR controls shared component (single source of truth)
 * Used by:
 *  - web/vr/viewer_stereo_xr.html
 *  - web/dashboard/vr.html
 */
(function () {
  const STYLE_ID = "vr-controls-shared-style";

  function ensureStyles() {
    if (document.getElementById(STYLE_ID)) return;
    const st = document.createElement("style");
    st.id = STYLE_ID;
    st.textContent = `
      .vr-controls-inline {
        display: inline-flex;
        align-items: center;
        gap: 0.45rem;
        flex-wrap: nowrap;
        white-space: nowrap;
      }
      .vr-controls-inline .btn {
        margin-right: 0;
        margin-bottom: 0;
        padding: 0.65rem 0.8rem;
        font-size: 0.95rem;
        flex: 0 0 auto;
      }
      #vr-robot-state-badge {
        display: inline-block;
        padding: 0.3rem 0.65rem;
        border-radius: 999px;
        font-size: 0.78rem;
        font-weight: 600;
        letter-spacing: 0.04em;
        text-transform: uppercase;
        border: 1.5px solid rgba(255,255,255,0.15);
        background: rgba(255,255,255,0.07);
        color: #8899aa;
        transition: background 0.25s, color 0.25s, border-color 0.25s;
        cursor: default;
        user-select: none;
        flex: 0 0 auto;
      }
      #vr-robot-state-badge.state-on   { background: rgba(39,174,96,0.22);  color: #2ecc71; border-color: rgba(46,204,113,0.5); }
      #vr-robot-state-badge.state-off  { background: rgba(192,57,43,0.22);  color: #e74c3c; border-color: rgba(231,76,60,0.5);  }
      #vr-robot-state-badge.state-warn { background: rgba(230,126,34,0.22); color: #f39c12; border-color: rgba(243,156,18,0.5); }
    `;
    document.head.appendChild(st);
  }

  function createVRControls(containerElement) {
    if (!containerElement) return null;
    if (containerElement.querySelector("#btnStart")) return containerElement;
    ensureStyles();
    const wrap = document.createElement("div");
    wrap.className = "vr-controls-inline";
    wrap.innerHTML = `
        <button class="btn primary" id="btnStart">Start</button>
        <button class="btn ghost" id="btnVR" disabled>Cam</button>
        <button class="btn ghost" id="btnTeleopPose" disabled>Pose</button>
        <button class="btn ghost" id="btnTeleop" disabled>Manual</button>
        <button class="btn ghost" id="btnTeleopHead" disabled>Head</button>
        <button class="btn ghost" id="btnTeleopHybrid" disabled>Hybrid</button>
        <button class="btn ghost" id="btnTeleopIk" disabled>Assist</button>
        <span id="vr-robot-state-badge">UNKNOWN</span>
    `;
    containerElement.insertBefore(wrap, containerElement.firstChild);
    return containerElement;
  }

  window.createVRControls = createVRControls;
})();

