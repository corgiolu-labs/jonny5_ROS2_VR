/**
 * JONNY5 Dashboard – funzioni comuni e inizializzazione
 *
 * NOTE [RPI-SAFE-REFACTOR-PHASE2]: modulo analizzato, nessuna modifica funzionale.
 * CORE: binding navbar + pulsanti SAFE/ENABLE/STOP/IMU/HOME/PARK/DEMO via window.wsSend.
 * LEGACY/UTILITY: fallback window.addLog quando j5_common.js non è caricato.
 */
import {
  addLog,
  sendCommand,
  connectJ5Dashboard,
  registerUartResponseHandler,
  registerSelfTestStatusHandler,
  registerSelfTestResultHandler,
  registerTelemetryHandler,
} from "../../shared/js/j5_common.js";
/** Inizializzazione base: attiva il link navbar della pagina corrente */
function initNavbarActive() {
  const nav = document.getElementById("navbar");
  if (!nav) return;

  const path = window.location.pathname;
  const page = path.split("/").pop() || "index.html";

  nav.querySelectorAll(".navbar a").forEach((a) => {
    const href = a.getAttribute("href") || "";
    if (href === page || (page === "" && href === "index.html")) {
      a.classList.add("active");
    } else {
      a.classList.remove("active");
    }
  });
}

/** Init eseguito al caricamento */
function init() {
  initNavbarActive();
  // Garantisce WS pronto anche se ws_dashboard.js non è ancora inizializzato.
  connectJ5Dashboard();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}

document.addEventListener("navbar-loaded", initNavbarActive);

/**************************************************
 * HOME PAGE INIT (EVENTI NAVBAR + BOTTONI)
 **************************************************/
if (document.location.pathname.endsWith("index.html") || document.location.pathname.endsWith("/") || document.location.pathname.match(/\/dashboard\/?$/)) {
  const runWhenReady = () => {
    let demoActive = false;
    let demoPending = false;
    const btnDemo = document.getElementById("btn-demo");
    const setDemoBtn = (active, pending = false) => {
      if (!btnDemo) return;
      demoActive = active;
      demoPending = pending;
      if (pending) {
        btnDemo.textContent = active ? "Demo: arresto..." : "Demo: avvio...";
      } else {
        btnDemo.textContent = active ? "Ferma Demo" : "Avvia Demo";
      }
      btnDemo.classList.toggle("active", active);
      btnDemo.disabled = pending;
    };
    setDemoBtn(false, false);

    const btnEnable = document.getElementById("btn-enable");
    const btnStop = document.getElementById("btn-stop");
    if (btnEnable) btnEnable.addEventListener("click", () => {
      // Sequenza industrial-grade: da STOPPED serve SAFE poi ENABLE. Da SAFE/IDLE SAFE è idempotente.
      sendCommand("uart", { cmd: "SAFE" });
      sendCommand("uart", { cmd: "ENABLE" });
      addLog("SAFE + ENABLE inviati");
    });
    if (btnStop) btnStop.addEventListener("click", () => {
      sendCommand("uart", { cmd: "STOP" });
      addLog("STOP inviato");
    });
    const btnImuToggle = document.getElementById("btn-imu-toggle");
    if (btnImuToggle) {
      let imuActive = false;
      const updateImuBtn = (active) => {
        imuActive = active;
        btnImuToggle.textContent = active ? "IMU ON" : "IMU OFF";
        btnImuToggle.classList.toggle("active", active);
      };
      updateImuBtn(true); /* IMU ON di default al boot */
      btnImuToggle.addEventListener("click", () => {
        const next = !imuActive;
        sendCommand("set_imu", { enabled: next });
        updateImuBtn(next);
        addLog(next ? "IMU ON inviato" : "IMU OFF inviato");
      });
      /* Sincronizza con l'ack del server (imu_enabled) dopo set_imu.
         NON usare imu_valid (stato hardware IMU) che è indipendente dall'abilitazione logica. */
      window._syncImuToggle = (enabled) => {
        if (enabled !== imuActive) updateImuBtn(enabled);
      };
    }

    const btnVrPose = document.getElementById("btn-vrpose");
    const btnHome   = document.getElementById("btn-home");
    const btnPark   = document.getElementById("btn-park");
    const btnSelfTest = document.getElementById("btn-self-test");

    if (btnVrPose) btnVrPose.addEventListener("click", () => {
      sendCommand("uart", { cmd: "TELEOPPOSE" });
      addLog("VR Pose inviato");
    });
    if (btnHome) btnHome.addEventListener("click", () => {
      if (typeof window.j5NotifyHomeRequested === "function") window.j5NotifyHomeRequested();
      sendCommand("uart", { cmd: "HOME" });
      addLog("HOME inviato");
    });
    if (btnPark) btnPark.addEventListener("click", () => {
      sendCommand("uart", { cmd: "PARK" });
      addLog("PARK inviato");
    });
    if (btnSelfTest) btnSelfTest.addEventListener("click", () => {
      if (btnSelfTest.disabled) return;
      const sent = sendCommand("self_test", { action: "run" });
      if (!sent) {
        addLog("SELF TEST non inviato (WS non pronto)");
        return;
      }
      btnSelfTest.disabled = true;
      addLog("SELF TEST avviato");
    });

    // ── Calibrazione IMU world_bias (yaw assoluto in HOME) ──────────────────
    // Replica lato browser dello script tools/calibrate_world_bias.py: invia
    // HOME, aspetta stabilizzazione, accumula campioni imu_q_raw_* dal payload
    // telemetry, media i quaternioni con sign-flip alignment, estrae il yaw
    // assoluto e invia set_imu_world_bias al backend per la persistenza.
    const btnImuCalib = document.getElementById("btn-imu-calib");
    const IMU_CALIB_HOME_SETTLE_MS = 5000;
    const IMU_CALIB_SAMPLE_MS = 20000;
    const IMU_CALIB_MIN_SAMPLES = 100;
    const imuCalibState = { running: false, samples: [], stopAt: 0, btnText: "" };

    function imuCalibSetBtn(label, disabled) {
      if (!btnImuCalib) return;
      btnImuCalib.textContent = label;
      btnImuCalib.disabled = Boolean(disabled);
    }

    function quatNorm(q) {
      const n = Math.hypot(q[0], q[1], q[2], q[3]);
      if (n <= 0) return q;
      return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
    }

    function quatMean(quats) {
      if (!quats.length) return null;
      const ref = quatNorm(quats[0]);
      const acc = [0, 0, 0, 0];
      for (const q of quats) {
        const qn = quatNorm(q);
        const dot = qn[0] * ref[0] + qn[1] * ref[1] + qn[2] * ref[2] + qn[3] * ref[3];
        const s = dot < 0 ? -1 : 1;
        acc[0] += s * qn[0]; acc[1] += s * qn[1]; acc[2] += s * qn[2]; acc[3] += s * qn[3];
      }
      const n = quats.length;
      return quatNorm([acc[0] / n, acc[1] / n, acc[2] / n, acc[3] / n]);
    }

    function quatToYawDeg(q) {
      // RPY (ZYX, intrinsic), restituisce yaw in gradi
      const [w, x, y, z] = q;
      const siny = 2.0 * (w * z + x * y);
      const cosy = 1.0 - 2.0 * (y * y + z * z);
      return Math.atan2(siny, cosy) * 180.0 / Math.PI;
    }

    function imuCalibFinalize() {
      const samples = imuCalibState.samples.slice();
      imuCalibState.samples = [];
      imuCalibState.running = false;
      if (samples.length < IMU_CALIB_MIN_SAMPLES) {
        imuCalibSetBtn(imuCalibState.btnText || "Calibra IMU", false);
        addLog(`Calibra IMU: troppo pochi campioni (${samples.length}/${IMU_CALIB_MIN_SAMPLES})`);
        return;
      }
      const avg = quatMean(samples);
      if (!avg) {
        imuCalibSetBtn(imuCalibState.btnText || "Calibra IMU", false);
        addLog("Calibra IMU: media quaternionica fallita");
        return;
      }
      const yawDeg = quatToYawDeg(avg);
      const yawRad = (yawDeg * Math.PI) / 180.0;
      const wbQuat = [Math.cos(yawRad / 2.0), 0.0, 0.0, Math.sin(yawRad / 2.0)];
      const durationS = IMU_CALIB_SAMPLE_MS / 1000.0;
      const rateHz = samples.length / durationS;
      imuCalibSetBtn("Salvataggio…", true);
      const sent = sendCommand("set_imu_world_bias", {
        quat_wxyz: wbQuat,
        rpy_deg: [0.0, 0.0, yawDeg],
        samples: samples.length,
        duration_s: durationS,
        rate_hz_target: 30.0,
        rate_hz_measured: rateHz,
        source: "dashboard_home",
      });
      if (!sent) {
        imuCalibSetBtn(imuCalibState.btnText || "Calibra IMU", false);
        addLog("Calibra IMU: WS non pronto, salvataggio fallito");
        return;
      }
      addLog(`Calibra IMU: ${samples.length} campioni, yaw=${yawDeg.toFixed(2)}°, salvataggio in corso…`);
      // Reset del bottone allo stato idle dopo 5 s (anche se il backend non
      // risponde; in caso di errore l'utente può ricliccare).
      setTimeout(() => {
        imuCalibSetBtn(imuCalibState.btnText || "Calibra IMU", false);
      }, 5000);
    }

    if (btnImuCalib) {
      imuCalibState.btnText = btnImuCalib.textContent;
      btnImuCalib.addEventListener("click", () => {
        if (imuCalibState.running) return;
        if (!confirm("Avviare la calibrazione IMU world_bias?\n\nIl robot verrà mandato in HOME e l'IMU campionata per ~20 secondi.\nAssicurarsi che nessuno tocchi il robot.")) {
          return;
        }
        imuCalibState.running = true;
        imuCalibState.samples = [];
        imuCalibSetBtn("HOME…", true);
        const sent = sendCommand("uart", { cmd: "HOME" });
        if (!sent) {
          imuCalibState.running = false;
          imuCalibSetBtn(imuCalibState.btnText || "Calibra IMU", false);
          addLog("Calibra IMU: HOME non inviato (WS non pronto)");
          return;
        }
        addLog("Calibra IMU: HOME inviato, attesa stabilizzazione 5 s…");
        setTimeout(() => {
          if (!imuCalibState.running) return;
          imuCalibState.stopAt = Date.now() + IMU_CALIB_SAMPLE_MS;
          imuCalibSetBtn("Campionamento…", true);
          addLog(`Calibra IMU: campionamento ${IMU_CALIB_SAMPLE_MS / 1000} s…`);
          const progressInterval = setInterval(() => {
            if (!imuCalibState.running) {
              clearInterval(progressInterval);
              return;
            }
            const remaining = Math.max(0, Math.round((imuCalibState.stopAt - Date.now()) / 1000));
            imuCalibSetBtn(`Campionamento ${imuCalibState.samples.length} (${remaining}s)…`, true);
            if (remaining <= 0) {
              clearInterval(progressInterval);
              imuCalibFinalize();
            }
          }, 500);
        }, IMU_CALIB_HOME_SETTLE_MS);
      });
    }

    registerTelemetryHandler((msg) => {
      if (!imuCalibState.running) return;
      if (Date.now() < imuCalibState.stopAt - IMU_CALIB_SAMPLE_MS) return; // ancora in fase HOME
      if (Date.now() > imuCalibState.stopAt) return; // oltre la finestra
      const qw = msg?.imu_q_raw_w, qx = msg?.imu_q_raw_x, qy = msg?.imu_q_raw_y, qz = msg?.imu_q_raw_z;
      if (qw == null || qx == null || qy == null || qz == null) return;
      imuCalibState.samples.push([+qw, +qx, +qy, +qz]);
    });
    if (btnDemo) btnDemo.addEventListener("click", () => {
      if (demoPending) return;
      const sent = sendCommand("uart", { cmd: "DEMO" });
      if (!sent) {
        addLog("DEMO non inviata (WS non pronto)");
        return;
      }
      setDemoBtn(demoActive, true);
      addLog(demoActive ? "Richiesta stop DEMO inviata" : "Richiesta avvio DEMO inviata");
    });

    registerUartResponseHandler((msg) => {
      const cmd = String(msg?.cmd || "").toUpperCase();
      if (cmd === "HOME" && msg.ok === false && typeof window.j5CancelHomeImuCapture === "function") {
        window.j5CancelHomeImuCapture();
      }
      if (cmd === "STOP" && msg.ok) {
        setDemoBtn(false, false);
        return;
      }
      if (cmd !== "DEMO") return;
      if (msg.ok && String(msg.response || "").includes("STARTED")) {
        setDemoBtn(true, false);
        addLog("DEMO avviata");
        return;
      }
      if (msg.ok && String(msg.response || "").includes("STOPPED")) {
        setDemoBtn(false, false);
        addLog("DEMO fermata senza STOP");
        return;
      }
      setDemoBtn(demoActive, false);
      addLog(`DEMO fallita: ${msg.response || "errore sconosciuto"}`);
    });

    registerSelfTestStatusHandler((msg) => {
      if (!btnSelfTest) return;
      btnSelfTest.disabled = Boolean(msg?.running);
      if (msg?.message) addLog(`SELF TEST: ${msg.message}`);
    });

    registerSelfTestResultHandler((msg) => {
      if (btnSelfTest) btnSelfTest.disabled = false;
      addLog(`SELF TEST RESULT: ${msg?.result || "UNKNOWN"}`);
    });
  };
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", runWhenReady);
  else runWhenReady();
}

// LEGACY/UTILITY: helper addLog ora centralizzato in j5_common.js.
// Il fallback locale precedente è stato rimosso in RPI-3 per ridurre duplicazione.
