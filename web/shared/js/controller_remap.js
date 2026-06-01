/* controller_remap.js — Mappatura runtime button controller VR.
 *
 * Carica la config da /api/controller-mappings, espone dispatch(mode, eventKey)
 * che esegue l'action registrata corrispondente. Live-reload via WS message
 * `controller_mappings_updated` (relayed dal backend).
 *
 * Uso (viewer):
 *   controllerRemap.registerAction("toggleHudVisible", toggleHudVisible);
 *   controllerRemap.loadFromServer();
 *   // poi nel loop frame:
 *   controllerRemap.dispatch(controllerRemap.modeName(vrMode, teleopMode), "left.Y.press");
 *
 * Uso (dashboard):
 *   controllerRemap.loadFromServer().then(cfg => render(cfg));
 *   // dopo POST salvataggio: invia WS {type:"controller_mappings_updated", config}.
 *
 * Caricato come classic script: espone globale window.controllerRemap.
 */
(function () {
  "use strict";

  const TELEOP_MODE_TO_NAME = {
    0: "CALIB",
    1: "POSE",
    2: "MANUAL",
    3: "HEAD",
    4: "HYBRID",
    5: "ASSIST",
  };

  const _state = {
    config: { version: 1, modes: {} },
    actions: new Map(),  // actionName -> fn
    loaded: false,
    listeners: new Set(),  // (cfg) => void  per UI refresh
  };

  function modeName(vrMode, teleopMode) {
    // vrMode === "stereo" => modalità calibrazione (mappata su "STEREO" in config).
    if (vrMode === "stereo") return "STEREO";
    if (typeof teleopMode === "number" && TELEOP_MODE_TO_NAME[teleopMode]) {
      return TELEOP_MODE_TO_NAME[teleopMode];
    }
    return "STEREO";
  }

  function registerAction(name, fn) {
    if (typeof name !== "string" || typeof fn !== "function") return;
    _state.actions.set(name, fn);
  }

  function listActions() {
    return Array.from(_state.actions.keys()).sort();
  }

  function _runAction(action, modeStr, eventKey) {
    const fn = _state.actions.get(action);
    if (!fn) {
      console.warn("[controller_remap] unknown action:", action, "for", modeStr, eventKey);
      return false;
    }
    try { fn(); return true; }
    catch (e) { console.error("[controller_remap] action error", action, e); return false; }
  }

  // Dispatch con fallback: prova prima la chiave esatta (con modifier),
  // poi se non c'è binding prova la stessa chiave SENZA modifier. Cosi' un
  // binding generico (es. "left.Y.press") fa da default per qualunque
  // contesto modificatore non esplicitamente mappato.
  function dispatch(modeStr, eventKey) {
    if (!modeStr || !eventKey) return false;
    const mmap = _state.config.modes ? _state.config.modes[modeStr] : null;
    if (!mmap) return false;
    // Tentativo 1: chiave esatta
    let action = mmap[eventKey];
    if (action && action !== "noop") return _runAction(action, modeStr, eventKey);
    // Tentativo 2: stripping modifier (parte dopo '|')
    const sep = eventKey.indexOf("|");
    if (sep > 0) {
      const baseKey = eventKey.substring(0, sep);
      action = mmap[baseKey];
      if (action && action !== "noop") return _runAction(action, modeStr, baseKey);
    }
    return false;
  }

  async function loadFromServer() {
    try {
      const r = await fetch("/api/controller-mappings", { cache: "no-store" });
      if (!r.ok) throw new Error("HTTP " + r.status);
      const cfg = await r.json();
      applyConfig(cfg);
      _state.loaded = true;
      console.log("[controller_remap] config loaded from server, modes:", Object.keys(cfg.modes || {}));
      return cfg;
    } catch (e) {
      console.warn("[controller_remap] load failed:", e);
      return null;
    }
  }

  function applyConfig(cfg) {
    if (!cfg || typeof cfg !== "object") return;
    const modes = cfg.modes;
    if (!modes || typeof modes !== "object") return;
    _state.config = {
      version: Number(cfg.version) || 1,
      modes: modes,
    };
    // Notifica listener (es. dashboard editor che vuole ridisegnare)
    for (const fn of _state.listeners) {
      try { fn(_state.config); } catch (_) {}
    }
  }

  function getConfig() {
    // Restituisce copia per evitare mutazioni esterne accidentali.
    try { return JSON.parse(JSON.stringify(_state.config)); }
    catch (_) { return _state.config; }
  }

  function onConfigChange(fn) {
    if (typeof fn === "function") _state.listeners.add(fn);
  }

  // Hook per l'host che riceve messaggi WS: passa qui ogni `msg` parsato JSON;
  // se è un controller_mappings_updated, applica live.
  function handleWsMessage(msg) {
    if (msg && msg.type === "controller_mappings_updated" && msg.config) {
      applyConfig(msg.config);
      console.log("[controller_remap] config live-updated via WS");
    }
  }

  // POST helper per la dashboard: salva la config sul server.
  async function saveToServer(cfg) {
    const r = await fetch("/api/controller-mappings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(cfg),
    });
    if (!r.ok) {
      let detail = "";
      try { detail = (await r.json()).error || ""; } catch (_) {}
      throw new Error("POST failed: HTTP " + r.status + (detail ? " - " + detail : ""));
    }
    return r.json();
  }

  window.controllerRemap = {
    modeName,
    registerAction,
    listActions,
    dispatch,
    loadFromServer,
    applyConfig,
    getConfig,
    onConfigChange,
    handleWsMessage,
    saveToServer,
  };
})();
