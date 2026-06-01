/**
 * j5_config.js — Centralized configuration for JONNY5 dashboard & VR viewer.
 * All magic numbers and connection parameters in one place.
 */
const J5_CONFIG = Object.freeze({
  // WebSocket
  WS_PORT: 8557,
  WS_RECONNECT_BASE_MS: 400,
  WS_RECONNECT_MAX_MS: 3000,
  WS_RECONNECT_FACTOR: 1.5,

  // WebRTC camera ports (mediamtx WHEP)
  WEBRTC_CAM0_PORT: 8554,
  WEBRTC_CAM1_PORT: 8555,

  // HTTPS server port (dashboard + VR viewer)
  HTTPS_PORT: 8443,

  // UI
  EVENT_LOG_MAX_ENTRIES: 50,
  SLIDER_DEBOUNCE_MS: 50,

  // Telemetry
  FK_THROTTLE_HZ: 10,

  // Motion profiles available
  MOTION_PROFILES: ["RTR3", "RTR5", "BB", "BCB"],

  // Joint names in order
  JOINT_NAMES: ["base", "spalla", "gomito", "yaw", "pitch", "roll"],
  JOINT_LABELS: ["Base", "Spalla", "Gomito", "Yaw", "Pitch", "Roll"],
});
