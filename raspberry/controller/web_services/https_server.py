import argparse
import http.server
import socket
import ssl
import os
import json
import urllib.request
import urllib.error
from urllib.parse import urlparse, parse_qs
from functools import partial

from controller.web_services.vr_config_defaults import get_vr_config_defaults, merge_vr_config_with_defaults
from controller.web_services import runtime_config_paths as rcfg

# Root static: web/ (vr/, dashboard/, shared/)
_WEB_SERVICES_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROLLER_DIR = os.path.dirname(_WEB_SERVICES_DIR)
# Default: raspberry5/web (sibling of controller/)
_DEFAULT_STATIC_ROOT = os.path.join(os.path.dirname(_CONTROLLER_DIR), "web")
# TLS: prefer config_runtime/tls, fallback legacy controller/certs during migration.

# Path legacy -> path nuovo (compatibilità vecchi bookmark / link)
_LEGACY_VR_PATHS = {
    "/viewer_stereo_xr.html": "/vr/viewer_stereo_xr.html",
    "/viewer_webrtc.html": "/vr/viewer_stereo_xr.html",
}


def _portal_https_base() -> str:
    return os.environ.get("CAPTIVE_PORTAL_PUBLIC_HTTPS", "https://10.42.0.1").rstrip("/")


def _portal_open_url() -> str:
    return f"{_portal_https_base()}/captive-portal/open"


def _portal_viewer_url() -> str:
    return os.environ.get(
        "CAPTIVE_PORTAL_TARGET_HTTPS",
        f"{_portal_https_base()}/vr/viewer_stereo_xr.html",
    )


def _resolve_tls_pair() -> tuple[str, str, str, str]:
    requested_cert = os.path.abspath(
        os.environ.get("HTTPS_CERT_FILE", rcfg.get_runtime_config_path("tls_cert"))
    )
    requested_key = os.path.abspath(
        os.environ.get("HTTPS_KEY_FILE", rcfg.get_runtime_config_path("tls_key"))
    )
    resolved_cert = rcfg.resolve_existing_config_path("tls_cert", env_var="HTTPS_CERT_FILE")
    resolved_key = rcfg.resolve_existing_config_path("tls_key", env_var="HTTPS_KEY_FILE")
    return requested_cert, requested_key, resolved_cert, resolved_key


class J5HTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Handler che risponde anche ai path legacy (senza /vr/) e alle API /api/."""

    def translate_path(self, path):
        path_clean = path.split("?")[0].split("#")[0].rstrip("/") or "/"
        if path_clean in _LEGACY_VR_PATHS:
            path = _LEGACY_VR_PATHS[path_clean] + (
                "?" + path.split("?", 1)[1] if "?" in path else ""
            )
        return super().translate_path(path)

    def _handle_ws_proxy(self):
        """Proxy a WebSocket upgrade request from the browser to the local WS server (127.0.0.1:8557).

        This allows the browser to connect via wss://<host>/ws using the same TLS certificate and
        origin it already trusts for HTTPS — no separate cert acceptance needed for port 8557.
        """
        import base64 as _b64
        import hashlib as _hashlib
        import socket as _socket
        import ssl as _ssl
        import threading as _threading

        self.close_connection = True

        ws_key = self.headers.get("Sec-WebSocket-Key", "")
        if not ws_key:
            self.send_error(400, "Missing Sec-WebSocket-Key")
            return
        ws_accept = _b64.b64encode(
            _hashlib.sha1(
                (ws_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
            ).digest()
        ).decode("ascii")
        # WebSocket upgrade requires an HTTP/1.1 101 response. BaseHTTPRequestHandler
        # defaults to HTTP/1.0, which some clients reject during the handshake.
        self.connection.sendall(
            (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {ws_accept}\r\n"
                "\r\n"
            ).encode("ascii")
        )

        # Connect to local WSS server (skip cert verification — loopback only)
        ctx = _ssl.SSLContext(_ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = _ssl.CERT_NONE
        try:
            raw = _socket.create_connection(("127.0.0.1", 8557), timeout=5)
            remote = ctx.wrap_socket(raw, server_hostname="127.0.0.1")
        except Exception as e:
            self.log_message("ws_proxy: cannot connect to local WS server: %s", str(e))
            return

        # Perform WebSocket upgrade handshake with local server
        nonce = _b64.b64encode(b"J5WsProxyKey0000").decode("ascii")  # 16 bytes
        try:
            remote.sendall((
                "GET / HTTP/1.1\r\nHost: 127.0.0.1:8557\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {nonce}\r\nSec-WebSocket-Version: 13\r\n\r\n"
            ).encode("ascii"))
            buf = b""
            while b"\r\n\r\n" not in buf:
                chunk = remote.recv(4096)
                if not chunk:
                    remote.close()
                    return
                buf += chunk
        except Exception as e:
            self.log_message("ws_proxy: upstream handshake error: %s", str(e))
            try:
                remote.close()
            except Exception:
                pass
            return

        # Bidirectional raw byte relay (WebSocket frames)
        browser = self.connection

        def _pipe(src, dst):
            try:
                while True:
                    data = src.recv(65536)
                    if not data:
                        break
                    dst.sendall(data)
            except Exception:
                pass
            finally:
                try:
                    src.shutdown(_socket.SHUT_RD)
                except Exception:
                    pass

        t = _threading.Thread(target=_pipe, args=(remote, browser), daemon=True)
        t.start()
        _pipe(browser, remote)
        t.join(timeout=5)
        try:
            remote.close()
        except Exception:
            pass

    def do_GET(self):
        path_clean = self.path.split("?")[0].split("#")[0]
        # WebSocket proxy: browser connects to wss://<host>/ws (same cert/origin as HTTPS page)
        if path_clean == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
            self._handle_ws_proxy()
            return
        if path_clean == "/api/routing-config":
            self._handle_get_routing_config()
            return
        if path_clean == "/api/controller-mappings":
            self._handle_get_controller_mappings()
            return
        if path_clean == "/api/webrtc-calibration":
            self._handle_get_webrtc_calibration()
            return
        if path_clean == "/api/vr-config-defaults":
            self._handle_get_vr_config_defaults()
            return
        if path_clean == "/api/video-config":
            self._handle_get_video_config()
            return
        if path_clean == "/api/imu-frame-calib":
            self._handle_get_imu_frame_calib()
            return
        if path_clean == "/api/mjpeg-fullstack":
            self._handle_mjpeg_fullstack()
            return
        super().do_GET()

    def do_POST(self):
        path_clean = self.path.split("?")[0].split("#")[0]
        if path_clean == "/api/routing-config":
            self._handle_post_routing_config()
            return
        if path_clean == "/api/controller-mappings":
            self._handle_post_controller_mappings()
            return
        if path_clean == "/api/refocus-cameras":
            self._handle_post_refocus_cameras()
            return
        if path_clean == "/api/webrtc-whep":
            self._handle_webrtc_whep_proxy()
            return
        if path_clean == "/api/imu-home-ref":
            self._handle_post_imu_home_ref()
            return
        if path_clean == "/api/imu-home-ref/clear":
            self._handle_delete_imu_home_ref()
            return
        self.send_error(404, "Not found")

    def _handle_get_routing_config(self):
        """Restituisce la configurazione routing salvata, o 404 se non esiste."""
        try:
            cfg = rcfg.load_routing_config_strict()
            body = json.dumps(cfg).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))

    def _handle_get_imu_frame_calib(self):
        """Return IMU-to-EE mount and BNO085 world_bias quaternions for the
        compare/observability layer on the dashboard.

        Same JSON files the `imu_analytics` validators consume; exposed here
        read-only so the FK/IK compare window can express the IMU estimate
        in robot base frame via:
            R_ee = R_world_bias^-1 · R_imu · R_mount^-1
        Missing or malformed configs fall back to identity quaternions,
        preserving the previous raw-IMU behavior (backward compatible).
        """
        def _normalize(cfg):
            """Extract quat_wxyz from {quat_wxyz:[4]} or {rpy_deg:[3]}; identity otherwise."""
            identity = [1.0, 0.0, 0.0, 0.0]
            if not isinstance(cfg, dict):
                return identity
            q = cfg.get("quat_wxyz")
            if isinstance(q, list) and len(q) == 4:
                try:
                    return [float(v) for v in q]
                except (TypeError, ValueError):
                    return identity
            rpy = cfg.get("rpy_deg")
            if isinstance(rpy, list) and len(rpy) == 3:
                try:
                    import math
                    roll, pitch, yaw = [math.radians(float(v)) for v in rpy]
                    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
                    return [
                        cr * cp * cy + sr * sp * sy,
                        sr * cp * cy - cr * sp * sy,
                        cr * sp * cy + sr * cp * sy,
                        cr * cp * sy - sr * sp * cy,
                    ]
                except (TypeError, ValueError):
                    return identity
            return identity

        try:
            mount_cfg = rcfg.load_runtime_json("imu_ee_mount", default=None)
        except Exception:
            mount_cfg = None
        try:
            world_bias_cfg = rcfg.load_runtime_json("imu_world_bias", default=None)
        except Exception:
            world_bias_cfg = None
        try:
            home_ref_cfg = rcfg.load_runtime_json("imu_home_ref", default=None)
        except Exception:
            home_ref_cfg = None

        payload = {
            "mount": {
                "quat_wxyz": _normalize(mount_cfg),
                "present": isinstance(mount_cfg, dict),
            },
            "world_bias": {
                "quat_wxyz": _normalize(world_bias_cfg),
                "present": isinstance(world_bias_cfg, dict),
            },
            "home": {
                # "Zero at HOME" offset capturato via POST /api/imu-home-ref.
                # Optional: se assente → identity (nessuna correzione aggiuntiva).
                "quat_wxyz": _normalize(home_ref_cfg),
                "present": isinstance(home_ref_cfg, dict),
                "calibrated_at": home_ref_cfg.get("calibrated_at") if isinstance(home_ref_cfg, dict) else None,
            },
        }
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _handle_post_imu_home_ref(self):
        """Save IMU "zero at HOME" quaternion to config_runtime/imu/imu_home_ref.json.

        Scope: compare/observability layer only. This file is consumed only by the
        FK/IK dashboard to cancel a residual yaw drift in the BNO085 magnetometer-
        derived Rotation Vector; it is NOT consulted by any teleop / VR / SPI /
        firmware operational path.

        Expected payload: {"quat_wxyz": [w,x,y,z], "calibrated_at": "...", "fk_pose_mm": [...]}.
        Only quat_wxyz is mandatory. Everything else is informational metadata.
        """
        try:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b""
            try:
                payload = json.loads(raw.decode("utf-8")) if raw else {}
            except Exception as e:
                raise RuntimeError(f"JSON non valido: {e}")
            rcfg.validate_imu_home_ref_shape(payload)

            # Sanitize: keep only well-known fields to avoid storing stray data.
            quat_wxyz = [float(v) for v in payload["quat_wxyz"]]
            record = {"quat_wxyz": quat_wxyz}
            if isinstance(payload.get("calibrated_at"), str):
                record["calibrated_at"] = payload["calibrated_at"]
            else:
                import datetime
                record["calibrated_at"] = datetime.datetime.now(
                    tz=datetime.timezone.utc
                ).isoformat(timespec="seconds").replace("+00:00", "Z")
            if isinstance(payload.get("fk_pose_mm"), list) and len(payload["fk_pose_mm"]) == 6:
                try:
                    record["fk_pose_mm"] = [float(v) for v in payload["fk_pose_mm"]]
                except (TypeError, ValueError):
                    pass
            if isinstance(payload.get("joints_virtual_deg"), list) and len(payload["joints_virtual_deg"]) == 6:
                try:
                    record["joints_virtual_deg"] = [float(v) for v in payload["joints_virtual_deg"]]
                except (TypeError, ValueError):
                    pass
            if isinstance(payload.get("note"), str) and len(payload["note"]) < 200:
                record["note"] = payload["note"]

            path = rcfg.get_runtime_config_write_path("imu_home_ref")
            rcfg.ensure_parent_dir(path)
            rcfg._write_text_atomic(path, json.dumps(record, indent=2) + "\n")

            body = json.dumps({"ok": True, "path": path, "record": record}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            body = json.dumps({"ok": False, "error": str(e)}).encode("utf-8")
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

    def _handle_delete_imu_home_ref(self):
        """Remove imu_home_ref.json (reset Zero-at-HOME correction)."""
        try:
            import os as _os
            path = rcfg.get_runtime_config_write_path("imu_home_ref")
            removed = False
            if _os.path.isfile(path):
                _os.remove(path)
                removed = True
            body = json.dumps({"ok": True, "removed": removed, "path": path}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            body = json.dumps({"ok": False, "error": str(e)}).encode("utf-8")
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

    def _handle_get_vr_config_defaults(self):
        """Restituisce i default autorevoli backend per routing/tuning VR-IMU."""
        body = json.dumps(get_vr_config_defaults()).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _handle_get_captive_portal_api(self):
        """
        Android Captive Portal API (RFC 8908 / Android 11+).
        Best-effort: se il client non accetta il certificato locale, ricadrà
        comunque sui probe HTTP legacy che intercettiamo sul server captive HTTP.
        """
        ip = self.client_address[0] if self.client_address else "0.0.0.0"
        payload = (
            {"captive": False}
            if captive_portal_auth.is_authenticated(ip)
            else {
                "captive": True,
                "user-portal-url": _portal_open_url(),
            }
        )
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/captive+json")
        self.send_header("Cache-Control", "private, no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_get_captive_portal_touch(self):
        ip = self.client_address[0] if self.client_address else "0.0.0.0"
        captive_portal_auth.mark_authenticated(ip)
        self.send_response(204)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

    def _handle_get_captive_portal_open(self):
        target_url = _portal_viewer_url()
        html = f"""<!doctype html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>JONNY5 Portal</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #08131b;
      --panel: #102537;
      --ink: #edf6ff;
      --muted: #a9c4dd;
      --accent: #41a4ff;
      --ok: #39c56f;
    }}
    body {{
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background:
        radial-gradient(circle at top, rgba(65,164,255,.24), transparent 34%),
        linear-gradient(180deg, #071019 0%, var(--bg) 100%);
      color: var(--ink);
      font-family: "Segoe UI", system-ui, sans-serif;
    }}
    .card {{
      width: min(92vw, 560px);
      padding: 28px;
      border-radius: 24px;
      background: rgba(16,37,55,.94);
      border: 1px solid rgba(132,176,214,.22);
      box-shadow: 0 18px 60px rgba(0,0,0,.35);
    }}
    h1 {{
      margin: 0 0 12px;
      font-size: 2rem;
    }}
    p {{
      margin: 0 0 16px;
      line-height: 1.5;
      color: var(--muted);
    }}
    .actions {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 20px;
    }}
    a {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-height: 52px;
      padding: 0 18px;
      border-radius: 14px;
      text-decoration: none;
      font-weight: 700;
    }}
    .primary {{
      background: var(--accent);
      color: white;
    }}
    .secondary {{
      border: 1px solid rgba(132,176,214,.28);
      color: var(--ink);
    }}
    .note {{
      margin-top: 18px;
      color: #8fe2ad;
      font-size: .95rem;
    }}
    .status {{
      margin-top: 16px;
      color: var(--muted);
      font-size: .95rem;
    }}
  </style>
</head>
<body>
  <main class="card">
    <h1>JONNY5 VR</h1>
    <p>Hotspot robot connesso. Questa pagina prepara l'apertura del viewer VR in modo piu tollerante per il browser captive del Quest.</p>
    <p>Se compare un avviso certificato locale, conferma e prosegui verso il viewer.</p>
    <div class="actions">
      <a id="open-viewer" class="primary" href="{target_url}" target="_blank" rel="noopener">Apri Teleoperazione VR</a>
      <a class="secondary" href="{_portal_https_base()}/dashboard/index.html" target="_blank" rel="noopener">Apri Dashboard</a>
    </div>
    <div id="portal-status" class="status">Controllo accesso locale in corso...</div>
    <div class="note">Questa pagina resta intenzionalmente leggera: quando lo stato diventa verde, tocca il pulsante blu per aprire il viewer completo.</div>
    <div class="note">URL locale viewer: {target_url}</div>
  </main>
  <script>
    (function () {{
      const target = {target_url!r};
      const statusEl = document.getElementById('portal-status');
      const openViewerBtn = document.getElementById('open-viewer');
      async function touch() {{
        try {{
          await fetch('/captive-portal/touch', {{ cache: 'no-store', credentials: 'same-origin' }});
        }} catch (_) {{}}
      }}
      async function bootstrap() {{
        await touch();
        try {{
          const res = await fetch('/api/video-config', {{ cache: 'no-store', credentials: 'same-origin' }});
          if (!res.ok) throw new Error('video-config HTTP ' + res.status);
          statusEl.textContent = 'Viewer locale raggiungibile. Tocca "Apri Teleoperazione VR".';
          statusEl.style.color = '#8fe2ad';
        }} catch (e) {{
          statusEl.textContent = 'Viewer locale raggiungibile in modo parziale. Puoi comunque provare ad aprirlo dal pulsante blu.';
          statusEl.style.color = '#f7d26a';
        }}
      }}
      setTimeout(bootstrap, 250);
      setInterval(touch, 2000);
      openViewerBtn.addEventListener('click', function () {{
        statusEl.textContent = 'Apertura manuale del viewer...';
        statusEl.style.color = '#8fe2ad';
      }});
      document.addEventListener('visibilitychange', function () {{
        if (document.visibilityState === 'visible') {{
          touch();
        }}
      }});
    }})();
  </script>
</body>
</html>
"""
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_get_webrtc_calibration(self):
        """Restituisce la calibrazione WebRTC dal path runtime ufficiale."""
        calib_path = rcfg.get_runtime_config_path("webrtc_calibration")
        if not os.path.exists(calib_path):
            self.send_response(404)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"error":"no webrtc calibration"}')
            return
        try:
            with open(calib_path, "r", encoding="utf-8") as f:
                data = f.read()
            body = data.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            self.send_error(500, str(e))

    def _handle_get_video_config(self):
        """Restituisce la configurazione video (solo pipeline low-latency: MediaMTX + WHEP)."""
        runtime_path = rcfg.get_runtime_config_path("video_pipeline")
        if not os.path.exists(runtime_path):
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"error": f"missing required runtime config: {runtime_path}"}).encode("utf-8"))
            return

        cfg = rcfg.load_runtime_yaml("video_pipeline", default=None)
        if not isinstance(cfg, dict):
            body = json.dumps(
                {
                    "error": "invalid or unreadable video_pipeline.yaml",
                    "video_pipeline": None,
                }
            ).encode("utf-8")
            self.send_response(503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return

        val = str(cfg.get("video_pipeline", "")).strip().lower()
        if val == "webrtc":
            out = {"video_pipeline": "webrtc"}
            body = json.dumps(out).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return

        if val == "mjpeg":
            msg = "MJPEG pipeline is disabled on this server; use MediaMTX + WHEP (video_pipeline: webrtc)."
        else:
            msg = f"unsupported video_pipeline value: {val!r}; expected webrtc."
        body = json.dumps({"error": msg, "video_pipeline": None}).encode("utf-8")
        self.send_response(503)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _handle_mjpeg_fullstack(self):
        """Streaming MJPEG full-stack per benchmark a parita' di condizioni con MediaMTX.

        GET /api/mjpeg-fullstack?width=W&height=H&fps=F&target_frames=N&label=NAME

        Riproduce la pipeline MJPEG di produzione storica:
            rpicam-vid (encoder JPEG HW) -> stdout pipe -> Python HTTP server ->
            multipart/x-mixed-replace -> consumer (browser fetch + ReadableStream)

        Permette confronto a parita' di condizioni con la pipeline MediaMTX/WebRTC
        (entrambi attivi sul Pi mentre servono un consumer remoto via rete TLS).

        Ciclo:
          1. STOP jonny5-mediamtx (libera camera CSI)
          2. Spawn rpicam-vid MJPEG -> stdout
          3. Estrai JPEG completi (marker SOI 0xFFD8 .. EOI 0xFFD9) e inviali
             come multipart parts al client fino a target_frames o disconnect
          4. Termina rpicam-vid
          5. START jonny5-mediamtx (ripristino pipeline VR)
        """
        import subprocess as _sp
        import time as _t

        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        try:
            width  = int((qs.get("width")  or ["1280"])[0])
            height = int((qs.get("height") or ["720"])[0])
            fps    = int((qs.get("fps")    or ["30"])[0])
            target_frames = int((qs.get("target_frames") or ["300"])[0])
            label  = (qs.get("label") or ["mjpeg_fullstack"])[0]
        except (TypeError, ValueError):
            self.send_error(400, "Invalid query parameters")
            return
        target_frames = max(20, min(1000, target_frames))
        # Sanity sui parametri camera
        width  = max(160, min(4608, width))
        height = max(120, min(2592, height))
        fps    = max(1,   min(120,  fps))

        # Stop MediaMTX (libera camera CSI). sudo NOPASSWD configurato.
        try:
            _sp.run(["sudo", "-n", "/bin/systemctl", "stop", "jonny5-mediamtx.service"],
                    check=False, timeout=5.0)
        except Exception as e:
            try:
                self.send_error(500, f"Cannot stop MediaMTX: {e}")
            except Exception:
                pass
            return
        _t.sleep(1.5)  # grace per rilascio camera

        # Calcolo timeout rpicam-vid: tempo necessario a target_frames @ fps + 3s margine.
        # --timeout 0 in alcune versioni significa "esci immediatamente", evitare.
        needed_ms = max(5000, int((target_frames * 1.4 / fps) * 1000) + 3000)
        rpicam_timeout_ms = min(needed_ms, 90000)  # cap 90s

        proc = None
        try:
            proc = _sp.Popen(
                ["rpicam-vid",
                 "--codec", "mjpeg",
                 "--width", str(width),
                 "--height", str(height),
                 "--framerate", str(fps),
                 "--timeout", str(rpicam_timeout_ms),
                 "--inline", "1",
                 "--nopreview",
                 "--output", "-"],
                stdout=_sp.PIPE, stderr=_sp.DEVNULL, bufsize=0,
            )

            # Headers multipart streaming
            boundary = "jonny5frame"
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            buf = b""
            SOI = b"\xff\xd8"
            EOI = b"\xff\xd9"
            frames_sent = 0
            deadline = _t.monotonic() + 90.0  # safety timeout
            client_alive = True

            while frames_sent < target_frames and _t.monotonic() < deadline and client_alive:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    soi_idx = buf.find(SOI)
                    if soi_idx < 0:
                        break
                    eoi_idx = buf.find(EOI, soi_idx + 2)
                    if eoi_idx < 0:
                        break
                    jpeg = buf[soi_idx:eoi_idx + 2]
                    buf = buf[eoi_idx + 2:]
                    part_header = (
                        f"\r\n--{boundary}\r\n"
                        f"Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n"
                        f"X-Frame-Index: {frames_sent}\r\n\r\n"
                    ).encode("ascii")
                    try:
                        self.wfile.write(part_header)
                        self.wfile.write(jpeg)
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        client_alive = False
                        break
                    frames_sent += 1
                    if frames_sent >= target_frames:
                        break
                # Nota: NON tronchiamo buf qui — per profili 4K un singolo JPEG
                # puo' essere > 64KB e attraversare piu' chunk, dobbiamo accumulare
                # finche' non troviamo un EOI completo. Il `buf = buf[eoi_idx + 2:]`
                # nel loop interno mantiene buf piccolo nei casi a bassa risoluzione.

            # Final boundary (delimita la fine dello streaming multipart)
            if client_alive:
                try:
                    self.wfile.write(f"\r\n--{boundary}--\r\n".encode("ascii"))
                    self.wfile.flush()
                except Exception:
                    pass
            try:
                self.log_message("mjpeg-fullstack [%s] %dx%d@%d done: %d/%d frames",
                                 label, width, height, fps, frames_sent, target_frames)
            except Exception:
                pass
        except Exception as e:
            try:
                self.log_error(f"mjpeg-fullstack runtime error: {e}")
            except Exception:
                pass
        finally:
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=2.0)
                except Exception:
                    try: proc.kill()
                    except Exception: pass
            # Restart MediaMTX (ripristina pipeline VR)
            try:
                _sp.run(["sudo", "-n", "/bin/systemctl", "start", "jonny5-mediamtx.service"],
                        check=False, timeout=5.0)
            except Exception:
                pass

    def _handle_webrtc_whep_proxy(self):
        """Proxy WHEP POST to MediaMTX (127.0.0.1:8889) per evitare mixed content nel viewer VR."""
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        path = (qs.get("path") or [None])[0]
        if path not in ("cam0", "cam1"):
            self.send_response(400)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"Missing or invalid query: path=cam0 or path=cam1")
            return
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0:
            self.send_response(400)
            self.end_headers()
            return
        body = self.rfile.read(length)
        mediamtx_url = f"http://127.0.0.1:8889/{path}/whep"
        req = urllib.request.Request(
            mediamtx_url,
            data=body,
            method="POST",
            headers={"Content-Type": self.headers.get("Content-Type", "application/sdp")},
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                status = resp.getcode()
                answer = resp.read()
                self.send_response(status)
                self.send_header("Content-Type", resp.headers.get("Content-Type", "application/sdp"))
                self.send_header("Content-Length", str(len(answer)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(answer)
        except urllib.error.HTTPError as e:
            self.send_response(e.code)
            self.send_header("Content-Type", "text/plain")
            try:
                body_err = e.read()
            except Exception:
                body_err = str(e).encode("utf-8")
            self.send_header("Content-Length", str(len(body_err)))
            self.end_headers()
            self.wfile.write(body_err)
        except Exception as e:
            self.send_response(502)
            self.send_header("Content-Type", "text/plain")
            msg = str(e).encode("utf-8")
            self.send_header("Content-Length", str(len(msg)))
            self.end_headers()
            self.wfile.write(msg)

    def _handle_post_routing_config(self):
        """Salva/persist la configurazione routing ricevuta come JSON."""
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)
        try:
            cfg = json.loads(raw.decode("utf-8"))
            # Leggi file esistente (se c'è) e fai merge: così un salvataggio parziale
            # (solo limits o solo tuning) non cancella routing già salvato e viceversa.
            existing = rcfg.load_routing_config_strict() or {}
            merged = merge_vr_config_with_defaults({**existing, **cfg})
            rcfg.validate_routing_config_shape(merged)
            if not rcfg.save_runtime_json("routing_config", merged, mirror_legacy=False):
                raise RuntimeError("unable to persist routing_config")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        except Exception as e:
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(e)}).encode())

    def _handle_get_controller_mappings(self):
        """Restituisce la mappatura runtime dei controller (mode -> event -> action)."""
        try:
            cfg = rcfg.load_runtime_json("controller_mappings", default=None)
            if not isinstance(cfg, dict):
                cfg = {"version": 1, "modes": {}}
            body = json.dumps(cfg).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))

    def _handle_post_controller_mappings(self):
        """Salva la mappatura runtime dei controller. La pagina dashboard, dopo questo POST,
        invia anche {type:"controller_mappings_updated", config:...} via WS per il broadcast
        live a tutti gli altri client (viewer XR incluso)."""
        length = int(self.headers.get("Content-Length", 0))
        try:
            raw = self.rfile.read(length) if length > 0 else b""
            cfg = json.loads(raw.decode("utf-8")) if raw else {}
            # Validazione minima: oggetto con 'modes' (dict di dict) e version int.
            if not isinstance(cfg, dict):
                raise RuntimeError("payload deve essere oggetto JSON")
            if "modes" not in cfg or not isinstance(cfg["modes"], dict):
                raise RuntimeError("campo 'modes' mancante o non oggetto")
            for mname, mmap in cfg["modes"].items():
                if not isinstance(mmap, dict):
                    raise RuntimeError(f"modes.{mname} non oggetto")
                for evt, act in mmap.items():
                    if not isinstance(evt, str) or not isinstance(act, str):
                        raise RuntimeError(f"modes.{mname}: chiave/valore non stringa")
            cfg.setdefault("version", 1)
            if not rcfg.save_runtime_json("controller_mappings", cfg, mirror_legacy=False):
                raise RuntimeError("unable to persist controller_mappings")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        except Exception as e:
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(e)}).encode())

    def _handle_post_refocus_cameras(self):
        """Trigger refocus hardware AF: lancia jonny5-refocus-cams.service che restarta
        MediaMTX e fa re-init libcamera AF su entrambe le cam. Debounce server-side 3s
        per evitare burst di restart su click multipli rapidi."""
        import subprocess
        import time as _time
        cls = type(self)
        now = _time.monotonic()
        last = getattr(cls, "_last_refocus_ts", 0.0)
        if (now - last) < 3.0:
            self.send_response(429)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"ok":false,"error":"cooldown 3s"}')
            return
        cls._last_refocus_ts = now
        try:
            subprocess.run(
                ["sudo", "-n", "/usr/bin/systemctl", "start", "jonny5-refocus-cams.service"],
                check=True, timeout=5, capture_output=True,
            )
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        except subprocess.CalledProcessError as e:
            err = (e.stderr or b"").decode("utf-8", "replace") or str(e)
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": err}).encode())
        except Exception as e:
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": str(e)}).encode())

    def do_OPTIONS(self):
        """CORS preflight."""
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def log_message(self, fmt, *args):
        # Silenzia log HTTP per richieste API frequenti
        first_arg = args[0] if args else ""
        first_arg = first_arg if isinstance(first_arg, str) else str(first_arg)
        if "/api/" not in first_arg:
            super().log_message(fmt, *args)


def _make_https_server(host, port, handler):
    """ThreadingHTTPServer con dual-stack IPv6 quando l'host contiene ':' (es. '::').
    IPV6_V6ONLY=0 lascia accettare anche connessioni IPv4 mappate (10.42.0.1 ecc.)."""
    if ":" in host:
        class _IPv6HTTPServer(http.server.ThreadingHTTPServer):
            address_family = socket.AF_INET6
            allow_reuse_address = True

            def server_bind(self):
                try:
                    self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
                except (OSError, AttributeError):
                    pass
                super().server_bind()
        return _IPv6HTTPServer((host, port), handler)
    return http.server.ThreadingHTTPServer((host, port), handler)


def main():
    parser = argparse.ArgumentParser(description="JONNY5 HTTPS server")
    parser.add_argument("--root", type=str, default=None, help="Root directory for static files (e.g. .../web)")
    args = parser.parse_args()

    host = os.environ.get("HTTPS_BIND", "0.0.0.0")
    port = int(os.environ.get("HTTPS_PORT", "8443"))
    directory = os.environ.get("HTTPS_DIR") or args.root or _DEFAULT_STATIC_ROOT
    directory = os.path.abspath(directory)
    requested_cert, requested_key, cert_file, key_file = _resolve_tls_pair()

    handler = partial(J5HTTPRequestHandler, directory=directory)
    httpd = _make_https_server(host, port, handler)

    if not os.path.isfile(cert_file) or not os.path.isfile(key_file):
        raise SystemExit(
            "TLS files not found. Provision config_runtime/tls/webrtc.crt and "
            "config_runtime/tls/webrtc.key, or set HTTPS_CERT_FILE/HTTPS_KEY_FILE."
        )
    if cert_file != requested_cert or key_file != requested_key:
        print(
            f"TLS fallback active: requested=({requested_cert}, {requested_key}) "
            f"resolved=({cert_file}, {key_file})"
        )

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

    print(f"HTTPS server on https://{host}:{port} (dir={directory}, cert={cert_file})")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
