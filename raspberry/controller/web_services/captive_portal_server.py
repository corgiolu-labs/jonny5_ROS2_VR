import argparse
import http.server
import os

from controller.web_services import captive_portal_auth


_PROBE_PATHS = {
    "/",
    "/index.html",
    "/204",
    "/generate_204",
    "/gen_204",
    "/generate204",
    "/hotspot-detect.html",
    "/redirect",
    "/connecttest.txt",
    "/mobile/status.php",
    "/ipv6check",
    "/ncsi.txt",
    "/canonical.html",
    "/library/test/success.html",
    "/kindle-wifi/wifistub.html",
    "/success.txt",
    "/fwlink",
    "/captive-portal/launch",
}

def _launch_url() -> str:
    return os.environ.get("CAPTIVE_PORTAL_LAUNCH_URL", "http://10.42.0.1/captive-portal/launch")


def _https_target() -> str:
    return os.environ.get(
        "CAPTIVE_PORTAL_TARGET_HTTPS",
        "https://10.42.0.1/captive-portal/open",
    )

class CaptivePortalHandler(http.server.BaseHTTPRequestHandler):
    server_version = "JONNY5Captive/1.2"

    def do_HEAD(self):
        self._handle_request(body=False)

    def do_GET(self):
        self._handle_request(body=True)

    def _handle_request(self, body: bool):
        path_clean = self.path.split("?", 1)[0].split("#", 1)[0]
        ip = self.client_address[0] if self.client_address else "0.0.0.0"
        if path_clean == "/captive-portal/connect":
            captive_portal_auth.mark_authenticated(ip)
            self._redirect("/captive-portal/connected", body=body)
            return
        if path_clean == "/captive-portal/touch":
            captive_portal_auth.mark_authenticated(ip)
            self._respond_no_content()
            return
        if path_clean == "/captive-portal/connected":
            self._serve_connected_page(body=body)
            return
        if path_clean in _PROBE_PATHS and captive_portal_auth.is_authenticated(ip):
            self._respond_no_content()
            return
        if path_clean == "/captive-portal/launch":
            self._serve_launch_page(body=body)
            return
        if path_clean in _PROBE_PATHS:
            self._redirect(_launch_url(), body=body)
            return
        self._serve_launch_page(body=body)

    def _respond_no_content(self):
        self.send_response(204)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

    def _redirect(self, url: str, body: bool):
        payload = (
            f"<!doctype html><html><head><meta http-equiv=\"refresh\" content=\"0; url={url}\"></head>"
            f"<body>Redirecting to <a href=\"{url}\">{url}</a></body></html>"
        ).encode("utf-8")
        self.send_response(302)
        self.send_header("Location", url)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        if body:
            self.wfile.write(payload)

    def _serve_launch_page(self, body: bool):
        target_url = _https_target()
        html = f"""<!doctype html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>JONNY5 Hotspot</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #061018;
      --card: rgba(14, 32, 48, .95);
      --ink: #eef7ff;
      --muted: #a9c4dd;
      --accent: #49a5ff;
    }}
    body {{
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background:
        radial-gradient(circle at top, rgba(73,165,255,.28), transparent 34%),
        linear-gradient(180deg, #05101a 0%, var(--bg) 100%);
      font-family: "Segoe UI", system-ui, sans-serif;
      color: var(--ink);
    }}
    main {{
      width: min(92vw, 540px);
      padding: 28px;
      border-radius: 24px;
      background: var(--card);
      border: 1px solid rgba(132,176,214,.22);
      box-shadow: 0 18px 60px rgba(0,0,0,.35);
    }}
    h1 {{ margin: 0 0 12px; font-size: 2rem; }}
    p {{ margin: 0 0 14px; color: var(--muted); line-height: 1.5; }}
    .actions {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 12px;
    }}
    a {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-height: 52px;
      padding: 0 18px;
      margin-top: 8px;
      border-radius: 14px;
      text-decoration: none;
      background: var(--accent);
      color: white;
      font-weight: 700;
    }}
    .secondary {{
      background: transparent;
      border: 1px solid rgba(132,176,214,.28);
      color: var(--ink);
    }}
    code {{
      display: inline-block;
      padding: 4px 8px;
      border-radius: 8px;
      background: rgba(255,255,255,.06);
      color: var(--ink);
      font-size: .95rem;
    }}
  </style>
</head>
<body>
  <main>
    <h1>JONNY5 Robot</h1>
    <p>Hotspot locale rilevato. Questa pagina resta volutamente su HTTP per massimizzare la compatibilita con Quest 1 / browser captive legacy.</p>
    <p>Apri il viewer VR con il pulsante qui sotto. Se appare un avviso sul certificato locale del robot, conferma una volta e prosegui.</p>
    <div class="actions">
      <a href="/captive-portal/connect">Connetti e apri Viewer VR</a>
      <a class="secondary" href="https://10.42.0.1/dashboard/index.html">Apri Dashboard</a>
    </div>
    <p style="margin-top:18px">Se il browser captive non apre il link al primo tocco, riprova dalla stessa pagina oppure usa questo indirizzo locale:</p>
    <p><code>https://10.42.0.1/vr/viewer_stereo_xr.html</code></p>
  </main>
</body>
</html>
"""
        payload = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        if body:
            self.wfile.write(payload)

    def _serve_connected_page(self, body: bool):
        target_url = _https_target()
        html = f"""<!doctype html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>JONNY5 Connected</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #061018;
      --card: rgba(14, 32, 48, .95);
      --ink: #eef7ff;
      --muted: #a9c4dd;
      --accent: #49a5ff;
    }}
    body {{
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background:
        radial-gradient(circle at top, rgba(73,165,255,.28), transparent 34%),
        linear-gradient(180deg, #05101a 0%, var(--bg) 100%);
      font-family: "Segoe UI", system-ui, sans-serif;
      color: var(--ink);
    }}
    main {{
      width: min(92vw, 540px);
      padding: 28px;
      border-radius: 24px;
      background: var(--card);
      border: 1px solid rgba(132,176,214,.22);
      box-shadow: 0 18px 60px rgba(0,0,0,.35);
    }}
    h1 {{ margin: 0 0 12px; font-size: 2rem; }}
    p {{ margin: 0 0 14px; color: var(--muted); line-height: 1.5; }}
    a {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-height: 52px;
      padding: 0 18px;
      margin-top: 8px;
      border-radius: 14px;
      text-decoration: none;
      background: var(--accent);
      color: white;
      font-weight: 700;
    }}
    .secondary {{
      background: transparent;
      border: 1px solid rgba(132,176,214,.28);
      color: var(--ink);
    }}
  </style>
</head>
<body>
  <main>
    <h1>&#x2713; Rete robot pronta</h1>
    <p>Autorizzazione completata. Ora puoi usare il robot.</p>
    <div style="display:flex;gap:12px;flex-wrap:wrap;margin-top:14px">
      <a id="open-link" href="{target_url}">Apri Viewer VR</a>
      <a class="secondary" href="http://10.42.0.1/captive-portal/launch">Torna al portal</a>
    </div>
    <p style="margin-top:20px;font-size:.9rem">
      <strong>Su Meta Quest:</strong> se il link non si apre, chiudi questo popup,
      apri <strong>Quest Browser</strong> e vai a:<br>
      <code style="word-break:break-all">https://10.42.0.1/vr/viewer_stereo_xr.html</code><br>
      Accetta l'avviso certificato locale una volta sola.
    </p>
    <p id="status" style="margin-top:8px;color:var(--muted);font-size:.85rem">Autorizzazione attiva.</p>
  </main>
  <script>
    (function () {{
      const statusEl = document.getElementById('status');
      async function touch() {{
        try {{
          await fetch('/captive-portal/touch', {{ cache: 'no-store', credentials: 'same-origin' }});
        }} catch (_) {{}}
      }}
      setInterval(touch, 2000);
      touch();
      document.getElementById('open-link').addEventListener('click', function () {{
        statusEl.textContent = 'Apertura viewer...';
      }});
    }})();
  </script>
</body>
</html>
"""
        payload = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        if body:
            self.wfile.write(payload)

    def log_message(self, fmt, *args):
        path = args[0] if args else ""
        path = path if isinstance(path, str) else str(path)
        if "/generate_204" not in path and "/gen_204" not in path:
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description="JONNY5 captive portal HTTP server")
    parser.parse_args()
    host = os.environ.get("CAPTIVE_PORTAL_BIND", "0.0.0.0")
    port = int(os.environ.get("CAPTIVE_PORTAL_PORT", "80"))
    httpd = http.server.ThreadingHTTPServer((host, port), CaptivePortalHandler)
    print(f"Captive portal server on http://{host}:{port} -> {_https_target()}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
