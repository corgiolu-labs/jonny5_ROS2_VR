import json, ssl, sys
from websockets.sync.client import connect
ctx = ssl.create_default_context(); ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
cmd = sys.argv[1]
msg = {"type": "imu_campaign"}
if cmd in ("on", "off"):
    msg["on"] = cmd == "on"
if len(sys.argv) > 2:
    msg["label"] = sys.argv[2]
ws = connect("wss://127.0.0.1:8557", ssl=ctx)
ws.send(json.dumps(msg))
for _ in range(200):
    m = json.loads(ws.recv(timeout=5))
    if m.get("type") == "imu_campaign_state":
        print(m); break
ws.close()
