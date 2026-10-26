#!/usr/bin/env python3
"""Generate j5mini.json from the Pi's own configuration, with the Pi's own code.

Run it ON THE PI (the calibrated files live there), with its venv:

    ~/raspberry5/.venv/bin/python3 export_config.py ~/raspberry5 > j5mini.json

It imports raspberry/controller/web_services/ws_handlers_uart.py and calls the same builders
ws-teleop uses on BOOT_READY, so the lines are byte-identical to what the Pi sends today:
SET_OFFSETS (j5_settings.json), SET_VR_PARAMS + SET_JOINT_LIMITS (routing_config.json),
SET_PWM_CONFIG (pwm_config.json), then IMUON.
"""
import json
import pathlib
import sys


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()   # the "raspberry" directory
    sys.path.insert(0, str(root))
    from controller.web_services import settings_manager
    from controller.web_services import ws_handlers_uart as u

    offsets = [int(v) for v in settings_manager.load().get("offsets", settings_manager.DEFAULTS["offsets"])]
    lines = ["SET_OFFSETS " + " ".join(str(v) for v in offsets)]
    raw = u.load_persisted_vr_config()
    if raw:
        cfg = u._build_runtime_vr_config_for_apply(raw)
        lines.append(u._build_set_vr_params_from_config(cfg))
        for idx, j in enumerate(["base", "spalla", "gomito", "yaw", "pitch", "roll"]):
            lim = (cfg.get("limits") or {}).get(j)
            if lim and "min" in lim and "max" in lim:
                lines.append(f"SET_JOINT_LIMITS {idx} {int(lim['min'])} {int(lim['max'])}")
    lines.append(u._build_set_pwm_config_cmd(u.load_persisted_pwm_config()))
    lines.append("IMUON")
    out = {
        "boot_commands": lines,
        "spi": {"device": "/dev/spidev0.0", "speed_hz": 1000000, "transfer_len": 64},
        "uart": {"device": "/dev/serial0", "baud": 115200},
        "ws": {"host": "0.0.0.0", "port": 8557,
               "cert": str(root / "config_runtime/tls/webrtc.crt"), "key": str(root / "config_runtime/tls/webrtc.key")},
        "link_timeout_ms": 150,
        "set_clock": True,
    }
    json.dump(out, sys.stdout, indent=1)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
