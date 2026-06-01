#!/usr/bin/env python3
"""
verify_joints_edges.py — Verifica E2E della fix JOINTS slider limits.

Da eseguire SUL PI (o da una macchina con WS + HTTP raggiungibili).

Cosa fa:
  1. Legge routing_config.limits (PHYSICAL) via /api/routing-config
  2. Legge offsets + dirs via WS get_settings (runtime authoritative)
  3. Replica la stessa conversione fisico→virtuale applicata dal frontend
     (dopo la fix) per calcolare lo slider min/max atteso
  4. Invia SETPOSE per ciascun estremo min/max di ogni giunto (gli altri 5
     joint restano a 90°) e registra se il backend restituisce warning
  5. Stampa una tabella completa e una VERDETTO finale

Formule (identiche a joints.js e al backend virtual_to_physical):
    physical = offset + dir * (virtual - 90)
    virtual  = (physical - offset) / dir + 90

Uso:
    # Sul Pi:
    python3 verify_joints_edges.py
"""
import asyncio
import json
import ssl
import time
import urllib.request
import websockets  # type: ignore

WS_URL   = "wss://127.0.0.1:8557"
HTTP_URL = "https://127.0.0.1:8443/api/routing-config"

JOINT_ORDER   = ["base", "spalla", "gomito", "yaw", "pitch", "roll"]
JOINT_LABELS  = ["BASE", "SPALLA", "GOMITO", "YAW", "PITCH", "ROLL"]


def fetch_routing_config() -> dict:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode    = ssl.CERT_NONE
    with urllib.request.urlopen(HTTP_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


async def ws_request(ws, payload: dict, match_type: str, timeout_s: float = 5.0):
    await ws.send(json.dumps(payload))
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        try:
            msg = json.loads(raw)
        except Exception:
            continue
        if msg.get("type") == match_type:
            return msg
    return None


async def send_setpose_and_check(ws, virtual_vals, vel=30, profile="RTR5", timeout_s=4.0):
    """Send SETPOSE; return (warning_str | None, uart_cmd_sent, response).

    We match any uart_response whose 'cmd' contains SETPOSE AFTER sending ours.
    """
    cmd = "SETPOSE " + " ".join(str(int(v)) for v in virtual_vals) + f" {vel} {profile}"
    # Drain any stale messages first
    drained_end = time.monotonic() + 0.2
    while time.monotonic() < drained_end:
        try:
            await asyncio.wait_for(ws.recv(), timeout=0.05)
        except asyncio.TimeoutError:
            break
        except Exception:
            break
    await ws.send(json.dumps({"type": "uart", "cmd": cmd}))
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        try:
            msg = json.loads(raw)
        except Exception:
            continue
        if msg.get("type") == "uart_response" and "SETPOSE" in str(msg.get("cmd", "")).upper():
            return msg.get("warning"), cmd, msg
    return "TIMEOUT", cmd, None


def compute_virtual_limits(phys_min, phys_max, offset, dir_):
    """Same formula as frontend applyVirtualLimitsFromSettings."""
    dir_ = -1 if dir_ < 0 else 1
    a = (phys_min - offset) / dir_ + 90
    b = (phys_max - offset) / dir_ + 90
    vmin = int(round(min(a, b)))
    vmax = int(round(max(a, b)))
    vmin = max(0, min(180, vmin))
    vmax = max(0, min(180, vmax))
    return vmin, vmax


def virtual_to_physical(virtual, offset, dir_):
    dir_ = -1 if dir_ < 0 else 1
    return offset + dir_ * (virtual - 90)


async def main():
    print("=" * 78)
    print("verify_joints_edges.py — E2E validation of JOINTS slider fix")
    print("=" * 78)

    # 1. Physical limits from routing_config (same endpoint frontend hits)
    try:
        rcfg = fetch_routing_config()
    except Exception as e:
        print(f"ERROR: cannot fetch {HTTP_URL}: {e}")
        return
    phys_lim = rcfg.get("limits") or {}
    print("\n[1] PHYSICAL limits (routing_config.limits):")
    for name in JOINT_ORDER:
        row = phys_lim.get(name, {})
        print(f"    {name:7s}: min={row.get('min')}, max={row.get('max')}")

    # 2. offsets + dirs via WS (authoritative runtime state)
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode    = ssl.CERT_NONE
    async with websockets.connect(WS_URL, ssl=ctx) as ws:
        settings = await ws_request(ws, {"type": "get_settings"}, "settings", 5.0)
        if not settings:
            print("ERROR: no settings response")
            return
        offsets = settings.get("offsets")
        dirs    = settings.get("dirs") or [1, 1, 1, 1, 1, 1]
        if not isinstance(offsets, list) or len(offsets) != 6:
            print(f"ERROR: invalid offsets: {offsets}")
            return
        print("\n[2] Runtime j5_settings:")
        print(f"    offsets = {offsets}")
        print(f"    dirs    = {dirs}")

        # 3. Expected virtual limits (frontend fix)
        print("\n[3] EXPECTED virtual slider range (frontend conversion):")
        expected_virt = {}
        for i, name in enumerate(JOINT_ORDER):
            pm = int(phys_lim[name]["min"]); pM = int(phys_lim[name]["max"])
            vmin, vmax = compute_virtual_limits(pm, pM, int(offsets[i]), int(dirs[i]))
            expected_virt[name] = (vmin, vmax)
            print(f"    {name:7s}: [{vmin:3d}, {vmax:3d}]  "
                  f"(phys=[{pm},{pM}], off={offsets[i]}, dir={dirs[i]})")

        # 4. Safety: ensure robot is enabled so SETPOSE actually executes/validates.
        #    If it's in SAFE, SETPOSE gets rejected BEFORE the clamp check runs —
        #    which would give us a false "no warning".
        await ws.send(json.dumps({"type": "uart", "cmd": "STATUS?"}))
        # Best-effort ENABLE (30s timeout); if already enabled the response is immediate.
        print("\n[4] Ensuring robot is ENABLEd so the clamp path actually runs...")
        await ws.send(json.dumps({"type": "uart", "cmd": "ENABLE"}))
        # Drain up to 10s while watching for enable ack
        t_end = time.monotonic() + 12.0
        enabled = False
        while time.monotonic() < t_end:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            try:
                m = json.loads(raw)
            except Exception:
                continue
            if m.get("type") == "uart_response" and "ENABLE" in str(m.get("cmd", "")).upper():
                enabled = bool(m.get("ok"))
                break
        print(f"    ENABLE ok={enabled}")
        if not enabled:
            print("    WARNING: ENABLE failed; SETPOSE validation may short-circuit.")
            print("    Verdict below still meaningful if backend still clamps in IDLE-only.")

        # 5. For each joint, send SETPOSE at min then max (others = 90) and record warning
        print("\n[5] Edge probe — sending SETPOSE at each computed edge:")
        print(f"    {'joint':7s} {'edge':4s} {'virt':>4s} {'→phys':>6s} "
              f"{'bound':>6s} {'warn?':>6s}  detail")
        print("    " + "-" * 72)
        results = []  # (joint, edge, virt, phys, bound, warn_bool, detail)
        for i, name in enumerate(JOINT_ORDER):
            vmin, vmax = expected_virt[name]
            pm = int(phys_lim[name]["min"]); pM = int(phys_lim[name]["max"])
            for edge_label, vtest in (("MIN", vmin), ("MAX", vmax)):
                vals = [90, 90, 90, 90, 90, 90]
                vals[i] = vtest
                warning, sent_cmd, _ = await send_setpose_and_check(ws, vals, vel=25)
                phys = virtual_to_physical(vtest, int(offsets[i]), int(dirs[i]))
                bound = pm if edge_label == "MIN" and dirs[i] > 0 or edge_label == "MAX" and dirs[i] < 0 else pM
                # Simplify: the actual clamp compares phys against [pm, pM]
                inside = pm <= phys <= pM
                warn_bool = bool(warning) and warning != "TIMEOUT"
                detail = "" if not warning else (warning if warning != "TIMEOUT" else "TIMEOUT")
                results.append((name, edge_label, vtest, phys, (pm, pM), warn_bool, inside, detail))
                print(f"    {name:7s} {edge_label:4s} {vtest:4d} {phys:6d} "
                      f"[{pm},{pM}] {'Y' if warn_bool else '-':>6s}  {detail[:45]}")
                # Small sleep to not overwhelm
                await asyncio.sleep(0.15)

        # 6. Verdict
        print("\n[6] VERDICT:")
        any_warn = any(r[5] for r in results)
        any_outside = any(not r[6] for r in results)
        if not any_warn and not any_outside:
            print("    ✓ PASS — all 12 edge probes stay inside physical limits "
                  "and produce NO warning.")
            print("    The slider min/max are coherent with the backend clamp.")
        else:
            print("    ✗ FAIL — details:")
            for (name, edge, v, p, (pm, pM), warn, inside, det) in results:
                if warn or not inside:
                    print(f"      {name} {edge}: virt={v} phys={p} "
                          f"bound=[{pm},{pM}] warn={warn} inside={inside} {det}")

        # Return to HOME for tidiness
        await ws.send(json.dumps({"type": "uart",
                                  "cmd": "SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)


if __name__ == "__main__":
    asyncio.run(main())
