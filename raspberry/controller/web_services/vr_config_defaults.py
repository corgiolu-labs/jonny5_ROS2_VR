"""
Authoritative Raspberry-side defaults for VR/IMU routing config.

These defaults are used for three distinct concerns:
  1. UI fallback / first-load defaults
  2. persisted-config merge on the Raspberry Pi
  3. runtime apply fallback when building SET_VR_PARAMS

They are intentionally defined in one backend module so frontend and backend do
not maintain divergent numeric copies.
"""

from copy import deepcopy


VR_PATCH_BAY_STATE_DEFAULTS = {
    # Corrispondenza FISICA reale asse-per-asse (niente diagonale "cosmetica"):
    # il firmware non incorpora permutazioni del polso, quindi ogni cella indica
    # quale asse del visore guida davvero il servo.
    "roll": {"src": 0, "sign": -1},  # ROLL(robot)  <- YAW(visore)  [invertito]
    "pitch": {"src": 2, "sign": 1},  # PITCH(robot) <- ROLL(visore)
    "yaw": {"src": 1, "sign": 1},    # YAW(robot)   <- PITCH(visore)
}

VR_PATCH_BAY_ENABLE_DEFAULTS = {
    "roll": True,
    "pitch": True,
    "yaw": True,
}

VR_ARM_CONTROL_DEFAULTS = {
    "preferredController": "right",
    "enabledControllers": {
        "right": True,
        "left": True,
    },
}

VR_CONTROLLER_MAPPING_STATE_DEFAULTS = {
    "base": {"src": 0, "sign": 1},
    "spalla": {"src": 1, "sign": 1},
    "gomito": {"src": 2, "sign": 1},
}

VR_CONTROLLER_MAPPING_ENABLE_DEFAULTS = {
    "base": True,
    "spalla": True,
    "gomito": True,
}

# Mode=5 — HEAD OVERFLOW ASSIST (polso da HEAD; B/S/G solo se polso in overflow).
VR_HEAD_ASSIST_DEFAULTS = {
    "enabled": True,
    "yaw": {"warnDeg": 22.0, "critDeg": 9.0},
    "pitch": {"warnDeg": 20.0, "critDeg": 8.0},
    "roll": {"warnDeg": 10.0, "critDeg": 4.0},
    "assistEnable": {"yaw": True, "pitch": True, "roll": False},
    "signYaw": 1,
    "signPitch": 1,
    "signRoll": 1,
    "gainBase": 0.48,
    "gainSpalla": 0.36,
    "gainGomito": 0.30,
    "gainRollArm": 0.10,
    "critGainMul": 1.75,
    "pitchSplit": {"spalla": 0.45, "gomito": 0.55},
    "rollSplit": {"spalla": 0.5, "gomito": 0.5},
    "assistAlpha": 0.72,
    "freeFollowAlpha": 0.16,
    "maxStepDegPerTick": 4.0,
    "reliefDeadband": 0.015,
    "releaseGraceMs": 220,
    "armReliefGainMul": 2.2,
    "armAssistAlphaMul": 2.0,
    "minSpallaStepDeg": 0.45,
    "minGomitoStepDeg": 0.35,
    "headMotionFollow": True,
    "headMotionDeadbandDeg": 0.18,
    "headMotionGainYaw": 1.0,
    "headMotionGainPitch": 1.30,
    "headMotionGainRoll": 0.0,
    "headMotionMaxStepDegPerTick": 4.0,
    "pitchReachEnabled": True,
    "pitchReachBias": 0.82,
}

LEGACY_VR_HEAD_LOOP_DEFAULTS = {
    "tg-yaw": 2.0,
    "tg-pitch": 3.0,
    "tg-roll": 1.0,
    "tg-alpha-small": 0.05,
    "tg-alpha-large": 0.35,
    "tg-deadzone": 3.0,
    "tg-maxstep": 60,
    "tg-velmax": 60,
    "tg-veldigital": 35,
    "tg-lpf-pitch": 1.0,
    "tg-lpf-roll": 1.0,
    "tg-joy-dz": 0.10,
    "tg-sensitivity": 1.0,
}

VR_TUNE_DEFAULTS = {
    "tg-vel-base": 0,
    "tg-vel-spalla": 0,
    "tg-vel-gomito": 0,
    "tg-vel-yaw": 0,
    "tg-vel-pitch": 0,
    "tg-vel-roll": 0,
    "tg-vel-yaw-head": 0,
    "tg-vel-pitch-head": 0,
    "tg-vel-roll-head": 0,
    "tg-vel-base-head": 0,
    "tg-vel-spalla-head": 0,
    "tg-vel-gomito-head": 0,
}


def get_vr_config_defaults() -> dict:
    """Return a fresh copy of the authoritative backend defaults."""
    return {
        "pbState": deepcopy(VR_PATCH_BAY_STATE_DEFAULTS),
        "pbEn": deepcopy(VR_PATCH_BAY_ENABLE_DEFAULTS),
        "armControl": deepcopy(VR_ARM_CONTROL_DEFAULTS),
        "controllerMappings": {
            "right": {
                "state": deepcopy(VR_CONTROLLER_MAPPING_STATE_DEFAULTS),
                "en": deepcopy(VR_CONTROLLER_MAPPING_ENABLE_DEFAULTS),
            },
            "left": {
                "state": deepcopy(VR_CONTROLLER_MAPPING_STATE_DEFAULTS),
                "en": deepcopy(VR_CONTROLLER_MAPPING_ENABLE_DEFAULTS),
            },
        },
        "headAssist": deepcopy(VR_HEAD_ASSIST_DEFAULTS),
        "tuning": deepcopy(VR_TUNE_DEFAULTS),
    }


def merge_vr_config_with_defaults(cfg: dict | None) -> dict:
    """
    Merge a possibly partial VR/IMU config with backend defaults.

    This is a structural merge only; callers still decide whether the resulting
    config should merely be persisted/shown or actually applied to firmware.
    """
    base = get_vr_config_defaults()
    cfg = cfg or {}

    merged = {**cfg}
    pb_state = cfg.get("pbState") or {}
    pb_enable = cfg.get("pbEn") or {}
    tuning = cfg.get("tuning") or {}

    merged["pbState"] = {
        axis: {**base["pbState"][axis], **(pb_state.get(axis) or {})}
        for axis in base["pbState"]
    }
    merged["pbEn"] = {
        axis: bool(pb_enable.get(axis, base["pbEn"][axis]))
        for axis in base["pbEn"]
    }
    arm_control = cfg.get("armControl") or {}
    enabled_src = arm_control.get("enabledControllers") or {}
    enabled_controllers = {
        side: bool(enabled_src.get(side, base["armControl"]["enabledControllers"][side]))
        for side in ("right", "left")
    }
    preferred_controller = str(
        arm_control.get(
            "preferredController",
            base["armControl"]["preferredController"],
        )
    ).strip().lower()
    if preferred_controller not in ("right", "left"):
        preferred_controller = base["armControl"]["preferredController"]
    if not enabled_controllers.get(preferred_controller):
        if enabled_controllers.get("right"):
            preferred_controller = "right"
        elif enabled_controllers.get("left"):
            preferred_controller = "left"
        else:
            preferred_controller = base["armControl"]["preferredController"]
    merged["armControl"] = {
        "preferredController": preferred_controller,
        "enabledControllers": enabled_controllers,
    }
    controller_mappings = cfg.get("controllerMappings") or {}
    merged["controllerMappings"] = {}
    for side in ("right", "left"):
        side_cfg = controller_mappings.get(side) or {}
        side_state = side_cfg.get("state") or {}
        side_en = side_cfg.get("en") or {}
        merged["controllerMappings"][side] = {
            "state": {
                axis: {
                    **VR_CONTROLLER_MAPPING_STATE_DEFAULTS[axis],
                    **(side_state.get(axis) or {}),
                }
                for axis in VR_CONTROLLER_MAPPING_STATE_DEFAULTS
            },
            "en": {
                axis: bool(side_en.get(axis, VR_CONTROLLER_MAPPING_ENABLE_DEFAULTS[axis]))
                for axis in VR_CONTROLLER_MAPPING_ENABLE_DEFAULTS
            },
        }

    ha_in = cfg.get("headAssist")
    if not isinstance(ha_in, dict):
        ha_in = {}
    if not ha_in and isinstance(cfg.get("ikMode"), dict):
        ha_in = {"enabled": bool((cfg.get("ikMode") or {}).get("enabled", True))}

    b_ha = base["headAssist"]
    yaw_in = ha_in.get("yaw") if isinstance(ha_in.get("yaw"), dict) else {}
    pitch_in = ha_in.get("pitch") if isinstance(ha_in.get("pitch"), dict) else {}
    roll_in = ha_in.get("roll") if isinstance(ha_in.get("roll"), dict) else {}
    ae_in = ha_in.get("assistEnable") if isinstance(ha_in.get("assistEnable"), dict) else {}
    ps_in = ha_in.get("pitchSplit") if isinstance(ha_in.get("pitchSplit"), dict) else {}
    rs_in = ha_in.get("rollSplit") if isinstance(ha_in.get("rollSplit"), dict) else {}

    merged["headAssist"] = {
        "enabled": bool(ha_in.get("enabled", b_ha["enabled"])),
        "yaw": {**b_ha["yaw"], **yaw_in},
        "pitch": {**b_ha["pitch"], **pitch_in},
        "roll": {**b_ha["roll"], **roll_in},
        "assistEnable": {
            "yaw": bool(ae_in.get("yaw", b_ha["assistEnable"]["yaw"])),
            "pitch": bool(ae_in.get("pitch", b_ha["assistEnable"]["pitch"])),
            "roll": bool(ae_in.get("roll", b_ha["assistEnable"]["roll"])),
        },
        "signYaw": int(ha_in.get("signYaw", b_ha["signYaw"])),
        "signPitch": int(ha_in.get("signPitch", b_ha["signPitch"])),
        "signRoll": int(ha_in.get("signRoll", b_ha["signRoll"])),
        "gainBase": float(ha_in.get("gainBase", b_ha["gainBase"])),
        "gainSpalla": float(ha_in.get("gainSpalla", b_ha["gainSpalla"])),
        "gainGomito": float(ha_in.get("gainGomito", b_ha["gainGomito"])),
        "gainRollArm": float(ha_in.get("gainRollArm", b_ha["gainRollArm"])),
        "critGainMul": float(ha_in.get("critGainMul", b_ha["critGainMul"])),
        "pitchSplit": {**b_ha["pitchSplit"], **ps_in},
        "rollSplit": {**b_ha["rollSplit"], **rs_in},
        "assistAlpha": float(ha_in.get("assistAlpha", b_ha["assistAlpha"])),
        "freeFollowAlpha": float(ha_in.get("freeFollowAlpha", b_ha["freeFollowAlpha"])),
        "maxStepDegPerTick": float(ha_in.get("maxStepDegPerTick", b_ha["maxStepDegPerTick"])),
        "reliefDeadband": float(ha_in.get("reliefDeadband", b_ha["reliefDeadband"])),
        "releaseGraceMs": float(ha_in.get("releaseGraceMs", b_ha["releaseGraceMs"])),
        "armReliefGainMul": float(ha_in.get("armReliefGainMul", b_ha["armReliefGainMul"])),
        "armAssistAlphaMul": float(ha_in.get("armAssistAlphaMul", b_ha["armAssistAlphaMul"])),
        "minSpallaStepDeg": float(ha_in.get("minSpallaStepDeg", b_ha["minSpallaStepDeg"])),
        "minGomitoStepDeg": float(ha_in.get("minGomitoStepDeg", b_ha["minGomitoStepDeg"])),
        "headMotionFollow": bool(ha_in.get("headMotionFollow", b_ha["headMotionFollow"])),
        "headMotionDeadbandDeg": float(ha_in.get("headMotionDeadbandDeg", b_ha["headMotionDeadbandDeg"])),
        "headMotionGainYaw": float(ha_in.get("headMotionGainYaw", b_ha["headMotionGainYaw"])),
        "headMotionGainPitch": float(ha_in.get("headMotionGainPitch", b_ha["headMotionGainPitch"])),
        "headMotionGainRoll": float(ha_in.get("headMotionGainRoll", b_ha["headMotionGainRoll"])),
        "headMotionMaxStepDegPerTick": float(ha_in.get("headMotionMaxStepDegPerTick", b_ha["headMotionMaxStepDegPerTick"])),
        "pitchReachEnabled": bool(ha_in.get("pitchReachEnabled", b_ha["pitchReachEnabled"])),
        "pitchReachBias": float(ha_in.get("pitchReachBias", b_ha["pitchReachBias"])),
    }

    merged["tuning"] = {**base["tuning"], **tuning}
    merged.pop("ikMode", None)
    return merged
