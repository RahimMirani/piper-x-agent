from dataclasses import dataclass, field
from pathlib import Path
import math
import tomllib

# Hard ceilings the configuration cannot raise. A TOML file is easy to edit in a
# hurry; these are the numbers that stay put.
MAX_SPEED_PERCENT = 30
MAX_JOINT_STEP_RAD = 1.0
MAX_CARTESIAN_STEP_M = 0.25

_MODES = {"mock", "hardware_readonly", "hardware_live"}


def _finite(value):
    return type(value) in (int, float) and not isinstance(value, bool) and math.isfinite(value)


def _bound(values, name):
    if values is None:
        return None
    if not isinstance(values, list) or len(values) != 3 or not all(_finite(v) for v in values):
        raise ValueError(f"{name} must be three finite numbers in metres")
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class LiveLimits:
    """Bounds every live command is checked against. Defaults are deliberately small."""

    speed_percent: int = 10
    max_joint_step_rad: float = 0.5
    max_cartesian_step_m: float = 0.10
    workspace_min_m: tuple | None = None
    workspace_max_m: tuple | None = None

    def workspace_bounds(self):
        if self.workspace_min_m is None or self.workspace_max_m is None:
            return None
        return self.workspace_min_m, self.workspace_max_m

    @classmethod
    def load(cls, data):
        allowed = {"speed_percent", "max_joint_step_rad", "max_cartesian_step_m",
                   "workspace_min_m", "workspace_max_m"}
        if data.keys() - allowed:
            raise ValueError(f"Unknown live configuration key: {data.keys() - allowed}")
        speed = data.get("speed_percent", 10)
        if type(speed) is not int or isinstance(speed, bool) or not 1 <= speed <= MAX_SPEED_PERCENT:
            raise ValueError(f"speed_percent must be an integer from 1 to {MAX_SPEED_PERCENT}")
        joint_step = data.get("max_joint_step_rad", 0.5)
        if not _finite(joint_step) or not 0 < joint_step <= MAX_JOINT_STEP_RAD:
            raise ValueError(f"max_joint_step_rad must be in (0, {MAX_JOINT_STEP_RAD}]")
        cartesian_step = data.get("max_cartesian_step_m", 0.10)
        if not _finite(cartesian_step) or not 0 < cartesian_step <= MAX_CARTESIAN_STEP_M:
            raise ValueError(f"max_cartesian_step_m must be in (0, {MAX_CARTESIAN_STEP_M}]")
        low = _bound(data.get("workspace_min_m"), "workspace_min_m")
        high = _bound(data.get("workspace_max_m"), "workspace_max_m")
        if (low is None) != (high is None):
            raise ValueError("Set both workspace_min_m and workspace_max_m, or neither")
        if low is not None and any(a >= b for a, b in zip(low, high)):
            raise ValueError("Each workspace_min_m axis must be below workspace_max_m")
        return cls(speed, float(joint_step), float(cartesian_step), low, high)


@dataclass(frozen=True)
class Config:
    mode: str
    run_directory: Path
    channel: str
    firmware: str
    scene_serial: str
    wrist_serial: str
    camera_max_age_s: float = 1.0
    camera_timeout_s: float = 3.0
    live: LiveLimits = field(default_factory=LiveLimits)

    @classmethod
    def load(cls, path: Path):
        path = path.resolve()
        with path.open("rb") as stream:
            data = tomllib.load(stream)
        allowed = {"mode", "run_directory", "arm", "cameras", "camera_max_age_s",
                   "camera_timeout_s", "live"}
        if data.keys() - allowed:
            raise ValueError(f"Unknown config keys: {data.keys() - allowed}")
        arm, cameras = data.get("arm", {}), data.get("cameras", {})
        if arm.keys() - {"channel", "firmware"} or cameras.keys() - {"scene_serial", "wrist_serial"}:
            raise ValueError("Unknown arm/camera configuration key")
        mode = data.get("mode")
        if mode not in _MODES:
            raise ValueError(f"mode must be one of {sorted(_MODES)}")
        live = LiveLimits.load(data.get("live", {}))
        if "live" in data and mode != "hardware_live":
            raise ValueError("A [live] section is only meaningful with mode = \"hardware_live\"")
        scene, wrist = cameras.get("scene_serial"), cameras.get("wrist_serial")
        if not all(isinstance(s, str) and s.strip() for s in (scene, wrist)) or scene == wrist:
            raise ValueError("Set two distinct, nonempty camera serials")
        limits = [data.get("camera_max_age_s", 1.0), data.get("camera_timeout_s", 3.0)]
        if any(not _finite(v) or not 0 < v <= 30 for v in limits):
            raise ValueError("Camera timing limits must be finite, positive, and <= 30 seconds")
        channel, firmware = arm.get("channel", "can0"), arm.get("firmware", "v188")
        if not isinstance(channel, str) or not channel.strip() or not isinstance(firmware, str):
            raise ValueError("Invalid arm channel or firmware")
        return cls(mode, (path.parent / data.get("run_directory", "../runs")).resolve(),
                   channel, firmware, scene, wrist, *limits, live=live)
