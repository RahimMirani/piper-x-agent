from dataclasses import dataclass
from pathlib import Path
import math
import tomllib


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

    @classmethod
    def load(cls, path: Path):
        path = path.resolve()
        with path.open("rb") as stream:
            data = tomllib.load(stream)
        allowed = {"mode", "run_directory", "arm", "cameras", "camera_max_age_s", "camera_timeout_s"}
        if data.keys() - allowed:
            raise ValueError(f"Unknown config keys: {data.keys() - allowed}")
        arm, cameras = data.get("arm", {}), data.get("cameras", {})
        if arm.keys() - {"channel", "firmware"} or cameras.keys() - {"scene_serial", "wrist_serial"}:
            raise ValueError("Unknown arm/camera configuration key")
        mode = data.get("mode")
        if mode not in {"mock", "hardware_readonly"}:
            raise ValueError("mode must be mock or hardware_readonly; live motion is not implemented")
        scene, wrist = cameras.get("scene_serial"), cameras.get("wrist_serial")
        if not all(isinstance(s, str) and s.strip() for s in (scene, wrist)) or scene == wrist:
            raise ValueError("Set two distinct, nonempty camera serials")
        limits = [data.get("camera_max_age_s", 1.0), data.get("camera_timeout_s", 3.0)]
        if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 30 for v in limits):
            raise ValueError("Camera timing limits must be finite, positive, and <= 30 seconds")
        channel, firmware = arm.get("channel", "can0"), arm.get("firmware", "v188")
        if not isinstance(channel, str) or not channel.strip() or not isinstance(firmware, str):
            raise ValueError("Invalid arm channel or firmware")
        return cls(mode, (path.parent / data.get("run_directory", "../runs")).resolve(),
                   channel, firmware, scene, wrist, *limits)
