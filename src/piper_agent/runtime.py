from contextlib import ExitStack
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid

from . import arming
from .arm import MockArm, ReadOnlyArm
from .cameras import MockCameras, OrbbecCameras


class ProcessLock:
    """One cooperating hardware process per user on this host."""

    def __init__(self, path=None):
        self.path = Path(path) if path else Path(tempfile.gettempdir()) / f"piper-x-agent-{os.getuid()}.lock"
        self.stream = None

    def __enter__(self):
        self.stream = self.path.open("a+")
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            self.stream = None
            raise RuntimeError("Another piper-x-agent hardware process is active") from None
        return self

    def __exit__(self, *args):
        if self.stream:
            self.stream.close()
            self.stream = None
        # Never unlink: other processes may already hold this inode open.


class Runtime:
    def __init__(self, config):
        self.config = config
        self.mutex = threading.RLock()
        self.log_mutex = threading.Lock()
        self.stack = ExitStack()
        self.arm = None
        self.cameras = None
        self.sequence = 0
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:8]
        self.directory = config.run_directory / self.run_id
        self.directory.mkdir(parents=True, mode=0o700)
        try:
            if config.mode != "mock":
                self.stack.enter_context(ProcessLock())
            self.log("session_start", {"mode": config.mode,
                                       "physical_motion_supported": self.live})
        except BaseException:
            self.stack.close()
            raise

    def log(self, event, data):
        with self.log_mutex, (self.directory / "events.jsonl").open("a") as stream:
            stream.write(json.dumps({"time_unix_s": time.time(), "event": event, "data": data},
                                    allow_nan=False) + "\n")

    @property
    def live(self):
        return self.config.mode == "hardware_live"

    def _arm(self):
        if self.arm is None:
            if self.config.mode == "mock":
                self.arm = MockArm()
            elif self.config.mode == "hardware_live":
                from .live_arm import LiveArm
                self.arm = LiveArm(self.config)
            else:
                self.arm = ReadOnlyArm(self.config)
            self.stack.callback(self.arm.close)
        return self.arm

    def _cameras(self):
        if self.cameras is None:
            self.cameras = MockCameras(self.config) if self.config.mode == "mock" else OrbbecCameras(self.config)
            self.stack.callback(self.cameras.close)
        return self.cameras

    def status(self):
        armed, _, remaining = arming.status()
        return {"mode": self.config.mode, "run_id": self.run_id,
                "physical_motion_supported": self.live,
                "motion_armed": armed if self.live else False,
                "motion_arm_seconds_remaining": round(remaining, 1) if self.live and armed else 0.0,
                "camera_roles": ["scene", "wrist"],
                "mock_is_physics_simulation": False,
                "collision_checking": False,
                "hardware_connected": self.arm is not None and self.config.mode != "mock",
                "run_directory_on_server": str(self.directory)}

    def state(self):
        with self.mutex:
            state = self._arm().state()
            self.log("state", state)
            return state

    def observe(self, include_arm=True):
        with self.mutex:
            start = time.monotonic()
            frames = self._cameras().capture()
            state = self._arm().state() if include_arm else None
            if any(time.monotonic() - f.received_monotonic > self.config.camera_max_age_s for f in frames):
                raise RuntimeError("Frames became stale while reading arm state; request a new observation")
            self.sequence += 1
            metadata = {"observation_id": self.sequence, "mode": self.config.mode, "state": state,
                        "mock_images_are_diagnostic_patterns": self.config.mode == "mock",
                        "synchronized": False, "frames": [],
                        "capture_skew_s": abs(frames[0].received_monotonic - frames[1].received_monotonic)}
            for frame in frames:
                extension = "png" if frame.mime == "image/png" else "jpg"
                name = f"{self.sequence:06d}-{frame.role}.{extension}"
                (self.directory / name).write_bytes(frame.data)
                metadata["frames"].append({**frame.metadata(), "file": name})
            metadata["tool_elapsed_s"] = time.monotonic() - start
            self.log("observation", metadata)
            return metadata, frames

    def simulate(self, action, value=None):
        with self.mutex:
            if self.config.mode != "mock":
                raise RuntimeError("Simulation commands are unavailable in hardware_readonly mode")
            try:
                arm = self._arm()
                if action == "move":
                    result = arm.move(value)
                elif action == "grip":
                    result = arm.grip(value)
                elif action == "stop":
                    result = arm.stop()
                else:
                    raise ValueError("Unknown mock action")
                self.log("mock_action", {"action": action, "value": value, "result": result})
                return result
            except Exception as exc:
                # Reject nonfinite values without allowing invalid JSON in logs.
                self.log("mock_action_rejected", {"action": action, "error": str(exc)})
                raise

    def act(self, action, value=None):
        """Execute one bounded physical command inside an armed window."""
        with self.mutex:
            if not self.live:
                raise RuntimeError("Physical motion requires mode = \"hardware_live\"")
            # Re-check on every call: a window opened before this session can
            # expire part way through it.
            remaining = arming.require_armed()
            arm = self._arm()
            try:
                if action == "move_joints":
                    result = arm.move_joints(value)
                elif action == "move_to_pose":
                    result = arm.move_to_pose(value)
                elif action == "set_gripper":
                    result = arm.set_gripper(**value)
                elif action == "stop":
                    result = arm.stop()
                else:
                    raise ValueError(f"Unknown physical action: {action}")
            except Exception as exc:
                self.log("action_rejected", {"action": action, "value": value,
                                             "error": f"{type(exc).__name__}: {exc}"})
                raise
            result["arm_seconds_remaining"] = round(remaining, 1)
            self.log("action", {"action": action, "value": value, "result": result})
            return result

    def go_home(self):
        """Operator-driven return to the configured home pose.

        Deliberately not an MCP tool and deliberately not gated on the arming
        window: this is the harness resetting the rig between trials, not a
        model deciding to move.
        """
        with self.mutex:
            if not self.live:
                raise RuntimeError("Homing requires mode = \"hardware_live\"")
            try:
                result = self._arm().home()
            except Exception as exc:
                self.log("home_failed", {"error": f"{type(exc).__name__}: {exc}"})
                raise
            self.log("home", result)
            return result

    def declare_done(self, success, note=""):
        """Record the model's own end-of-episode claim. Grading never trusts it."""
        record = {"claimed_success": bool(success), "note": str(note)[:2000]}
        with self.mutex:
            self.log("episode_done", record)
        return {**record, "recorded": True,
                "note_to_model": "Recorded. A human grader reviews the video and decides the outcome."}

    def close(self):
        with self.mutex:
            try:
                self.log("session_end", {})
            finally:
                self.stack.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
