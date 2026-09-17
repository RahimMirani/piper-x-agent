from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from piper_agent.arm import MockArm, ReadOnlyArm
from piper_agent.cameras import Frame
from piper_agent.config import Config
from piper_agent.runtime import ProcessLock, Runtime


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = Config("mock", self.root / "runs", "can0", "v188", "scene", "wrist")

    def test_bad_motion_is_rejected_without_state_change(self):
        arm = MockArm()
        for target in ([0] * 5, [float("nan")] * 6, [float("inf")] * 6, [True] * 6, [1] * 6):
            with self.subTest(target=target), self.assertRaises(ValueError):
                arm.move(target)
        self.assertEqual(arm.state()["joints_rad"], [0] * 6)
        arm.move([0.05] * 6)
        arm.stop()
        with self.assertRaises(RuntimeError):
            arm.move([0] * 6)
        with self.assertRaises(RuntimeError):
            arm.grip(0.02)

    def test_hardware_simulation_rejected_before_any_sdk_import(self):
        with Runtime(replace(self.config, mode="hardware_readonly")) as runtime:
            for action in ("move", "grip", "stop"):
                with self.assertRaises(RuntimeError):
                    runtime.simulate(action, [0] * 6)
            self.assertIsNone(runtime.arm)

    def test_observations_persist_two_distinct_pngs_and_metadata(self):
        with Runtime(self.config) as runtime:
            metadata, frames = runtime.observe()
            self.assertEqual([f.role for f in frames], ["scene", "wrist"])
            self.assertTrue(metadata["mock_images_are_diagnostic_patterns"])
            self.assertFalse(metadata["synchronized"])
            self.assertNotEqual(frames[0].data, frames[1].data)
            for f, record in zip(frames, metadata["frames"]):
                self.assertTrue(f.data.startswith(b"\x89PNG"))
                self.assertEqual((runtime.directory / record["file"]).read_bytes(), f.data)
            records = [json.loads(s) for s in (runtime.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual(records[-1]["event"], "observation")

    def test_stale_frame_rejected(self):
        with Runtime(self.config) as runtime:
            cameras = runtime._cameras()
            frames = [Frame(role, role, b"x", "image/png", time.monotonic() - 5, time.time())
                      for role in ("scene", "wrist")]
            with patch.object(cameras, "capture", return_value=frames):
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    runtime.observe()
            self.assertEqual(runtime.sequence, 0)

    def test_lock_excludes_second_process_owner_then_releases(self):
        path = self.root / "test.lock"
        with ProcessLock(path):
            with self.assertRaises(RuntimeError):
                with ProcessLock(path):
                    self.fail("Second lock acquired")
        with ProcessLock(path):
            pass

    def test_config_rejects_unknown_mode_duplicate_serial_and_nan(self):
        path = self.root / "test.toml"
        for text in (
            'mode="live"\n[cameras]\nscene_serial="a"\nwrist_serial="b"',
            'mode="mock"\n[cameras]\nscene_serial="a"\nwrist_serial="a"',
            'mode="mock"\ncamera_max_age_s=nan\n[cameras]\nscene_serial="a"\nwrist_serial="b"',
            'mode="mock"\nallow_motion=true\n[cameras]\nscene_serial="a"\nwrist_serial="b"',
        ):
            path.write_text(text)
            with self.subTest(text=text), self.assertRaises(ValueError):
                Config.load(path)

    def test_readonly_adapter_uses_only_connection_and_feedback(self):
        calls = []
        class FakeRobot:
            sequence = 0
            def connect(self): calls.append("connect")
            def disconnect(self): calls.append("disconnect")
            def get_joint_angles(self):
                calls.append("get_joint_angles")
                self.sequence += 1
                return types.SimpleNamespace(msg=[0] * 6, timestamp=self.sequence)
        sdk = types.ModuleType("pyAgxArm")
        sdk.ArmModel = types.SimpleNamespace(PIPER_X="piper_x")
        sdk.AgxArmFactory = types.SimpleNamespace(create_arm=lambda cfg: FakeRobot())
        sdk.create_agx_arm_config = lambda **kw: kw
        with patch.dict(sys.modules, {"pyAgxArm": sdk}):
            arm = ReadOnlyArm(self.config)
            self.assertEqual(arm.state()["joints_rad"], [0] * 6)
            for method, args in ((arm.move, ([0] * 6,)), (arm.grip, (0.01,)), (arm.stop, ())):
                with self.assertRaises(RuntimeError):
                    method(*args)
            arm.close()
        self.assertEqual(calls, ["connect", "get_joint_angles", "get_joint_angles", "disconnect"])

    def test_readonly_adapter_rejects_frozen_feedback(self):
        arm = ReadOnlyArm.__new__(ReadOnlyArm)
        arm.robot = types.SimpleNamespace(get_joint_angles=lambda: types.SimpleNamespace(msg=[0] * 6, timestamp=1))
        with patch("piper_agent.arm.time.monotonic", side_effect=[0, 0.1, 0.2, 3]), patch("piper_agent.arm.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "advancing"):
                arm.state()


if __name__ == "__main__":
    unittest.main()
