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
from piper_agent.gripper_test import _gripper_feedback, _sample
from piper_agent.motion_test import _commandable_pose, _joint_feedback
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

    def test_camera_snapshot_does_not_connect_arm(self):
        with Runtime(self.config) as runtime:
            metadata, frames = runtime.observe(include_arm=False)
            self.assertIsNone(runtime.arm)
            self.assertIsNone(metadata["state"])
            self.assertEqual(len(frames), 2)

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


class MotionGuardTests(unittest.TestCase):
    """Guards around the supervised hardware tests; no SDK and no hardware."""

    def test_small_boundary_excursion_is_clamped_and_reported(self):
        # The arm rests a couple of degrees past the joint 2 and joint 3
        # boundaries with motors off. move_j clamps silently, so the test must
        # command the clamped pose knowingly.
        measured = [0.0856, -0.0383, 0.0403, 0.6324, 0.0254, -0.0808]
        pose, shift = _commandable_pose(measured)
        self.assertEqual(pose[1], 0.0)
        self.assertEqual(pose[2], 0.0)
        self.assertAlmostEqual(shift[1], 0.0383)
        self.assertAlmostEqual(shift[2], -0.0403)
        self.assertEqual([shift[i] for i in (0, 3, 4, 5)], [0.0] * 4)

    def test_pose_far_outside_limits_is_refused(self):
        measured = [0.0, -0.4, 0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(RuntimeError, "joint 2"):
            _commandable_pose(measured)

    def test_joint_feedback_requires_a_frame_published_during_the_call(self):
        # A cached frame can predate the command under test; only an advancing
        # timestamp proves the reading is new.
        stamps = iter([5, 5, 5, 7])
        robot = types.SimpleNamespace(
            get_joint_angles=lambda: types.SimpleNamespace(msg=[0.5] * 6, timestamp=next(stamps)))
        with patch("piper_agent.motion_test.time.sleep"):
            joints, stamp = _joint_feedback(robot, 2.0)
        self.assertEqual((joints, stamp), ([0.5] * 6, 7))

    def test_joint_feedback_rejects_frozen_feedback(self):
        robot = types.SimpleNamespace(
            get_joint_angles=lambda: types.SimpleNamespace(msg=[0.0] * 6, timestamp=1))
        with patch("piper_agent.motion_test.time.monotonic", side_effect=[0, 0.1, 0.2, 9]), \
                patch("piper_agent.motion_test.time.sleep"):
            with self.assertRaises(TimeoutError):
                _joint_feedback(robot, 2.0)

    def test_gripper_feedback_rejects_frozen_feedback(self):
        effector = types.SimpleNamespace(
            get_gripper_status=lambda: types.SimpleNamespace(msg=None, timestamp=1))
        with patch("piper_agent.gripper_test.time.monotonic", side_effect=[0, 0.1, 0.2, 9]), \
                patch("piper_agent.gripper_test.time.sleep"):
            with self.assertRaises(TimeoutError):
                _gripper_feedback(effector)

    def test_gripper_sample_rejects_driver_faults(self):
        def status(**flags):
            foc = types.SimpleNamespace(voltage_too_low=False, motor_overheating=False,
                                        driver_overcurrent=False, driver_overheating=False,
                                        sensor_status=False, driver_error_status=False,
                                        driver_enable_status=True, homing_status=True)
            for name, value in flags.items():
                setattr(foc, name, value)
            return types.SimpleNamespace(
                msg=types.SimpleNamespace(value=0.02, force=0.1, foc_status=foc), timestamp=1)

        with patch("piper_agent.gripper_test._gripper_feedback", return_value=status()):
            self.assertEqual(_sample(None, "now")["measured_width_m"], 0.02)
        for flags, pattern in (({"driver_overcurrent": True}, "driver_overcurrent"),
                               ({"driver_enable_status": False}, "not enabled")):
            with patch("piper_agent.gripper_test._gripper_feedback", return_value=status(**flags)):
                with self.subTest(flags=flags), self.assertRaisesRegex(RuntimeError, pattern):
                    _sample(None, "now")


if __name__ == "__main__":
    unittest.main()
