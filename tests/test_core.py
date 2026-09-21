from dataclasses import replace
import math
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
from piper_agent import arming
from piper_agent.config import Config, LiveLimits
from piper_agent.live_arm import LiveArm
from piper_agent.runtime import ProcessLock, Runtime
from piper_agent.sdk_motion import (commandable_pose, gripper_feedback, gripper_observation,
                                    gripper_sample, joint_feedback, require_in_joint_limits)


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
        pose, shift = commandable_pose(measured)
        self.assertEqual(pose[1], 0.0)
        self.assertEqual(pose[2], 0.0)
        self.assertAlmostEqual(shift[1], 0.0383)
        self.assertAlmostEqual(shift[2], -0.0403)
        self.assertEqual([shift[i] for i in (0, 3, 4, 5)], [0.0] * 4)

    def test_pose_far_outside_limits_is_refused(self):
        measured = [0.0, -0.4, 0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(RuntimeError, "joint 2"):
            commandable_pose(measured)

    def test_joint_feedback_requires_a_frame_published_during_the_call(self):
        # A cached frame can predate the command under test; only an advancing
        # timestamp proves the reading is new.
        stamps = iter([5, 5, 5, 7])
        robot = types.SimpleNamespace(
            get_joint_angles=lambda: types.SimpleNamespace(msg=[0.5] * 6, timestamp=next(stamps)))
        with patch("piper_agent.sdk_motion.time.sleep"):
            joints, stamp = joint_feedback(robot, 2.0)
        self.assertEqual((joints, stamp), ([0.5] * 6, 7))

    def test_joint_feedback_rejects_frozen_feedback(self):
        robot = types.SimpleNamespace(
            get_joint_angles=lambda: types.SimpleNamespace(msg=[0.0] * 6, timestamp=1))
        with patch("piper_agent.sdk_motion.time.monotonic", side_effect=[0, 0.1, 0.2, 9]), \
                patch("piper_agent.sdk_motion.time.sleep"):
            with self.assertRaises(TimeoutError):
                joint_feedback(robot, 2.0)

    def test_gripper_feedback_rejects_frozen_feedback(self):
        effector = types.SimpleNamespace(
            get_gripper_status=lambda: types.SimpleNamespace(msg=None, timestamp=1))
        with patch("piper_agent.sdk_motion.time.monotonic", side_effect=[0, 0.1, 0.2, 9]), \
                patch("piper_agent.sdk_motion.time.sleep"):
            with self.assertRaises(TimeoutError):
                gripper_feedback(effector)

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

        with patch("piper_agent.sdk_motion.gripper_feedback", return_value=status()):
            self.assertEqual(gripper_sample(None, "now")["measured_width_m"], 0.02)
        for flags, pattern in (({"driver_overcurrent": True}, "driver_overcurrent"),
                               ({"driver_enable_status": False}, "not enabled")):
            with patch("piper_agent.sdk_motion.gripper_feedback", return_value=status(**flags)):
                with self.subTest(flags=flags), self.assertRaisesRegex(RuntimeError, pattern):
                    gripper_sample(None, "now")

    def test_gripper_observation_reports_a_disabled_driver_instead_of_hiding_it(self):
        # After a power cycle the driver reads disabled until the first move
        # command. A read must say so rather than return an empty width that
        # looks identical to a wiring fault.
        foc = types.SimpleNamespace(voltage_too_low=False, motor_overheating=False,
                                    driver_overcurrent=False, driver_overheating=False,
                                    sensor_status=False, driver_error_status=False,
                                    driver_enable_status=False, homing_status=False)
        frame = types.SimpleNamespace(
            msg=types.SimpleNamespace(value=0.001, force=0.0, foc_status=foc), timestamp=1)
        with patch("piper_agent.sdk_motion.gripper_feedback", return_value=frame):
            observation = gripper_observation(None)
        self.assertTrue(observation["available"])
        self.assertFalse(observation["driver_enabled"])
        self.assertEqual(observation["measured_width_m"], 0.001)

        with patch("piper_agent.sdk_motion.gripper_feedback", side_effect=TimeoutError("no frames")):
            unavailable = gripper_observation(None)
        self.assertFalse(unavailable["available"])
        self.assertIn("no frames", unavailable["reason"])


class ArmingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        path = Path(self.temp.name) / "arm.json"
        patcher = patch("piper_agent.arming.arm_path", return_value=path)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.path = path

    def test_unarmed_by_default_and_motion_is_refused(self):
        self.assertEqual(arming.status()[0], False)
        with self.assertRaisesRegex(RuntimeError, "not armed"):
            arming.require_armed()

    def test_arming_expires_on_its_own(self):
        arming.arm(1)
        self.assertTrue(arming.status()[0])
        self.assertGreater(arming.require_armed(), 0)
        with patch("piper_agent.arming.time.time", return_value=time.time() + 3600):
            self.assertFalse(arming.status()[0])
            with self.assertRaises(RuntimeError):
                arming.require_armed()

    def test_disarm_and_window_bounds(self):
        arming.arm(5)
        arming.disarm()
        self.assertFalse(arming.status()[0])
        for minutes in (0, -1, arming.MAX_MINUTES + 1, True, "10"):
            with self.subTest(minutes=minutes), self.assertRaises(ValueError):
                arming.arm(minutes)

    def test_corrupt_window_file_reads_as_unarmed(self):
        self.path.write_text("not json")
        self.assertFalse(arming.status()[0])


class LiveLimitsTests(unittest.TestCase):
    def test_ceilings_cannot_be_raised_from_configuration(self):
        for data in ({"speed_percent": 100}, {"speed_percent": 0},
                     {"max_joint_step_rad": 2.0}, {"max_joint_step_rad": 0},
                     {"max_cartesian_step_m": 1.0},
                     {"workspace_min_m": [0.1, 0.1, 0.1]},
                     {"workspace_min_m": [0.5, 0, 0], "workspace_max_m": [0.1, 1, 1]},
                     {"unknown": 1}):
            with self.subTest(data=data), self.assertRaises(ValueError):
                LiveLimits.load(data)

    def test_workspace_requires_both_bounds(self):
        limits = LiveLimits.load({})
        self.assertIsNone(limits.workspace_bounds())
        limits = LiveLimits.load({"workspace_min_m": [0, -1, 0], "workspace_max_m": [1, 1, 1]})
        self.assertEqual(limits.workspace_bounds(), ((0.0, -1.0, 0.0), (1.0, 1.0, 1.0)))

    def test_live_section_rejected_outside_live_mode(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "c.toml"
            path.write_text('mode="hardware_readonly"\n[live]\nspeed_percent=10\n'
                            '[cameras]\nscene_serial="a"\nwrist_serial="b"')
            with self.assertRaisesRegex(ValueError, "hardware_live"):
                Config.load(path)


class LiveArmBoundsTests(unittest.TestCase):
    """Bounds checks run before anything is sent to the arm."""

    def _arm(self, **limit_kwargs):
        arm = LiveArm.__new__(LiveArm)
        arm.limits = LiveLimits(**limit_kwargs)
        arm.ready = True
        arm.enable_baseline = {}
        arm.sent = []
        arm.robot = types.SimpleNamespace(
            move_j=lambda t: arm.sent.append(("move_j", t)),
            move_l=lambda t: arm.sent.append(("move_l", t)),
            set_motion_mode=lambda m: None,
            reset=lambda: arm.sent.append(("reset", [])))
        return arm

    def test_out_of_range_joint_target_is_refused_not_clamped(self):
        arm = self._arm()
        # Joint 2's range is [0, pi]; -0.5 must be refused rather than
        # silently becoming 0.0 the way the SDK would clamp it.
        with self.assertRaisesRegex(ValueError, "refused, not clamped"):
            arm.move_joints([0.0, -0.5, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(arm.sent, [])

    def test_oversized_joint_step_is_refused(self):
        arm = self._arm(max_joint_step_rad=0.1)
        with patch("piper_agent.live_arm.joint_feedback", return_value=([0.0] * 6, 1)):
            with self.assertRaisesRegex(ValueError, "Refusing a"):
                arm.move_joints([0.9, 0.1, -0.1, 0.0, 0.0, 0.0])
        self.assertEqual(arm.sent, [])

    def test_cartesian_motion_unavailable_without_a_workspace(self):
        arm = self._arm()
        with self.assertRaisesRegex(RuntimeError, "workspace"):
            arm.move_to_pose([0.2, 0.0, 0.2, 0.0, 0.0, 0.0])
        self.assertEqual(arm.sent, [])

    def test_pose_outside_workspace_is_refused(self):
        arm = self._arm(workspace_min_m=(0.0, -0.2, 0.0), workspace_max_m=(0.4, 0.2, 0.4))
        with self.assertRaisesRegex(ValueError, "outside the configured workspace"):
            arm.move_to_pose([0.9, 0.0, 0.2, 0.0, 0.0, 0.0])
        self.assertEqual(arm.sent, [])

    def test_malformed_and_out_of_band_arguments_are_refused(self):
        arm = self._arm()
        for target in ([0.0] * 5, [float("nan")] * 6, [True] * 6, "six"):
            with self.subTest(target=target), self.assertRaises((ValueError, TypeError)):
                arm.move_joints(target)
        for width in (-0.01, 0.5, float("inf"), True, "wide"):
            with self.subTest(width=width), self.assertRaises(ValueError):
                arm.set_gripper(width)
        self.assertEqual(arm.sent, [])

    def test_cartesian_move_aborts_on_an_oversized_joint_swing(self):
        # A 2 cm lateral step near the base axis measured a 0.36 rad base
        # rotation on hardware, so the Cartesian bound alone is not a joint bound.
        from piper_agent.sdk_motion import wait_pose_target
        robot = types.SimpleNamespace(
            get_arm_status=lambda: None,
            get_joint_angles=lambda: types.SimpleNamespace(msg=[0.9] + [0.0] * 5,
                                                           timestamp=time.time()),
            get_flange_pose=lambda: types.SimpleNamespace(msg=[0.0] * 6, timestamp=time.time()))
        with patch("piper_agent.sdk_motion.assert_no_faults"):
            with self.assertRaisesRegex(RuntimeError, "joint has swung"):
                wait_pose_target(robot, [0.0] * 6, 5.0,
                                 start_joints=[0.0] * 6, max_joint_excursion_rad=0.5)

    def test_cartesian_convergence_ignores_the_latched_failure_flag(self):
        # move_l latches REACH_TARGET_POS_FAILED when it cannot follow the
        # straight line exactly, and the flag survives arrival. Gating on it
        # failed a move that had physically succeeded to within 1.1 mm.
        from piper_agent.sdk_motion import wait_pose_target
        status = types.SimpleNamespace(msg=types.SimpleNamespace(motion_status=1))
        robot = types.SimpleNamespace(
            get_arm_status=lambda: status,
            get_flange_pose=lambda: types.SimpleNamespace(msg=[0.0] * 6, timestamp=time.time()))
        with patch("piper_agent.sdk_motion.assert_no_faults"):
            reached = wait_pose_target(robot, [0.0] * 6, 5.0)
        self.assertEqual(reached["max_position_error_m"], 0.0)
        # Converged anyway, and reported the controller's disagreement.
        self.assertEqual(reached["controller_motion_status"], "1")

    def test_home_requires_configuration_and_walks_in_bounded_steps(self):
        arm = self._arm(max_joint_step_rad=0.2)
        with self.assertRaisesRegex(RuntimeError, "No home pose configured"):
            arm.home()

        # Home can be far from wherever the last episode stopped, so it is
        # approached in steps no larger than the per-call budget.
        arm = self._arm(max_joint_step_rad=0.2, home_joints_rad=(0.5, 0.0, 0.0, 0.0, 0.0, 0.0))
        poses = [[0.0] * 6, [0.2, 0, 0, 0, 0, 0], [0.4, 0, 0, 0, 0, 0], [0.5, 0, 0, 0, 0, 0]]
        with patch("piper_agent.live_arm.joint_feedback", side_effect=[(p, 1) for p in poses]), \
                patch("piper_agent.live_arm.assert_no_faults"), \
                patch("piper_agent.live_arm.wait_target", return_value={"max_error_rad": 0.0}), \
                patch("piper_agent.live_arm.time.sleep"):
            result = arm.home()
        self.assertTrue(result["reached"])
        self.assertEqual(len(result["legs"]), 3)
        self.assertEqual(arm.sent[0][0], "reset")
        for kind, waypoint in arm.sent[1:]:
            self.assertEqual(kind, "move_j")
            self.assertLessEqual(max(abs(v) for v in waypoint), 0.5)

    def test_orientation_comparison_crosses_the_pi_seam(self):
        # Found by a model driving the arm: it requested roll -3.140 and the
        # controller reported +3.14128 -- the same orientation, 0.0019 rad
        # apart. Plain subtraction called it 6.28 rad and no tolerance could
        # ever accept it, so a completed lift timed out.
        from piper_agent.sdk_motion import angle_difference
        self.assertAlmostEqual(abs(angle_difference(-3.140, 3.14128)), 0.00191, places=5)
        self.assertAlmostEqual(angle_difference(0.1, -0.1), 0.2, places=9)
        self.assertAlmostEqual(abs(angle_difference(math.pi, -math.pi)), 0.0, places=9)

    def test_pose_converges_across_the_seam(self):
        from piper_agent.sdk_motion import wait_pose_target
        target = [0.0, 0.0, 0.2, -3.140, 0.0, 0.0]
        robot = types.SimpleNamespace(
            get_arm_status=lambda: None,
            get_flange_pose=lambda: types.SimpleNamespace(
                msg=[0.0, 0.0, 0.2, 3.14128, 0.0, 0.0], timestamp=time.time()))
        with patch("piper_agent.sdk_motion.assert_no_faults"):
            reached = wait_pose_target(robot, target, 5.0)
        self.assertLess(reached["max_angle_error_rad"], 0.01)

    def test_arm_status_faults_are_detected(self):
        # Only err_status bits were checked before, so a collision or a rejected
        # target read as healthy.
        from piper_agent.sdk_motion import arm_faults
        clean = types.SimpleNamespace()
        for name in ("joint_1_angle_limit", "communication_status_joint_1"):
            setattr(clean, name, False)
        for code, expected in ((0x00, []), (0x07, ["arm_status=COLLISION_OCCURRED"]),
                               (0x04, ["arm_status=TARGET_POS_EXCEEDS_LIMIT"])):
            robot = types.SimpleNamespace(get_arm_status=lambda code=code: types.SimpleNamespace(
                msg=types.SimpleNamespace(err_status=clean, arm_status=code, err_code=0)))
            with self.subTest(code=code):
                self.assertEqual(arm_faults(robot), expected)

    def test_unreachable_cartesian_pose_is_reported_not_waited_out(self):
        # From the home corner move_l has no IK solution and simply ignores the
        # command: no motion, no error. Without this the caller burns the whole
        # timeout and then fires a damped stop over a command never accepted.
        from piper_agent.sdk_motion import wait_pose_target
        robot = types.SimpleNamespace(
            get_arm_status=lambda: None,
            get_flange_pose=lambda: types.SimpleNamespace(msg=[0.0] * 6, timestamp=time.time()))
        with patch("piper_agent.sdk_motion.assert_no_faults"), \
                patch("piper_agent.sdk_motion._NO_MOTION_GRACE_S", 0.0):
            with self.assertRaisesRegex(RuntimeError, "did not accept this Cartesian pose"):
                wait_pose_target(robot, [0.0, 0.0, 0.2, 0.0, 0.0, 0.0], 5.0,
                                 start_pose=[0.0] * 6)

    def test_in_range_target_reaches_the_sdk_unchanged(self):
        arm = self._arm()
        target = [0.1, 0.2, -0.3, 0.4, 0.4, 0.4]  # every joint within the default 0.5 rad step
        with patch("piper_agent.live_arm.joint_feedback", return_value=([0.0] * 6, 1)), \
                patch("piper_agent.live_arm.assert_no_faults"), \
                patch("piper_agent.live_arm.wait_target",
                      return_value={"joints_rad": target, "max_error_rad": 0.0}):
            result = arm.move_joints(target)
        self.assertEqual(arm.sent, [("move_j", target)])
        self.assertEqual(result["commanded_joints_rad"], target)


class LiveRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        patcher = patch("piper_agent.arming.arm_path", return_value=self.root / "arm.json")
        patcher.start()
        self.addCleanup(patcher.stop)
        self.config = Config("hardware_live", self.root / "runs", "can0", "v188", "s", "w")

    def test_actions_refused_while_unarmed_and_arm_is_never_constructed(self):
        with Runtime(self.config) as runtime:
            with self.assertRaisesRegex(RuntimeError, "not armed"):
                runtime.act("move_joints", [0.0] * 6)
            self.assertIsNone(runtime.arm)
            self.assertFalse(runtime.status()["motion_armed"])

    def test_mock_mode_never_exposes_physical_actions(self):
        config = replace(self.config, mode="mock")
        with Runtime(config) as runtime:
            with self.assertRaisesRegex(RuntimeError, "hardware_live"):
                runtime.act("move_joints", [0.0] * 6)

    def test_action_budget_stops_an_episode_that_will_not_finish(self):
        arming.arm(5)
        config = replace(self.config, live=LiveLimits(max_actions_per_episode=2))
        fake = types.SimpleNamespace(move_joints=lambda v: {"measured_joints_rad": v})
        with Runtime(config) as runtime:
            with patch.object(Runtime, "_arm", return_value=fake):
                first = runtime.act("move_joints", [0.0] * 6)
                self.assertEqual(first["actions_remaining"], 1)
                runtime.act("move_joints", [0.0] * 6)
                with self.assertRaisesRegex(RuntimeError, "budget exhausted"):
                    runtime.act("move_joints", [0.0] * 6)
            records = [json.loads(s) for s in (runtime.directory / "events.jsonl").read_text().splitlines()]
            # Logged as its own event so grading separates "ran out of budget"
            # from "the model failed at the task".
            self.assertEqual(records[-1]["event"], "budget_exhausted")

    def test_done_is_recorded_without_deciding_the_outcome(self):
        with Runtime(self.config) as runtime:
            result = runtime.declare_done(True, "looks good")
            self.assertTrue(result["claimed_success"])
            self.assertIn("human grader", result["note_to_model"])
            records = [json.loads(s) for s in (runtime.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual(records[-1]["event"], "episode_done")

    def test_unknown_action_is_rejected_and_logged(self):
        arming.arm(5)
        with Runtime(self.config) as runtime:
            with patch.object(Runtime, "_arm", return_value=object()):
                with self.assertRaises(ValueError):
                    runtime.act("launch", None)
            records = [json.loads(s) for s in (runtime.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual(records[-1]["event"], "action_rejected")


if __name__ == "__main__":
    unittest.main()
