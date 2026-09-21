"""Bounded physical control for supervised evaluation episodes.

Every command is absolute, refused rather than clamped when out of range,
bounded by a per-call step limit, and verified against feedback published after
the command was sent. Nothing here is collision-aware: the limits keep a single
call small, they do not know where the table is.
"""

import time

from .sdk_motion import (assert_no_faults, commandable_pose, connect_arm,
                         enable_and_baseline, flange_pose_feedback, gripper_observation,
                         gripper_sample, joint_feedback, require_in_joint_limits,
                         require_valid_pose_angles, validate_six, wait_pose_target,
                         wait_target)

_MOVE_TIMEOUT_S = 15.0
_JOINT_TOLERANCE_RAD = 0.01
_POSITION_TOLERANCE_M = 0.005
_ANGLE_TOLERANCE_RAD = 0.05
_GRIPPER_SETTLE_S = 1.2
_GRIPPER_MAX_WIDTH_M = 0.07
_GRIPPER_MAX_FORCE_N = 2.0
_HOME_TOLERANCE_RAD = 0.02


class LiveArm:
    """Physical arm control. Enables motors on the first motion command only."""

    def __init__(self, config):
        self.config = config
        self.limits = config.live
        self.robot = connect_arm(config)
        self.effector = None
        self.ready = False
        self.enable_baseline = None
        try:
            self.robot.connect()
        except BaseException:
            self.robot.disconnect()
            raise

    # -- lifecycle -------------------------------------------------------

    def _ensure_ready(self):
        """Enable and baseline once, on the first motion command.

        Reading state never enables the arm, so starting a server or taking an
        observation leaves the motors exactly as the operator left them.
        """
        if not self.ready:
            measured, base, shift, _ = enable_and_baseline(self.robot, self.limits.speed_percent)
            self.enable_baseline = {"measured_joints_rad": measured,
                                    "commanded_joints_rad": base,
                                    "sdk_clamp_shift_rad": shift}
            self.ready = True
        return self.enable_baseline

    def _effector(self):
        if self.effector is None:
            # Registers a driver only; sends no CAN command.
            self.effector = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        return self.effector

    def close(self):
        # Do not disable: an unsupported arm can drop under gravity.
        self.robot.disconnect()

    # -- observation -----------------------------------------------------

    def state(self):
        joints, stamp = joint_feedback(self.robot, 2.0)
        try:
            pose, _ = flange_pose_feedback(self.robot, 1.0)
        except TimeoutError:
            pose = None
        try:
            gripper = gripper_observation(self._effector())
        except Exception as exc:
            gripper = {"available": False, "reason": f"{type(exc).__name__}: {exc}"}
        return {"source": "hardware_live",
                "joints_rad": joints,
                "flange_pose": pose,
                "gripper": gripper,
                "gripper_aperture_m": gripper.get("measured_width_m"),
                "gripper_homed": gripper.get("homed"),
                "motors_enabled": self.ready,
                "sdk_timestamp": stamp,
                "sampled_at_unix_s": time.time(),
                "physical_motion_supported": True}

    # -- motion ----------------------------------------------------------

    def move_joints(self, joints_rad):
        target = validate_six(joints_rad, "joints_rad")
        require_in_joint_limits(target)
        self._ensure_ready()
        start, _ = joint_feedback(self.robot, 2.0)
        step = max(abs(a - b) for a, b in zip(target, start))
        if step > self.limits.max_joint_step_rad:
            raise ValueError(
                f"Refusing a {step:.4f} rad step; this call may move any joint at most "
                f"{self.limits.max_joint_step_rad:.4f} rad. Command a closer pose and repeat."
            )
        assert_no_faults(self.robot, "before a commanded joint move")
        self.robot.set_motion_mode("j")
        self.robot.move_j(target)
        try:
            reached = wait_target(self.robot, target, _MOVE_TIMEOUT_S,
                                  tolerance=_JOINT_TOLERANCE_RAD)
        except BaseException:
            self._damped_stop()
            raise
        return {"commanded_joints_rad": target,
                "start_joints_rad": start,
                "measured_joints_rad": reached["joints_rad"],
                "max_error_rad": reached["max_error_rad"],
                "requested_step_rad": step,
                "speed_percent": self.limits.speed_percent}

    def move_to_pose(self, pose):
        target = validate_six(pose, "pose")
        require_valid_pose_angles(target)
        bounds = self.limits.workspace_bounds()
        if bounds is None:
            raise RuntimeError(
                "move_to_pose is unavailable: set workspace_min_m and workspace_max_m "
                "in the [live] configuration so Cartesian motion has a bounded box."
            )
        low, high = bounds
        for axis, value, lower, upper in zip("xyz", target[:3], low, high):
            if not lower <= value <= upper:
                raise ValueError(
                    f"Pose {axis}={value:.4f} m is outside the configured workspace "
                    f"[{lower:.4f}, {upper:.4f}] m. The command was refused."
                )
        self._ensure_ready()
        start, _ = flange_pose_feedback(self.robot, 2.0)
        step = max(abs(a - b) for a, b in zip(target[:3], start[:3]))
        # Compare with a micrometre of slack: a step written as exactly the
        # limit lands a few float ULPs above it and would be refused.
        if step > self.limits.max_cartesian_step_m + 1e-9:
            raise ValueError(
                f"Refusing a {step:.4f} m step; this call may move the flange at most "
                f"{self.limits.max_cartesian_step_m:.4f} m. Command a closer pose and repeat."
            )
        assert_no_faults(self.robot, "before a commanded Cartesian move")
        start_joints, _ = joint_feedback(self.robot, 2.0)
        self.robot.set_motion_mode("l")
        self.robot.move_l(target)
        # Scale the band to the step. A flat 5 mm tolerance lets a 10 mm command
        # that travelled half way report success, which is the same false pass
        # that the joint path already guards against.
        tolerance = min(_POSITION_TOLERANCE_M, max(0.0005, step * 0.25))
        try:
            reached = wait_pose_target(self.robot, target, _MOVE_TIMEOUT_S,
                                       position_tolerance=tolerance,
                                       angle_tolerance=_ANGLE_TOLERANCE_RAD,
                                       start_joints=start_joints,
                                       max_joint_excursion_rad=self.limits.max_joint_step_rad,
                                       start_pose=start)
        except BaseException:
            self._damped_stop()
            raise
        joints, _ = joint_feedback(self.robot, 2.0)
        return {"commanded_pose": target,
                "start_pose": start,
                "measured_pose": reached["flange_pose"],
                "start_joints_rad": start_joints,
                "measured_joints_rad": joints,
                "joint_excursion_rad": max(abs(a - b) for a, b in zip(joints, start_joints)),
                "max_position_error_m": reached["max_position_error_m"],
                "max_angle_error_rad": reached["max_angle_error_rad"],
                "controller_motion_status": reached["controller_motion_status"],
                "requested_step_m": step,
                "measured_step_m": max(abs(a - b) for a, b in
                                       zip(reached["flange_pose"][:3], start[:3])),
                "position_tolerance_m": tolerance,
                "speed_percent": self.limits.speed_percent}

    def set_gripper(self, width_m, force_n=1.0):
        for name, value, upper in (("width_m", width_m, _GRIPPER_MAX_WIDTH_M),
                                   ("force_n", force_n, _GRIPPER_MAX_FORCE_N)):
            if type(value) not in (int, float) or isinstance(value, bool) or not 0 <= value <= upper:
                raise ValueError(f"{name} must be a number between 0 and {upper}")
        effector = self._effector()
        # Read, do not require, the driver state beforehand: the move message
        # carries the enable bit, so a gripper idle since power-up is disabled
        # until the first command. The check after the move is strict, so a
        # gripper that failed to come up still fails loudly.
        before = gripper_observation(effector)
        effector.move_gripper_m(value=float(width_m), force=float(force_n))
        time.sleep(_GRIPPER_SETTLE_S)
        after = gripper_sample(effector, "after commanding the gripper")
        return {"commanded_width_m": float(width_m), "commanded_force_n": float(force_n),
                "start_width_m": before.get("measured_width_m"),
                "driver_enabled_before": before.get("driver_enabled"),
                "measured_width_m": after["measured_width_m"],
                "measured_force_n": after["measured_force_n"],
                "homed": after["homed"],
                "absolute_width_verified": after["homed"]}

    # -- homing ----------------------------------------------------------

    def home(self, timeout_s=60.0):
        """Walk to the configured home pose in bounded steps.

        Trials have to start from the same configuration or the difference
        between two models is buried under where the previous episode happened
        to stop. Home can be far away, so approach it in steps no larger than
        the per-call budget rather than commanding one long move.
        """
        target = self.limits.home_joints_rad
        if target is None:
            raise RuntimeError(
                "No home pose configured: set home_joints_rad in the [live] section."
            )
        target = list(target)
        require_in_joint_limits(target)
        # Clear a latched stop or a rejected-target state first. Homing is the
        # operator's recovery path, so it must work from a faulted arm rather
        # than refusing on the fault it is there to clear.
        self.robot.reset()
        time.sleep(1.0)
        self._ensure_ready()
        self.robot.set_motion_mode("j")
        legs = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            current, _ = joint_feedback(self.robot, 2.0)
            remaining = max(abs(a - b) for a, b in zip(target, current))
            if remaining <= _HOME_TOLERANCE_RAD:
                return {"home_joints_rad": target, "measured_joints_rad": current,
                        "legs": legs, "reached": True,
                        "final_error_rad": remaining}
            scale = min(1.0, self.limits.max_joint_step_rad / remaining)
            waypoint = [c + (t - c) * scale for c, t in zip(current, target)]
            # The interpolated point can sit a hair outside a boundary when the
            # arm is resting just past one; clamp it the same way a baseline is.
            waypoint, _ = commandable_pose(waypoint)
            assert_no_faults(self.robot, "while homing")
            self.robot.move_j(waypoint)
            try:
                reached = wait_target(self.robot, waypoint, _MOVE_TIMEOUT_S,
                                      tolerance=_JOINT_TOLERANCE_RAD)
            except BaseException:
                self._damped_stop()
                raise
            legs.append({"waypoint_rad": waypoint,
                         "max_error_rad": reached["max_error_rad"],
                         "remaining_rad": remaining})
        raise TimeoutError(f"Did not reach the home pose within {timeout_s:.0f}s; legs={len(legs)}")

    # -- stopping --------------------------------------------------------

    def _damped_stop(self):
        try:
            self.robot.electronic_emergency_stop()
        except Exception:
            pass

    def stop(self):
        """Vendor damped stop. This is software, and not a hardware E-stop."""
        self._damped_stop()
        joints, _ = joint_feedback(self.robot, 2.0)
        return {"stopped": True, "measured_joints_rad": joints,
                "is_hardware_estop": False,
                "note": "Damped software stop requested. Use the physical cutoff for emergencies."}
