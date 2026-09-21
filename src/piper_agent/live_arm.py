"""Bounded physical control for supervised evaluation episodes.

Every command is absolute, refused rather than clamped when out of range,
bounded by a per-call step limit, and verified against feedback published after
the command was sent. Nothing here is collision-aware: the limits keep a single
call small, they do not know where the table is.
"""

import time

from .sdk_motion import (assert_no_faults, connect_arm, enable_and_baseline,
                         flange_pose_feedback, gripper_sample, joint_feedback,
                         require_in_joint_limits, require_valid_pose_angles,
                         validate_six, wait_pose_target, wait_target)

_MOVE_TIMEOUT_S = 15.0
_JOINT_TOLERANCE_RAD = 0.01
_POSITION_TOLERANCE_M = 0.005
_ANGLE_TOLERANCE_RAD = 0.05
_GRIPPER_SETTLE_S = 1.2
_GRIPPER_MAX_WIDTH_M = 0.07
_GRIPPER_MAX_FORCE_N = 2.0


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
            gripper = gripper_sample(self._effector(), "while reading state")
        except (TimeoutError, RuntimeError):
            gripper = None
        return {"source": "hardware_live",
                "joints_rad": joints,
                "flange_pose": pose,
                "gripper_aperture_m": gripper["measured_width_m"] if gripper else None,
                "gripper_homed": gripper["homed"] if gripper else None,
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
        if step > self.limits.max_cartesian_step_m:
            raise ValueError(
                f"Refusing a {step:.4f} m step; this call may move the flange at most "
                f"{self.limits.max_cartesian_step_m:.4f} m. Command a closer pose and repeat."
            )
        assert_no_faults(self.robot, "before a commanded Cartesian move")
        self.robot.set_motion_mode("l")
        self.robot.move_l(target)
        try:
            reached = wait_pose_target(self.robot, target, _MOVE_TIMEOUT_S,
                                       position_tolerance=_POSITION_TOLERANCE_M,
                                       angle_tolerance=_ANGLE_TOLERANCE_RAD)
        except BaseException:
            self._damped_stop()
            raise
        joints, _ = joint_feedback(self.robot, 2.0)
        return {"commanded_pose": target,
                "start_pose": start,
                "measured_pose": reached["flange_pose"],
                "measured_joints_rad": joints,
                "max_position_error_m": reached["max_position_error_m"],
                "max_angle_error_rad": reached["max_angle_error_rad"],
                "requested_step_m": step,
                "speed_percent": self.limits.speed_percent}

    def set_gripper(self, width_m, force_n=1.0):
        for name, value, upper in (("width_m", width_m, _GRIPPER_MAX_WIDTH_M),
                                   ("force_n", force_n, _GRIPPER_MAX_FORCE_N)):
            if type(value) not in (int, float) or isinstance(value, bool) or not 0 <= value <= upper:
                raise ValueError(f"{name} must be a number between 0 and {upper}")
        effector = self._effector()
        before = gripper_sample(effector, "before commanding the gripper")
        effector.move_gripper_m(value=float(width_m), force=float(force_n))
        time.sleep(_GRIPPER_SETTLE_S)
        after = gripper_sample(effector, "after commanding the gripper")
        return {"commanded_width_m": float(width_m), "commanded_force_n": float(force_n),
                "start_width_m": before["measured_width_m"],
                "measured_width_m": after["measured_width_m"],
                "measured_force_n": after["measured_force_n"],
                "homed": after["homed"],
                "absolute_width_verified": after["homed"]}

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
