"""Explicit, supervised PiPER-X motion smoke test.

This module is intentionally separate from the MCP server. It only performs a
single-joint offset and return, or a small lateral sweep, after an explicit CLI
confirmation flag. Shared SDK behaviour lives in sdk_motion.
"""

import math

from .sdk_motion import (commandable_pose, connect_arm, enable_and_baseline,
                         wait_target)

# Raised from 0.02 rad (1.15 deg, barely visible) to 0.2 rad (11.5 deg) so a
# supervised operator can see the arm move. This is a bounded smoke test in
# clear space at _SPEED_PERCENT, not a licence for planned trajectories: there
# is still no collision checking here.
_MAX_DELTA_RAD = 0.2

_SPEED_PERCENT = 10

_HARDWARE_MODES = {"hardware_readonly", "hardware_live"}


def _tolerance_for(delta):
    return min(0.01, max(0.001, abs(float(delta)) * 0.25))


def _check_delta(delta):
    if type(delta) not in (int, float) or not math.isfinite(delta) or not 0 < abs(delta) <= _MAX_DELTA_RAD:
        raise ValueError(f"delta must be finite, nonzero, and at most {_MAX_DELTA_RAD} rad")


def run_single_joint_test(config, joint=1, delta=0.02, timeout=8.0):
    if config.mode not in _HARDWARE_MODES:
        raise ValueError("Motion test requires a hardware configuration")
    if type(joint) is not int or not 1 <= joint <= 6:
        raise ValueError("joint must be an integer from 1 to 6")
    _check_delta(delta)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 2 <= timeout <= 30:
        raise ValueError("timeout must be between 2 and 30 seconds")

    robot = connect_arm(config)
    try:
        robot.connect()
        measured, base, shift, start_stamp = enable_and_baseline(robot, _SPEED_PERCENT)
        target = base.copy()
        target[joint - 1] += float(delta)
        if any(abs(value) > 3.1 for value in target):
            raise ValueError("Refusing target too close to generic +/-3.1 rad safety envelope")
        commandable_pose(target)

        robot.set_motion_mode("j")
        tolerance = _tolerance_for(delta)

        robot.move_j(target)
        outward = wait_target(robot, target, timeout, tolerance=tolerance)
        observed_delta = abs(outward["joints_rad"][joint - 1] - base[joint - 1])
        if observed_delta < abs(float(delta)) * 0.75:
            raise RuntimeError(
                f"Commanded joint did not visibly move: requested={float(delta):.6f} "
                f"observed={observed_delta:.6f} rad"
            )
        robot.move_j(base)
        returned = wait_target(robot, base, timeout, tolerance=tolerance)
        return {"joint": joint, "delta_rad": float(delta), "speed_percent": _SPEED_PERCENT,
                "measured_start_joints_rad": measured,
                "commanded_start_joints_rad": base,
                "sdk_clamp_shift_rad": shift,
                "target_joints_rad": target,
                "returned_joints_rad": returned["joints_rad"],
                "observed_delta_rad": observed_delta,
                "start_timestamp": start_stamp, "outward": outward,
                "return": returned, "physical_motion_supported": True}
    except BaseException:
        # If a commanded trajectory fails, request the vendor's damped stop.
        # Leave the arm enabled/holding; disabling a raised arm can drop it.
        try:
            robot.electronic_emergency_stop()
        except Exception:
            pass
        raise
    finally:
        robot.disconnect()


def run_lateral_sweep(config, delta=0.02, timeout=8.0):
    """Yaw the whole arm left/right around its current pose, then return."""
    if config.mode not in _HARDWARE_MODES:
        raise ValueError("Lateral test requires a hardware configuration")
    _check_delta(delta)

    robot = connect_arm(config)
    try:
        robot.connect()
        measured, base, shift, _ = enable_and_baseline(robot, _SPEED_PERCENT)
        left, right = base.copy(), base.copy()
        left[0] -= float(delta)
        right[0] += float(delta)
        for pose in (left, right):
            commandable_pose(pose)

        robot.set_motion_mode("j")
        tolerance = _tolerance_for(delta)
        legs = {}
        for name, pose in (("left", left), ("right", right), ("return", base)):
            robot.move_j(pose)
            legs[name] = wait_target(robot, pose, timeout, tolerance=tolerance)
        swept = abs(legs["right"]["joints_rad"][0] - legs["left"]["joints_rad"][0])
        if swept < abs(float(delta)) * 1.5:
            raise RuntimeError(
                f"Joint 1 did not sweep both ways: requested={2 * abs(float(delta)):.6f} "
                f"observed={swept:.6f} rad"
            )
        return {"measured_base_joints_rad": measured,
                "commanded_base_joints_rad": base,
                "sdk_clamp_shift_rad": shift,
                "left": legs["left"], "right": legs["right"], "return": legs["return"],
                "observed_sweep_rad": swept, "delta_rad": float(delta),
                "speed_percent": _SPEED_PERCENT, "physical_motion_supported": True}
    except BaseException:
        try:
            robot.electronic_emergency_stop()
        except Exception:
            pass
        raise
    finally:
        robot.disconnect()
