"""Explicit, supervised PiPER-X motion smoke test.

This module is intentionally separate from the MCP server. It only performs a
single-joint offset and return, or a small lateral sweep, after an explicit CLI
confirmation flag.
"""

import math
import time

_PIPER_X_LIMITS = ((-2.617994, 2.617994), (0.0, 3.141593),
                   (-2.96706, 0.0), (-1.553344, 1.553344),
                   (-1.553344, 1.553344), (-3.141593, 3.141593))

# The SDK clamps every move_j target into the limits above and only prints a
# warning, so a baseline read even slightly outside a limit silently becomes a
# command that moves that joint to the boundary. The arm rests a couple of
# degrees past the joint 2 and joint 3 boundaries with motors off, so refusing
# outright would block every test. Allow a small excursion, command the clamped
# pose deliberately, and report the shift. Anything larger means the arm model,
# firmware profile or zeroing is wrong and must not be moved.
_CLAMP_TOLERANCE_RAD = 0.05

_SPEED_PERCENT = 10


def _clamp_pose(joints):
    """Return the pose move_j will actually command and the per-joint shift."""
    clamped = [min(max(value, lower), upper)
               for value, (lower, upper) in zip(joints, _PIPER_X_LIMITS)]
    return clamped, [after - before for after, before in zip(clamped, joints)]


def _commandable_pose(joints):
    """Validate a feedback pose and return the pose the SDK will command."""
    clamped, shift = _clamp_pose(joints)
    for index, (value, offset, (lower, upper)) in enumerate(zip(joints, shift, _PIPER_X_LIMITS), 1):
        if abs(offset) > _CLAMP_TOLERANCE_RAD:
            raise RuntimeError(
                f"Refusing motion: joint {index} feedback {value:.6f} rad is "
                f"{abs(offset):.6f} rad outside PiPER-X SDK range "
                f"[{lower:.6f}, {upper:.6f}], and move_j would silently clamp it. "
                "Verify the arm model/firmware zeroing before commanding motion."
            )
    return clamped, shift


def _arm_faults(robot):
    """Return the arm's active fault names, or an empty list."""
    status = robot.get_arm_status()
    if status is None:
        return []
    faults = [name for name, value in vars(status.msg.err_status).items()
              if value is True]
    if int(getattr(status.msg, "err_code", 0) or 0):
        faults.append(f"err_code={int(status.msg.err_code)}")
    return faults


def _assert_no_faults(robot, stage):
    faults = _arm_faults(robot)
    if faults:
        raise RuntimeError(f"Arm reported faults {faults} {stage}")


def _joint_feedback(robot, timeout):
    """Return angles from a frame published during this call.

    The SDK hands back a cached message, so the first readable frame may predate
    the command under test. Require the timestamp to advance before trusting it.
    """
    deadline = time.monotonic() + timeout
    initial = None
    while time.monotonic() < deadline:
        msg = robot.get_joint_angles()
        if msg is not None:
            stamp = msg.timestamp
            if initial is None:
                initial = stamp
            elif stamp > initial:
                return [float(v) for v in msg.msg], stamp
        time.sleep(0.01)
    raise TimeoutError("No advancing joint feedback")


def _stable_joint_feedback(robot, timeout):
    """Return a baseline only after three coherent advancing frames."""
    deadline = time.monotonic() + timeout
    previous = None
    stable = 0
    while time.monotonic() < deadline:
        joints, stamp = _joint_feedback(robot, min(0.5, max(0.05, deadline - time.monotonic())))
        if previous is not None and max(abs(a - b) for a, b in zip(joints, previous)) <= 0.002:
            stable += 1
            if stable >= 3:
                return joints, stamp
        else:
            stable = 0
        previous = joints
    raise TimeoutError("No coherent joint baseline within timeout")


def _wait_target(robot, target, timeout, tolerance=0.01):
    deadline = time.monotonic() + timeout
    last = None
    previous = None
    while time.monotonic() < deadline:
        joints, stamp = _joint_feedback(robot, min(0.5, max(0.05, deadline - time.monotonic())))
        # During an active trajectory this SDK can briefly publish a partially
        # assembled frame with one or more joints at exactly zero. Reject
        # implausible discontinuities instead of treating that frame as real.
        if previous is not None and max(abs(a - b) for a, b in zip(joints, previous)) > 0.25:
            continue
        previous = joints
        last = {"joints_rad": joints, "timestamp": stamp,
                "max_error_rad": max(abs(a - b) for a, b in zip(joints, target))}
        _assert_no_faults(robot, "while converging on a commanded pose")
        status = robot.get_arm_status()
        # motion_status only separates reach-success from reach-failure; it is
        # not a busy flag, so convergence rests on the measured error above.
        motion_done = status is not None and getattr(status.msg, "motion_status", None) == 0
        if motion_done and last["max_error_rad"] <= tolerance:
            return last
    raise TimeoutError(f"Target did not converge within {timeout:.1f}s; last={last}")


def _connect(config):
    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    sdk_config = create_agx_arm_config(robot=ArmModel.PIPER_X,
                                       firmeware_version=config.firmware,
                                       interface="socketcan", channel=config.channel)
    return AgxArmFactory.create_arm(sdk_config)


def _enable_and_baseline(robot):
    """Enable at low speed, wait for a settled pose, return the commandable pose."""
    _assert_no_faults(robot, "before enabling")
    # Explicitly request a low speed. No gripper or firmware/configuration
    # operations are involved.
    robot.set_speed_percent(_SPEED_PERCENT)
    # The SDK's enable() readback can race the six feedback frames. Poll the
    # complete list for up to three seconds; never move on a partial enable.
    robot.enable()
    enabled = False
    statuses = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        statuses = robot.get_joints_enable_status_list()
        if statuses == [True] * 6:
            enabled = True
            break
        time.sleep(0.1)
    if not enabled:
        raise RuntimeError(f"Arm did not report all joints enabled: {statuses!r}")

    # Enabling can let the arm settle a few degrees as torque comes on.
    # Establish the motion baseline only after that transient, so the return
    # command does not ask for the pre-enable pose.
    time.sleep(0.5)
    measured, stamp = _stable_joint_feedback(robot, 4.0)
    base, shift = _commandable_pose(measured)
    _assert_no_faults(robot, "after enabling")
    return measured, base, shift, stamp


def _tolerance_for(delta):
    return min(0.01, max(0.001, abs(float(delta)) * 0.25))


# Raised from 0.02 rad (1.15 deg, barely visible) to 0.2 rad (11.5 deg) so a
# supervised operator can see the arm move. This is a bounded smoke test in
# clear space at _SPEED_PERCENT, not a licence for planned trajectories: there
# is still no collision checking here.
_MAX_DELTA_RAD = 0.2


def _check_delta(delta):
    if type(delta) not in (int, float) or not math.isfinite(delta) or not 0 < abs(delta) <= _MAX_DELTA_RAD:
        raise ValueError(f"delta must be finite, nonzero, and at most {_MAX_DELTA_RAD} rad")


def run_single_joint_test(config, joint=1, delta=0.02, timeout=8.0):
    if config.mode != "hardware_readonly":
        raise ValueError("Motion test requires hardware_readonly configuration")
    if type(joint) is not int or not 1 <= joint <= 6:
        raise ValueError("joint must be an integer from 1 to 6")
    _check_delta(delta)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 2 <= timeout <= 30:
        raise ValueError("timeout must be between 2 and 30 seconds")

    robot = _connect(config)
    try:
        robot.connect()
        measured, base, shift, start_stamp = _enable_and_baseline(robot)
        target = base.copy()
        target[joint - 1] += float(delta)
        if any(abs(value) > 3.1 for value in target):
            raise ValueError("Refusing target too close to generic +/-3.1 rad safety envelope")
        _commandable_pose(target)

        robot.set_motion_mode("j")
        tolerance = _tolerance_for(delta)

        robot.move_j(target)
        outward = _wait_target(robot, target, timeout, tolerance=tolerance)
        observed_delta = abs(outward["joints_rad"][joint - 1] - base[joint - 1])
        if observed_delta < abs(float(delta)) * 0.75:
            raise RuntimeError(
                f"Commanded joint did not visibly move: requested={float(delta):.6f} "
                f"observed={observed_delta:.6f} rad"
            )
        robot.move_j(base)
        returned = _wait_target(robot, base, timeout, tolerance=tolerance)
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
    if config.mode != "hardware_readonly":
        raise ValueError("Lateral test requires hardware_readonly configuration")
    _check_delta(delta)

    robot = _connect(config)
    try:
        robot.connect()
        measured, base, shift, _ = _enable_and_baseline(robot)
        left, right = base.copy(), base.copy()
        left[0] -= float(delta)
        right[0] += float(delta)
        for pose in (left, right):
            _commandable_pose(pose)

        robot.set_motion_mode("j")
        tolerance = _tolerance_for(delta)
        legs = {}
        for name, pose in (("left", left), ("right", right), ("return", base)):
            robot.move_j(pose)
            legs[name] = _wait_target(robot, pose, timeout, tolerance=tolerance)
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
