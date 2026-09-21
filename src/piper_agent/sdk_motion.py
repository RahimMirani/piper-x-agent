"""Shared PiPER-X SDK helpers for every path that commands the arm.

The supervised smoke tests and the live evaluation adapter must agree on what
counts as fresh feedback, a commandable pose and a converged move. Keeping one
copy here is the point: a freshness bug fixed in one place stays fixed.
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
CLAMP_TOLERANCE_RAD = 0.05

# move_l rejects orientations outside these; mirror the check so a refusal
# explains itself instead of surfacing as an opaque SDK ValueError.
_POSE_ANGLE_LIMITS = ((-math.pi, math.pi), (-math.pi / 2, math.pi / 2), (-math.pi, math.pi))

# How far apart two frames must be in time before their difference says
# anything about whether the flange has stopped.
_SETTLE_INTERVAL_S = 0.25

# How long to allow before deciding the controller simply refused a Cartesian
# target. move_l reports nothing when there is no IK solution: it neither moves
# nor errors, so without this the caller waits out the whole timeout and then
# fires a damped stop over a command the arm never accepted.
_NO_MOTION_GRACE_S = 2.5

_GRIPPER_FAULT_FLAGS = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                        "driver_overheating", "sensor_status", "driver_error_status")


def angle_difference(a, b):
    """Smallest signed difference between two angles, across the +/-pi seam.

    Orientation angles wrap: a roll of -3.140 and one of +3.14128 are the same
    orientation about a thousandth of a radian apart, but plain subtraction
    calls them 6.28 rad apart and no tolerance will ever accept them. This is
    for ORIENTATIONS only. Joint angles must keep using plain subtraction: a
    joint at -3.14 really is a full revolution away from one at +3.14, and
    wrapping there would call an unreached target converged.
    """
    return math.atan2(math.sin(a - b), math.cos(a - b))


def joint_limits():
    return _PIPER_X_LIMITS


def validate_six(values, name):
    if len(values) != 6 or any(type(v) not in (int, float) or isinstance(v, bool)
                               or not math.isfinite(v) for v in values):
        raise ValueError(f"{name} must be six finite numbers")
    return [float(v) for v in values]


def clamp_pose(joints):
    """Return the pose move_j will actually command and the per-joint shift."""
    clamped = [min(max(value, lower), upper)
               for value, (lower, upper) in zip(joints, _PIPER_X_LIMITS)]
    return clamped, [after - before for after, before in zip(clamped, joints)]


def commandable_pose(joints):
    """Validate a feedback pose and return the pose the SDK will command."""
    clamped, shift = clamp_pose(joints)
    for index, (value, offset, (lower, upper)) in enumerate(zip(joints, shift, _PIPER_X_LIMITS), 1):
        if abs(offset) > CLAMP_TOLERANCE_RAD:
            raise RuntimeError(
                f"Refusing motion: joint {index} feedback {value:.6f} rad is "
                f"{abs(offset):.6f} rad outside PiPER-X SDK range "
                f"[{lower:.6f}, {upper:.6f}], and move_j would silently clamp it. "
                "Verify the arm model/firmware zeroing before commanding motion."
            )
    return clamped, shift


def require_in_joint_limits(joints):
    """Refuse an out-of-range commanded target instead of letting the SDK clamp it."""
    for index, (value, (lower, upper)) in enumerate(zip(joints, _PIPER_X_LIMITS), 1):
        if not lower <= value <= upper:
            raise ValueError(
                f"Joint {index} target {value:.6f} rad is outside the PiPER-X range "
                f"[{lower:.6f}, {upper:.6f}]. The command was refused, not clamped; "
                "send a target inside the range."
            )


def require_valid_pose_angles(pose):
    names = ("roll", "pitch", "yaw")
    for name, value, (lower, upper) in zip(names, pose[3:], _POSE_ANGLE_LIMITS):
        if not lower <= value <= upper:
            raise ValueError(
                f"Pose {name} {value:.6f} rad is outside [{lower:.6f}, {upper:.6f}]"
            )


# arm_status values that mean the controller will not act on commands, or has
# hit something. Checking only the err_status bits missed all of these — a
# collision included. 0x00 is NORMAL and 0x08 is a teaching-drag overspeed that
# cannot arise here.
_BLOCKING_ARM_STATUS = {
    0x01: "EMERGENCY_STOP",
    0x02: "NO_SOLUTION",
    0x03: "SINGULARITY_POINT",
    0x04: "TARGET_POS_EXCEEDS_LIMIT",
    0x05: "JOINT_COMMUNICATION_ERR",
    0x06: "JOINT_BRAKE_NOT_RELEASED",
    0x07: "COLLISION_OCCURRED",
}


def arm_faults(robot):
    """Return the arm's active fault names, or an empty list."""
    status = robot.get_arm_status()
    if status is None:
        return []
    faults = [name for name, value in vars(status.msg.err_status).items() if value is True]
    state = getattr(status.msg, "arm_status", None)
    if state is not None:
        try:
            name = _BLOCKING_ARM_STATUS.get(int(state))
        except (TypeError, ValueError):
            name = None
        if name:
            faults.append(f"arm_status={name}")
    if int(getattr(status.msg, "err_code", 0) or 0):
        faults.append(f"err_code={int(status.msg.err_code)}")
    return faults


def assert_no_faults(robot, stage):
    faults = arm_faults(robot)
    if faults:
        raise RuntimeError(f"Arm reported faults {faults} {stage}")


def joint_feedback(robot, timeout):
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


def flange_pose_feedback(robot, timeout):
    """Return a Cartesian flange pose published during this call.

    get_flange_pose assembles x/y, z/rx and ry/rz from three CAN messages into
    one mutable buffer that the SDK reuses, so copy the values out immediately
    and require the timestamp to advance before trusting them.
    """
    deadline = time.monotonic() + timeout
    initial = None
    while time.monotonic() < deadline:
        msg = robot.get_flange_pose()
        if msg is not None:
            stamp = msg.timestamp
            values = [float(v) for v in msg.msg]
            if initial is None:
                initial = stamp
            elif stamp > initial:
                return values, stamp
        time.sleep(0.01)
    raise TimeoutError("No advancing flange pose feedback")


def stable_joint_feedback(robot, timeout):
    """Return a baseline only after three coherent advancing frames."""
    deadline = time.monotonic() + timeout
    previous = None
    stable = 0
    while time.monotonic() < deadline:
        joints, stamp = joint_feedback(robot, min(0.5, max(0.05, deadline - time.monotonic())))
        if previous is not None and max(abs(a - b) for a, b in zip(joints, previous)) <= 0.002:
            stable += 1
            if stable >= 3:
                return joints, stamp
        else:
            stable = 0
        previous = joints
    raise TimeoutError("No coherent joint baseline within timeout")


def wait_target(robot, target, timeout, tolerance=0.01):
    deadline = time.monotonic() + timeout
    last = None
    previous = None
    while time.monotonic() < deadline:
        joints, stamp = joint_feedback(robot, min(0.5, max(0.05, deadline - time.monotonic())))
        # During an active trajectory this SDK can briefly publish a partially
        # assembled frame with one or more joints at exactly zero. Reject
        # implausible discontinuities instead of treating that frame as real.
        if previous is not None and max(abs(a - b) for a, b in zip(joints, previous)) > 0.25:
            continue
        previous = joints
        last = {"joints_rad": joints, "timestamp": stamp,
                "max_error_rad": max(abs(a - b) for a, b in zip(joints, target))}
        assert_no_faults(robot, "while converging on a commanded pose")
        status = robot.get_arm_status()
        # motion_status only separates reach-success from reach-failure; it is
        # not a busy flag, so convergence rests on the measured error above.
        motion_done = status is not None and getattr(status.msg, "motion_status", None) == 0
        if motion_done and last["max_error_rad"] <= tolerance:
            return last
    raise TimeoutError(f"Target did not converge within {timeout:.1f}s; last={last}")


def wait_pose_target(robot, target, timeout, position_tolerance=0.005, angle_tolerance=0.05,
                     start_joints=None, max_joint_excursion_rad=None, start_pose=None):
    """Wait for the flange pose to settle at target.

    motion_status is not usable as a gate here, unlike the joint path. move_l
    latches REACH_TARGET_POS_FAILED when the controller cannot follow the
    straight-line path exactly, and the flag stays set after the flange has in
    fact arrived, so gating on it fails a move that physically succeeded and then
    fires a damped stop. Converge on a measured error that has stopped changing,
    and report what the controller believed rather than obeying it.

    A Cartesian step does not bound joint motion: near the base axis a two
    centimetre lateral move can demand twenty degrees of base rotation. When the
    caller supplies a joint budget, abort the move the moment it is exceeded.
    """
    deadline = time.monotonic() + timeout
    started = time.monotonic()
    last = None
    previous = None
    previous_time = 0.0
    settled = 0
    while time.monotonic() < deadline:
        pose, stamp = flange_pose_feedback(robot, min(0.5, max(0.05, deadline - time.monotonic())))
        assert_no_faults(robot, "while converging on a commanded Cartesian pose")
        excursion = None
        if start_joints is not None and max_joint_excursion_rad is not None:
            joints, _ = joint_feedback(robot, 0.5)
            excursion = max(abs(a - b) for a, b in zip(joints, start_joints))
            if excursion > max_joint_excursion_rad:
                raise RuntimeError(
                    f"Aborting Cartesian move: a joint has swung {excursion:.4f} rad, past the "
                    f"{max_joint_excursion_rad:.4f} rad budget for one call. A small Cartesian "
                    "step near the base axis can demand a large joint motion."
                )
        position_error = max(abs(a - b) for a, b in zip(pose[:3], target[:3]))
        angle_error = max(abs(angle_difference(a, b)) for a, b in zip(pose[3:], target[3:]))
        # A target with no IK solution from the current configuration is simply
        # ignored: the flange does not move and nothing is reported. Say so
        # promptly and actionably instead of waiting out the timeout.
        if start_pose is not None and time.monotonic() - started > _NO_MOTION_GRACE_S:
            moved = max(abs(a - b) for a, b in zip(pose[:3], start_pose[:3]))
            if moved < 0.001 and position_error > position_tolerance:
                raise RuntimeError(
                    "The controller did not accept this Cartesian pose: the flange has not moved "
                    f"after {_NO_MOTION_GRACE_S:.1f}s and is still {position_error:.4f} m away. "
                    "It is most likely unreachable from the current arm configuration. Move in "
                    "joint space to open the arm out first, then try Cartesian again."
                )
        status = robot.get_arm_status()
        last = {"flange_pose": pose, "timestamp": stamp,
                "max_position_error_m": position_error, "max_angle_error_rad": angle_error,
                "max_joint_excursion_rad": excursion,
                "controller_motion_status": str(getattr(status.msg, "motion_status", None))
                                            if status is not None else None}
        # Compare against a frame from a fixed interval ago, never the frame
        # immediately before. At 10% speed with ~100 Hz feedback a moving flange
        # advances well under a millimetre per frame, so consecutive frames make
        # a move in progress look stationary and the wait returns early.
        now = time.monotonic()
        if previous is None or now - previous_time >= _SETTLE_INTERVAL_S:
            if previous is not None and \
                    max(abs(a - b) for a, b in zip(pose[:3], previous[:3])) <= 0.0005 and \
                    max(abs(angle_difference(a, b)) for a, b in zip(pose[3:], previous[3:])) <= 0.005:
                settled += 1
            else:
                settled = 0
            previous = pose
            previous_time = now
        if settled >= 2 and position_error <= position_tolerance and angle_error <= angle_tolerance:
            return last
    raise TimeoutError(f"Cartesian target did not settle within {timeout:.1f}s; last={last}")


def connect_arm(config):
    """Create an SDK arm handle. The caller still has to call connect()."""
    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    sdk_config = create_agx_arm_config(robot=ArmModel.PIPER_X,
                                       firmeware_version=config.firmware,
                                       interface="socketcan", channel=config.channel)
    return AgxArmFactory.create_arm(sdk_config)


def enable_and_baseline(robot, speed_percent):
    """Enable at low speed, wait for a settled pose, return the commandable pose."""
    assert_no_faults(robot, "before enabling")
    # Explicitly request a low speed. No gripper or firmware/configuration
    # operations are involved.
    robot.set_speed_percent(speed_percent)
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
    measured, stamp = stable_joint_feedback(robot, 4.0)
    base, shift = commandable_pose(measured)
    assert_no_faults(robot, "after enabling")
    return measured, base, shift, stamp


def gripper_feedback(effector, timeout=2.0):
    """Return a gripper frame published during this call, not a cached one."""
    deadline = time.monotonic() + timeout
    initial = None
    while time.monotonic() < deadline:
        status = effector.get_gripper_status()
        if status is not None:
            stamp = status.timestamp
            if initial is None:
                initial = stamp
            elif stamp > initial:
                return status
        time.sleep(0.01)
    raise TimeoutError("No advancing gripper feedback; check the effector CAN connection")


def gripper_observation(effector, timeout=2.0):
    """Non-raising gripper read for status reporting.

    A read should say what the gripper's condition is, including "the driver is
    off", rather than collapsing every problem into a missing value the caller
    cannot tell apart from a wiring fault.
    """
    try:
        status = gripper_feedback(effector, timeout)
    except TimeoutError as exc:
        return {"available": False, "reason": str(exc)}
    foc = status.msg.foc_status
    return {"available": True,
            "measured_width_m": float(status.msg.value),
            "measured_force_n": float(status.msg.force),
            "driver_enabled": bool(foc.driver_enable_status),
            "homed": bool(foc.homing_status),
            "faults": [name for name in _GRIPPER_FAULT_FLAGS if getattr(foc, name)],
            "timestamp": status.timestamp}


def gripper_sample(effector, stage):
    status = gripper_feedback(effector)
    foc = status.msg.foc_status
    faults = [name for name in _GRIPPER_FAULT_FLAGS if getattr(foc, name)]
    if faults:
        raise RuntimeError(f"Gripper reported faults {faults} {stage}")
    if not foc.driver_enable_status:
        raise RuntimeError(f"Gripper driver is not enabled {stage}")
    return {"measured_width_m": float(status.msg.value),
            "measured_force_n": float(status.msg.force),
            "timestamp": status.timestamp,
            "homed": bool(foc.homing_status)}
