"""Explicit, supervised PiPER-X motion smoke test.

This module is intentionally separate from the MCP server. It only performs a
single-joint offset and return after an explicit CLI confirmation flag.
"""

import math
import time


def _joint_feedback(robot, timeout):
    deadline = time.monotonic() + timeout
    previous = None
    while time.monotonic() < deadline:
        msg = robot.get_joint_angles()
        if msg is not None:
            stamp = msg.timestamp
            if previous is None or stamp > previous:
                return [float(v) for v in msg.msg], stamp
            previous = stamp
        time.sleep(0.01)
    raise TimeoutError("No advancing joint feedback")


def _wait_target(robot, target, timeout):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        joints, stamp = _joint_feedback(robot, min(0.5, max(0.05, deadline - time.monotonic())))
        last = {"joints_rad": joints, "timestamp": stamp,
                "max_error_rad": max(abs(a - b) for a, b in zip(joints, target))}
        status = robot.get_arm_status()
        motion_done = status is not None and getattr(status.msg, "motion_status", None) == 0
        if motion_done and last["max_error_rad"] <= 0.01:
            return last
    raise TimeoutError(f"Target did not converge within {timeout:.1f}s; last={last}")


def run_single_joint_test(config, joint=1, delta=0.02, timeout=8.0):
    if config.mode != "hardware_readonly":
        raise ValueError("Motion test requires hardware_readonly configuration")
    if type(joint) is not int or not 1 <= joint <= 6:
        raise ValueError("joint must be an integer from 1 to 6")
    if type(delta) not in (int, float) or not math.isfinite(delta) or not 0 < abs(delta) <= 0.02:
        raise ValueError("delta must be finite, nonzero, and at most 0.02 rad")
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 2 <= timeout <= 30:
        raise ValueError("timeout must be between 2 and 30 seconds")

    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    sdk_config = create_agx_arm_config(robot=ArmModel.PIPER_X,
                                       firmeware_version=config.firmware,
                                       interface="socketcan", channel=config.channel)
    robot = AgxArmFactory.create_arm(sdk_config)
    target = None
    start = None
    try:
        robot.connect()
        start, start_stamp = _joint_feedback(robot, 3.0)
        target = start.copy()
        target[joint - 1] += float(delta)
        if any(abs(value) > 3.1 for value in target):
            raise ValueError("Refusing target too close to generic +/-3.1 rad safety envelope")

        # Explicitly request a low speed and joint-space mode. No gripper or
        # firmware/configuration operations are involved.
        robot.set_speed_percent(10)
        if not robot.enable():
            # Some firmware returns False when the joints were already enabled.
            # Accept only an explicit all-six-enabled readback.
            statuses = robot.get_joints_enable_status_list()
            if statuses != [True] * 6:
                raise RuntimeError(f"Arm did not report all joints enabled: {statuses!r}")
        robot.set_motion_mode("j")

        robot.move_j(target)
        outward = _wait_target(robot, target, timeout)
        robot.move_j(start)
        returned = _wait_target(robot, start, timeout)
        return {"joint": joint, "delta_rad": float(delta), "speed_percent": 10,
                "start_joints_rad": start, "target_joints_rad": target,
                "returned_joints_rad": returned["joints_rad"],
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
