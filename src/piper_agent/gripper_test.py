"""Small, low-force gripper actuator check with measured feedback."""

import time

from .sdk_motion import connect_arm, gripper_sample

_FORCE_N = 1.0
_WIDTHS_M = (0.02, 0.04, 0.02)
_SETTLE_S = 1.5
# The open step is 0.02 m wider than the two closed steps. Require most of that
# to show up in the feedback; commanding the gripper is not evidence it moved.
_MIN_MEASURED_TRAVEL_M = 0.010

_HARDWARE_MODES = {"hardware_readonly", "hardware_live"}


def run_gripper_test(config):
    if config.mode not in _HARDWARE_MODES:
        raise ValueError("Gripper test requires a hardware configuration")
    robot = connect_arm(config)
    try:
        robot.connect()
        # init_effector only registers a driver; it sends no CAN command. The
        # gripper is never zeroed or calibrated here, so absolute width is only
        # trustworthy when the controller already reports it as homed.
        effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        baseline = gripper_sample(effector, "before commanding the gripper")
        steps = []
        for width in _WIDTHS_M:
            effector.move_gripper_m(value=width, force=_FORCE_N)
            time.sleep(_SETTLE_S)
            steps.append({"commanded_width_m": width,
                          **gripper_sample(effector, f"after commanding {width:.3f} m")})
        travel = max(s["measured_width_m"] for s in steps) - min(s["measured_width_m"] for s in steps)
        if travel < _MIN_MEASURED_TRAVEL_M:
            raise RuntimeError(
                f"Gripper did not visibly travel: commanded "
                f"{max(_WIDTHS_M) - min(_WIDTHS_M):.3f} m, measured {travel:.4f} m"
            )
        return {"commanded_widths_m": list(_WIDTHS_M), "force_n": _FORCE_N,
                "baseline": baseline, "steps": steps,
                "measured_travel_m": travel,
                "homed": baseline["homed"],
                "absolute_width_verified": baseline["homed"],
                "physical_motion_supported": True}
    finally:
        robot.disconnect()
