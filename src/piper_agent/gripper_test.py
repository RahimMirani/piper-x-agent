"""Small, low-force gripper actuator check with measured feedback."""

import time

_FORCE_N = 1.0
_WIDTHS_M = (0.02, 0.04, 0.02)
_SETTLE_S = 1.5
# The open step is 0.02 m wider than the two closed steps. Require most of that
# to show up in the feedback; commanding the gripper is not evidence it moved.
_MIN_MEASURED_TRAVEL_M = 0.010

_FAULT_FLAGS = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                "driver_overheating", "sensor_status", "driver_error_status")


def _gripper_feedback(effector, timeout=2.0):
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


def _sample(effector, stage):
    status = _gripper_feedback(effector)
    foc = status.msg.foc_status
    faults = [name for name in _FAULT_FLAGS if getattr(foc, name)]
    if faults:
        raise RuntimeError(f"Gripper reported faults {faults} {stage}")
    if not foc.driver_enable_status:
        raise RuntimeError(f"Gripper driver is not enabled {stage}")
    return {"measured_width_m": float(status.msg.value),
            "measured_force_n": float(status.msg.force),
            "timestamp": status.timestamp,
            "homed": bool(foc.homing_status)}


def run_gripper_test(config):
    if config.mode != "hardware_readonly":
        raise ValueError("Gripper test requires hardware_readonly configuration")
    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    sdk_config = create_agx_arm_config(robot=ArmModel.PIPER_X,
                                       firmeware_version=config.firmware,
                                       interface="socketcan", channel=config.channel)
    robot = AgxArmFactory.create_arm(sdk_config)
    try:
        robot.connect()
        # init_effector only registers a driver; it sends no CAN command. The
        # gripper is never zeroed or calibrated here, so absolute width is only
        # trustworthy when the controller already reports it as homed.
        effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        baseline = _sample(effector, "before commanding the gripper")
        steps = []
        for width in _WIDTHS_M:
            effector.move_gripper_m(value=width, force=_FORCE_N)
            time.sleep(_SETTLE_S)
            steps.append({"commanded_width_m": width,
                          **_sample(effector, f"after commanding {width:.3f} m")})
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
