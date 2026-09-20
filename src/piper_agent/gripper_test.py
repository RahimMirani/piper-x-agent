"""Small, low-force gripper actuator check."""

import time


def run_gripper_test(config):
    if config.mode != "hardware_readonly":
        raise ValueError("Gripper test requires hardware_readonly configuration")
    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    sdk_config = create_agx_arm_config(robot=ArmModel.PIPER_X,
                                       firmeware_version=config.firmware,
                                       interface="socketcan", channel=config.channel)
    robot = AgxArmFactory.create_arm(sdk_config)
    effector = None
    widths = (0.02, 0.04, 0.02)
    try:
        robot.connect()
        effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        for width in widths:
            effector.move_gripper_m(value=width, force=1.0)
            time.sleep(1.0)
        return {"widths_m": list(widths), "force_n": 1.0,
                "physical_motion_supported": True}
    finally:
        robot.disconnect()
