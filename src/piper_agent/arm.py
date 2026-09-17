import math
import threading
import time


def validate_joints(joints):
    if len(joints) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in joints):
        raise ValueError("Expected six finite joint angles in radians")
    return [float(v) for v in joints]


class MockArm:
    """Deterministic state stub, NOT a physics or collision simulator."""

    def __init__(self):
        self.joints = [0.0] * 6
        self.aperture = 0.04
        self.stopped = False
        self.lock = threading.RLock()

    def state(self):
        with self.lock:
            return {"source": "mock", "joints_rad": self.joints.copy(),
                    "gripper_aperture_m": self.aperture, "stopped": self.stopped,
                    "physical_motion_supported": False, "sampled_at_unix_s": time.time()}

    def move(self, joints):
        with self.lock:
            target = validate_joints(joints)
            if self.stopped:
                raise RuntimeError("Mock stop is latched; start a new session to reset")
            if any(abs(a - b) > 0.1 for a, b in zip(target, self.joints)):
                raise ValueError("Mock step exceeds 0.1 rad per joint")
            if any(abs(a) > math.pi for a in target):
                raise ValueError("Mock range is +/- pi; these are NOT Piper X joint limits")
            self.joints = target
            return self.state()

    def grip(self, aperture):
        with self.lock:
            if self.stopped:
                raise RuntimeError("Mock stop is latched")
            if type(aperture) not in (int, float) or not math.isfinite(aperture) or not 0 <= aperture <= 0.07:
                raise ValueError("Mock aperture must be 0..0.07 metres")
            self.aperture = aperture
            return self.state()

    def stop(self):
        with self.lock:
            self.stopped = True
            return self.state()

    def close(self):
        pass


class ReadOnlyArm:
    """SDK telemetry only. Never enables, homes, clears faults or sends motion."""

    def __init__(self, config):
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

        cfg = create_agx_arm_config(robot=ArmModel.PIPER_X,
                                   firmeware_version=config.firmware,
                                   interface="socketcan", channel=config.channel)
        self.robot = AgxArmFactory.create_arm(cfg)
        try:
            self.robot.connect()
        except BaseException:
            self.robot.disconnect()
            raise

    def state(self):
        # SDK caches feedback. Require the timestamp to ADVANCE during this read,
        # rather than assuming a non-None cached value is fresh. No clock-unit guess.
        deadline = time.monotonic() + 2.0
        initial = None
        while time.monotonic() < deadline:
            msg = self.robot.get_joint_angles()
            if msg is not None:
                stamp = msg.timestamp
                if initial is not None and stamp > initial:
                    return {"source": "hardware_readonly", "joints_rad": validate_joints(list(msg.msg)),
                            "sdk_timestamp": stamp, "sampled_at_unix_s": time.time(),
                            "physical_motion_supported": False,
                            "gripper_aperture_m": None}
                initial = stamp if initial is None else initial
            time.sleep(0.01)
        raise RuntimeError("No advancing joint feedback within 2 seconds; check CAN, power and SDK profile")

    def move(self, joints):
        raise RuntimeError("Physical motion is unavailable in v0.1; complete Pi commissioning first")

    def grip(self, aperture):
        raise RuntimeError("Physical gripper control is unavailable in v0.1")

    def stop(self):
        raise RuntimeError("This telemetry adapter cannot stop hardware. Use the physical stop procedure")

    def close(self):
        # Do not disable: an unsupported arm can drop under gravity.
        self.robot.disconnect()
