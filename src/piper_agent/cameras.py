from dataclasses import dataclass
import struct
import threading
import time
import zlib


@dataclass(frozen=True)
class Frame:
    role: str
    serial: str
    data: bytes
    mime: str
    received_monotonic: float
    received_unix_s: float
    device_timestamp: int | None = None

    def metadata(self):
        return {"role": self.role, "serial": self.serial, "mime": self.mime,
                "received_unix_s": self.received_unix_s,
                "age_s": time.monotonic() - self.received_monotonic,
                "device_timestamp": self.device_timestamp}


def mock_png(role):
    """A diagnostic pattern, not a rendering of the arm or its environment."""
    width, height = 320, 240
    color = (35, 100, 190) if role == "scene" else (190, 90, 35)
    rows = bytearray()
    for y in range(height):
        rows.append(0)
        for x in range(width):
            rows.extend(color if ((x // 20 + y // 20) % 2) else (25, 25, 25))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"tEXt", b"Description\0MOCK diagnostic pattern - not a robot camera")
            + chunk(b"IDAT", zlib.compress(bytes(rows))) + chunk(b"IEND", b""))


class MockCameras:
    def __init__(self, config):
        self.serials = {"scene": config.scene_serial, "wrist": config.wrist_serial}
        self.images = {role: mock_png(role) for role in self.serials}

    def capture(self):
        return [Frame(role, serial, self.images[role], "image/png", time.monotonic(), time.time())
                for role, serial in self.serials.items()]

    def latest(self, role):
        return self.capture()[0 if role == "scene" else 1]

    def close(self):
        pass


def enumerate_orbbec():
    import pyorbbecsdk as ob
    context = ob.Context()
    devices = context.query_devices()
    result = []
    for i in range(devices.get_count()):
        info = devices.get_device_by_index(i).get_device_info()
        result.append({"name": info.get_name(), "serial": info.get_serial_number(),
                       "firmware": info.get_firmware_version()})
    return result


class OrbbecCameras:
    """Serial-selected DC1 RGB streams. Requires the legacy pyorbbecsdk v1 build.

    Workers consume continuously; only genuinely new device timestamps update
    frame freshness. Depth processing and camera calibration are not implemented.
    """

    def __init__(self, config):
        import pyorbbecsdk as ob
        import cv2
        import numpy as np
        self.ob, self.cv2, self.np = ob, cv2, np
        self.config = config
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frames = {}
        self.errors = {}
        self.workers = []
        self.pipelines = []
        self.context = ob.Context()
        devices = self.context.query_devices()
        by_serial = {}
        for i in range(devices.get_count()):
            device = devices.get_device_by_index(i)
            by_serial[device.get_device_info().get_serial_number()] = device
        mapping = {"scene": config.scene_serial, "wrist": config.wrist_serial}
        missing = set(mapping.values()) - by_serial.keys()
        if missing:
            raise RuntimeError(f"Missing configured camera serials {sorted(missing)}; found {sorted(by_serial)}")
        try:
            for role, serial in mapping.items():
                pipeline = ob.Pipeline(by_serial[serial])
                settings = ob.Config()
                profile = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
                if profile.get_format() != ob.OBFormat.MJPG:
                    raise RuntimeError("Expected DC1 MJPG color profile; verify camera and legacy SDK")
                settings.enable_stream(profile)
                pipeline.start(settings)
                self.pipelines.append(pipeline)
                worker = threading.Thread(target=self._run, args=(role, serial, pipeline), daemon=True)
                self.workers.append(worker)
                worker.start()
        except BaseException:
            self.close()
            raise

    def _run(self, role, serial, pipeline):
        last_stamp = None
        try:
            while not self.stop_event.is_set():
                frames = pipeline.wait_for_frames(200)
                color = frames.get_color_frame() if frames is not None else None
                if color is None:
                    continue
                stamp = color.get_timestamp()
                if last_stamp is not None and stamp <= last_stamp:
                    continue
                received, wall = time.monotonic(), time.time()
                decoded = self.cv2.imdecode(self.np.frombuffer(color.get_data(), dtype=self.np.uint8), self.cv2.IMREAD_COLOR)
                if decoded is None:
                    raise RuntimeError("Invalid MJPG camera frame")
                h, w = decoded.shape[:2]
                scale = min(1.0, 960 / max(h, w))
                if scale < 1:
                    decoded = self.cv2.resize(decoded, (round(w * scale), round(h * scale)))
                ok, jpeg = self.cv2.imencode(".jpg", decoded, [self.cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                last_stamp = stamp
                with self.condition:
                    self.frames[role] = Frame(role, serial, jpeg.tobytes(), "image/jpeg", received, wall, stamp)
                    self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.errors[role] = str(exc)
                self.condition.notify_all()

    def capture(self):
        requested = time.monotonic()
        deadline = requested + self.config.camera_timeout_s
        with self.condition:
            while True:
                if self.errors:
                    raise RuntimeError(f"Camera capture failed: {self.errors}")
                frames = [self.frames.get(role) for role in ("scene", "wrist")]
                now = time.monotonic()
                if all(f is not None and f.received_monotonic >= requested
                       and now - f.received_monotonic <= self.config.camera_max_age_s for f in frames):
                    return frames
                if now >= deadline:
                    raise RuntimeError("Timed out waiting for fresh frames from BOTH cameras")
                self.condition.wait(min(0.1, deadline - now))

    def latest(self, role):
        with self.condition:
            frame = self.frames.get(role)
        if frame is None or time.monotonic() - frame.received_monotonic > self.config.camera_max_age_s:
            raise RuntimeError(f"No fresh {role} camera frame")
        return frame

    def close(self):
        self.stop_event.set()
        for worker in self.workers:
            worker.join(timeout=1)
        for pipeline in self.pipelines:
            try:
                pipeline.stop()
            except Exception:
                pass
