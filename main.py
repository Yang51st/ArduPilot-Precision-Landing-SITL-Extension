#!/usr/bin/env python3
"""ArUco precision landing bridge for Simulink or a Raspberry Pi camera."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil


FRAME_HEADER = struct.Struct("<4sIIIII")
# Camera coordinates are right/down/forward. The target frame is forward
# (toward the marker's top edge), right, down, matching MAV_FRAME_BODY_FRD.
CAMERA_TO_BODY = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
ARUCO_FROM_TARGET = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=np.float64)
REQUIRED_PARAMETERS = {
    "PLND_ENABLED": 1.0,
    "PLND_TYPE": 1.0,
    "PLND_EST_TYPE": 0.0,
    "PLND_ALT_MIN": 0.0,
    "PLND_ALT_MAX": 0.0,
    "SERIAL2_PROTOCOL": 2.0,
}


class Disconnected(RuntimeError):
    pass


class CameraDisconnected(Disconnected):
    pass


class CameraUnavailable(CameraDisconnected):
    pass


@dataclass(frozen=True)
class Detection:
    marker_id: int
    size_m: float
    camera_xyz: np.ndarray
    body_rpy: np.ndarray
    body_quaternion: tuple[float, float, float, float]

    @property
    def body_frd(self) -> np.ndarray:
        x, y, z = self.camera_xyz
        return np.array([-y, x, z], dtype=np.float32)

    @property
    def distance(self) -> float:
        return float(np.linalg.norm(self.camera_xyz))


def orientation_from_rotation(rotation: np.ndarray) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """Return body-relative target roll/pitch/yaw and quaternion (w, x, y, z)."""
    rotation_vector = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-9:
        quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        xyz = rotation_vector / angle * math.sin(angle / 2)
        quaternion = np.array([math.cos(angle / 2), *xyz])
    if quaternion[0] < 0:
        quaternion = -quaternion
    w, x, y, z = quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2 * (w * y - z * x), -1, 1)))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=np.float32), tuple(map(float, quaternion))


class ArucoDetector:
    def __init__(self, config: dict, calibration: Path | None) -> None:
        family = config["marker_family"]
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, family):
            raise RuntimeError(f"OpenCV does not provide ArUco family {family}")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        self.sizes = {int(item["id"]): float(item["size_m"]) for item in config["markers"]}
        if not self.sizes or any(marker_id < 0 or size <= 0 for marker_id, size in self.sizes.items()):
            raise ValueError("markers need a non-negative id and positive size_m")
        if len(self.sizes) != len(config["markers"]):
            raise ValueError("marker ids must be unique")
        self.fov = float(config["camera_hfov_deg"])
        self.camera_matrix = self.distortion = self.calibration_size = None
        self._intrinsics: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}
        if calibration:
            data = json.loads(calibration.read_text(encoding="utf-8"))
            self.camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
            self.distortion = np.asarray(data.get("distortion", []), dtype=np.float64)
            if data.get("image_size"):
                self.calibration_size = tuple(map(int, data["image_size"]))
                calibration_width, _ = self.calibration_size
                calibrated_fov = math.degrees(
                    2 * math.atan(calibration_width / (2 * float(self.camera_matrix[0, 0])))
                )
                if not math.isclose(calibrated_fov, self.fov, rel_tol=0.20):
                    raise ValueError(
                        f"calibration horizontal FOV is {calibrated_fov:.1f} deg, but the configured "
                        f"camera/sensor mode is {self.fov:.1f} deg; recapture calibration images "
                        "using the same sensor mode"
                    )
        self.object_corners = np.array(
            [[-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]],
            dtype=np.float32,
        )

    def intrinsics(self, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        cached = self._intrinsics.get((width, height))
        if cached is not None:
            return cached
        if self.camera_matrix is not None:
            if self.calibration_size and self.calibration_size != (width, height):
                calibration_width, calibration_height = self.calibration_size
                scale_x = width / calibration_width
                scale_y = height / calibration_height
                if not math.isclose(scale_x, scale_y, rel_tol=0.01):
                    raise ValueError(
                        f"calibration is for {self.calibration_size}; cannot scale it to {(width, height)} "
                        "because the aspect ratio changed"
                    )
                matrix = self.camera_matrix.copy()
                matrix[0, :] *= scale_x
                matrix[1, :] *= scale_y
            else:
                matrix = self.camera_matrix
            distortion = self.distortion
        else:
            focal = width / 2 / math.tan(math.radians(self.fov) / 2)
            matrix = np.array(
                [[focal, 0, (width - 1) / 2], [0, focal, (height - 1) / 2], [0, 0, 1]],
                dtype=np.float64,
            )
            distortion = np.zeros(5)
        self._intrinsics[(width, height)] = matrix, distortion
        return matrix, distortion

    def detect(self, image: np.ndarray) -> Detection | None:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        detected_ids = [int(marker_id) for marker_id in ids.reshape(-1)]
        visible_ids = [marker_id for marker_id in detected_ids if marker_id in self.sizes]
        if len(visible_ids) != len(set(visible_ids)):
            return None
        candidates = []
        for marker_corners, marker_id in zip(corners, detected_ids):
            if marker_id not in self.sizes:
                continue
            points = marker_corners.reshape(4, 2).astype(np.float32)
            area = float(cv2.contourArea(points))
            candidates.append((self.sizes[marker_id], -area, marker_id, points))
        if not candidates:
            return None

        height, width = image.shape[:2]
        matrix, distortion = self.intrinsics(width, height)
        # Nested targets reveal progressively smaller physical markers during
        # descent. Prefer the smallest configured marker that is visible. For
        # equal physical sizes, prefer the larger image and then the lower ID.
        size, _negative_area, marker_id, points = min(candidates)
        ok, rotation_vector, translation = cv2.solvePnP(
            self.object_corners * size,
            points,
            matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        xyz = translation.reshape(3).astype(np.float32)
        if not np.all(np.isfinite(xyz)) or xyz[2] <= 0:
            return None
        aruco_to_camera = cv2.Rodrigues(rotation_vector)[0]
        target_to_body = CAMERA_TO_BODY @ aruco_to_camera @ ARUCO_FROM_TARGET
        rpy, quaternion = orientation_from_rotation(target_to_body)
        return Detection(marker_id, size, xyz, rpy, quaternion)


class SimulinkFrames:
    def __init__(self, endpoint: str) -> None:
        self.host, self.port = split_endpoint(endpoint)
        self.waiting_announced = False

    @staticmethod
    def receive(connection: socket.socket, size: int, stop: threading.Event) -> bytes:
        data = bytearray()
        while len(data) < size and not stop.is_set():
            try:
                chunk = connection.recv(size - len(data))
            except socket.timeout:
                continue
            if not chunk:
                raise CameraDisconnected("stream closed")
            data.extend(chunk)
        if len(data) != size:
            raise CameraDisconnected("stream stopped")
        return bytes(data)

    def frames(self, stop: threading.Event) -> Iterator[np.ndarray]:
        if not self.waiting_announced:
            print(f"Camera: waiting for Simulink at {self.host}:{self.port}", flush=True)
            self.waiting_announced = True
        try:
            connection = socket.create_connection((self.host, self.port), timeout=2)
        except OSError as error:
            raise CameraUnavailable from error
        with connection:
            connection.settimeout(1)
            print("Camera: connected to Simulink", flush=True)
            self.waiting_announced = False
            while not stop.is_set():
                header = self.receive(connection, FRAME_HEADER.size, stop)
                magic, width, height, channels, _, size = FRAME_HEADER.unpack(header)
                if (
                    magic != b"APSM"
                    or channels != 3
                    or size != width * height * 3
                    or not 0 < size <= 32 << 20
                ):
                    raise CameraDisconnected("invalid frame")
                yield np.frombuffer(self.receive(connection, size, stop), np.uint8).reshape(height, width, 3)


class PiFrames:
    def __init__(self, config: dict) -> None:
        self.config = config
        self.dropped_frames = 0

    def frames(self, stop: threading.Event) -> Iterator[np.ndarray]:
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise CameraDisconnected("Picamera2 is not installed") from error
        camera = None
        try:
            camera = Picamera2()
            width = int(self.config["camera_width"])
            height = int(self.config["camera_height"])
            pixel_format = self.config.get("camera_format", "YUV420")
            if pixel_format not in {"YUV420", "BGR888"}:
                raise ValueError("camera_format must be YUV420 or BGR888")
            main = {
                "size": (width, height),
                "format": pixel_format,
            }
            options = {}
            if "camera_sensor_width" in self.config:
                options["raw"] = {
                    "size": (int(self.config["camera_sensor_width"]), int(self.config["camera_sensor_height"]))
                }
            setup = camera.create_video_configuration(
                main=main, controls={"FrameRate": float(self.config["camera_fps"])}, **options
            )
            camera.configure(setup)
            camera.start()
        except Exception as error:
            if camera is not None:
                camera.close()
            raise CameraDisconnected(f"cannot start Pi camera: {error}") from error
        print(f"Camera: Picamera2 {width}x{height} {pixel_format}", flush=True)
        # Capture independently of detection. This overlaps sensor wait time with
        # processing and keeps only the newest image instead of building latency.
        latest: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        capture_stop = threading.Event()
        capture_errors: list[Exception] = []

        def capture() -> None:
            try:
                while not stop.is_set() and not capture_stop.is_set():
                    image = camera.capture_array("main")
                    if pixel_format == "YUV420":
                        # Picamera2 returns YUV420 as (height * 3/2, stride).
                        # ArUco needs only the full-resolution luma plane.
                        image = image[:height, :width]
                    while not capture_stop.is_set():
                        try:
                            latest.put_nowait(image)
                            break
                        except queue.Full:
                            try:
                                latest.get_nowait()
                            except queue.Empty:
                                continue
                            self.dropped_frames += 1
            except Exception as error:
                capture_errors.append(error)

        worker = threading.Thread(target=capture, name="camera-capture", daemon=True)
        worker.start()
        try:
            while not stop.is_set():
                try:
                    yield latest.get(timeout=0.2)
                except queue.Empty:
                    if capture_errors:
                        raise CameraDisconnected(f"camera capture failed: {capture_errors[0]}")
                    if not worker.is_alive():
                        raise CameraDisconnected("camera capture stopped")
        finally:
            capture_stop.set()
            try:
                camera.stop()
            except Exception:
                pass
            worker.join(timeout=2)
            camera.close()


class LandingTarget:
    def __init__(self, endpoint: str, baud: int | None, dry_run: bool, config: dict) -> None:
        self.endpoint, self.baud, self.dry_run = endpoint, baud, dry_run
        self.connection = None
        self.last_heartbeat = -math.inf
        self.vehicle_yaw = None
        self.flight_mode = None
        self.armed = False
        self.last_yaw_command = -math.inf
        self.yaw_offset = math.radians(float(config.get("landing_yaw_offset_deg", 0)))
        self.yaw_rate = float(config.get("yaw_command_rate_deg_s", 30))
        self.yaw_tolerance = math.radians(float(config.get("yaw_tolerance_deg", 2)))
        self.yaw_modes = set(config.get("yaw_align_modes", ["AUTO", "LAND", "RTL"]))

    def connect(self) -> None:
        if self.dry_run:
            return
        print(f"MAVLink: connecting to {self.endpoint}", flush=True)
        options = {"baud": self.baud} if self.baud else {}
        self.connection = mavutil.mavlink_connection(
            self.endpoint,
            source_system=42,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
            autoreconnect=True,
            **options,
        )
        heartbeat = self.connection.wait_heartbeat(timeout=5)
        if heartbeat is None:
            self.close()
            raise Disconnected("no ArduPilot heartbeat")
        self._update_heartbeat(heartbeat)
        self.check_parameters()
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            100000,
            0,
            0,
            0,
            0,
            0,
        )
        print("MAVLink: connected; precision-landing parameters verified", flush=True)

    def check_parameters(self) -> None:
        target_system = self.connection.target_system
        target_component = self.connection.target_component
        for name in REQUIRED_PARAMETERS:
            self.connection.mav.param_request_read_send(target_system, target_component, name.encode(), -1)
        values = {}
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and len(values) < len(REQUIRED_PARAMETERS):
            message = self.connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if message:
                name = str(message.param_id).rstrip("\0")
                if name in REQUIRED_PARAMETERS:
                    values[name] = float(message.param_value)
        wrong = [
            name
            for name, expected in REQUIRED_PARAMETERS.items()
            if name not in values or not math.isclose(values[name], expected, abs_tol=0.01)
        ]
        if wrong:
            raise RuntimeError("incorrect or missing ArduPilot parameters: " + ", ".join(wrong))

    def _update_heartbeat(self, message) -> None:
        self.flight_mode = mavutil.mode_string_v10(message)
        self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def drain(self) -> None:
        if self.connection is None:
            return
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            if message.get_srcSystem() != self.connection.target_system:
                continue
            if message.get_type() == "ATTITUDE":
                self.vehicle_yaw = float(message.yaw)
            elif message.get_type() == "HEARTBEAT":
                self._update_heartbeat(message)

    def align_yaw(self, detection: Detection) -> None:
        now = time.monotonic()
        yaw_error = math.remainder(float(detection.body_rpy[2]) + self.yaw_offset, 2 * math.pi)
        if (
            self.vehicle_yaw is None
            or not self.armed
            or self.flight_mode not in self.yaw_modes
            or abs(yaw_error) <= self.yaw_tolerance
            or now - self.last_yaw_command < 0.5
        ):
            return
        desired_yaw_deg = math.degrees((self.vehicle_yaw + yaw_error) % (2 * math.pi))
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            desired_yaw_deg,
            self.yaw_rate,
            0,
            0,
            0,
            0,
            0,
        )
        self.last_yaw_command = now

    def send(self, detection: Detection) -> None:
        """Send one current metric target; callers must not invoke this for a lost target."""
        if self.dry_run:
            return
        if self.connection is None:
            raise Disconnected("MAVLink connection lost")
        x, y, z = map(float, detection.camera_xyz)
        body_x, body_y, body_z = map(float, detection.body_frd)
        size = 2 * math.atan2(detection.size_m / 2, z)
        try:
            self.connection.mav.landing_target_send(
                time.monotonic_ns() // 1000,
                detection.marker_id,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                math.atan2(x, z),
                math.atan2(y, z),
                detection.distance,
                size,
                size,
                body_x,
                body_y,
                body_z,
                detection.body_quaternion,
                mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                # position_valid=0 is angular mode, not a target-lost flag.
                1,
            )
            self.align_yaw(detection)
        except OSError as error:
            self.close()
            raise Disconnected(str(error)) from error

    def send_heartbeat(self) -> None:
        """Keep the companion link alive independently of marker visibility."""
        if self.dry_run or self.connection is None:
            return
        now = time.monotonic()
        if now - self.last_heartbeat < 1:
            return
        try:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.last_heartbeat = now
        except OSError as error:
            self.close()
            raise Disconnected(str(error)) from error

    def close(self) -> None:
        if self.connection:
            self.connection.close()
            self.connection = None
        self.vehicle_yaw = None
        self.flight_mode = None
        self.armed = False
        self.last_heartbeat = -math.inf
        self.last_yaw_command = -math.inf


def split_endpoint(value: str) -> tuple[str, int]:
    host, separator, port = value.rpartition(":")
    if not separator or not host or not port.isdigit() or not 1 <= int(port) <= 65535:
        raise ValueError("frame endpoint must be HOST:PORT")
    return host, int(port)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("sim", "pi"), help="Simulink TCP stream or Raspberry Pi camera")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("landing_config.json"))
    parser.add_argument("--calibration", type=Path, help="camera calibration JSON")
    parser.add_argument("--mavlink", help="override the configured MAVLink endpoint")
    parser.add_argument("--dry-run", action="store_true", help="detect without sending MAVLink")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    root = json.loads(args.config.read_text(encoding="utf-8"))
    config = {"marker_family": root["marker_family"], **root[args.mode]}
    detector = ArucoDetector(config, args.calibration)
    frames = SimulinkFrames(config["frame_endpoint"]) if args.mode == "sim" else PiFrames(config)
    endpoint = args.mavlink or config["mavlink_endpoint"]
    baud = int(config["mavlink_baud"]) if config.get("mavlink_baud") else None
    target = LandingTarget(endpoint, baud, args.dry_run, config)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print("MAVLink: dry run" if args.dry_run else "Bridge: starting", flush=True)

    detections = frames_seen = 0
    detection_seconds = 0.0
    reported_drops = 0
    report_at = time.monotonic()
    detection = None
    visible_marker_id = None
    while not stop.is_set():
        try:
            if target.connection is None and not args.dry_run:
                target.connect()
            target.drain()
            for image in frames.frames(stop):
                frames_seen += 1
                target.drain()
                detection_started = time.monotonic()
                detection = detector.detect(image)
                detection_seconds += time.monotonic() - detection_started
                if detection:
                    detections += 1
                    target.send(detection)
                    if visible_marker_id is None:
                        print(f"Vision: target acquired using marker {detection.marker_id}", flush=True)
                    elif visible_marker_id != detection.marker_id:
                        print(
                            f"Vision: switched marker {visible_marker_id} -> {detection.marker_id}",
                            flush=True,
                        )
                    visible_marker_id = detection.marker_id
                elif visible_marker_id is not None:
                    print("Vision: target not detected; LANDING_TARGET paused", flush=True)
                    visible_marker_id = None
                target.send_heartbeat()
                now = time.monotonic()
                if now - report_at >= 1:
                    elapsed = now - report_at
                    rate = frames_seen / elapsed
                    mean_detection_ms = 1000 * detection_seconds / frames_seen
                    total_drops = getattr(frames, "dropped_frames", 0)
                    new_drops = total_drops - reported_drops
                    reported_drops = total_drops
                    drop_detail = f", {new_drops} stale dropped" if new_drops else ""
                    detail = "marker not visible"
                    if detection:
                        x, y, z = detection.body_frd
                        roll, pitch, yaw = np.degrees(detection.body_rpy)
                        detail = (
                            f"id {detection.marker_id}, body FRD [{x:+.2f}, {y:+.2f}, {z:+.2f}] m, "
                            f"target RPY [{roll:+.1f}, {pitch:+.1f}, {yaw:+.1f}] deg"
                        )
                    print(
                        f"Vision: {detections}/{frames_seen}; {rate:.1f} Hz, "
                        f"{mean_detection_ms:.1f} ms detect{drop_detail}; {detail}",
                        flush=True,
                    )
                    detections = frames_seen = 0
                    detection_seconds = 0.0
                    report_at = now
        except CameraUnavailable:
            stop.wait(1)
        except CameraDisconnected as error:
            if not stop.is_set():
                print(f"Camera: {error}; retrying", file=sys.stderr, flush=True)
                stop.wait(1)
        except (Disconnected, ConnectionError, OSError, RuntimeError) as error:
            if not stop.is_set():
                print(f"Bridge: {error}; retrying", file=sys.stderr, flush=True)
                target.close()
                stop.wait(1)
    target.close()
    print("Bridge: stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
