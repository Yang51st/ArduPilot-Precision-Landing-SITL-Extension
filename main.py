#!/usr/bin/env python3
"""ArUco precision landing bridge for Simulink or a Raspberry Pi camera."""

from __future__ import annotations

import argparse
import json
import math
import os
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
        if calibration:
            data = json.loads(calibration.read_text(encoding="utf-8"))
            self.camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
            self.distortion = np.asarray(data.get("distortion", []), dtype=np.float64)
            if data.get("image_size"):
                self.calibration_size = tuple(map(int, data["image_size"]))
        self.object_corners = np.array(
            [[-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]],
            dtype=np.float32,
        )

    def intrinsics(self, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        if self.camera_matrix is not None:
            if self.calibration_size and self.calibration_size != (width, height):
                raise ValueError(f"calibration is for {self.calibration_size}, not {(width, height)}")
            return self.camera_matrix, self.distortion
        focal = width / 2 / math.tan(math.radians(self.fov) / 2)
        matrix = np.array(
            [[focal, 0, (width - 1) / 2], [0, focal, (height - 1) / 2], [0, 0, 1]],
            dtype=np.float64,
        )
        return matrix, np.zeros(5)

    def detect(self, image: np.ndarray) -> Detection | None:
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        if ids is None:
            return None
        detected_ids = [int(marker_id) for marker_id in ids.reshape(-1)]
        visible_ids = [marker_id for marker_id in detected_ids if marker_id in self.sizes]
        if len(visible_ids) != len(set(visible_ids)):
            return None
        height, width = image.shape[:2]
        matrix, distortion = self.intrinsics(width, height)
        best = None
        for marker_corners, marker_id in zip(corners, detected_ids):
            if marker_id not in self.sizes:
                continue
            points = marker_corners.reshape(4, 2).astype(np.float32)
            area = float(cv2.contourArea(points))
            if best and area <= best[0]:
                continue
            size = self.sizes[marker_id]
            ok, rotation_vector, translation = cv2.solvePnP(
                self.object_corners * size,
                points,
                matrix,
                distortion,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                continue
            xyz = translation.reshape(3).astype(np.float32)
            if np.all(np.isfinite(xyz)) and xyz[2] > 0:
                aruco_to_camera = cv2.Rodrigues(rotation_vector)[0]
                target_to_body = CAMERA_TO_BODY @ aruco_to_camera @ ARUCO_FROM_TARGET
                rpy, quaternion = orientation_from_rotation(target_to_body)
                best = (area, Detection(marker_id, size, xyz, rpy, quaternion))
        return best[1] if best else None


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

    def frames(self, stop: threading.Event) -> Iterator[np.ndarray]:
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise CameraDisconnected("Picamera2 is not installed") from error
        camera = None
        try:
            camera = Picamera2()
            main = {
                "size": (int(self.config["camera_width"]), int(self.config["camera_height"])),
                # Picamera2's BGR888 name produces RGB byte order in memory.
                "format": "BGR888",
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
        print(f"Camera: Picamera2 {main['size'][0]}x{main['size'][1]}", flush=True)
        try:
            while not stop.is_set():
                yield camera.capture_array("main")
        finally:
            camera.stop()
            camera.close()


class LandingTarget:
    def __init__(self, endpoint: str, baud: int | None, dry_run: bool, config: dict) -> None:
        self.endpoint, self.baud, self.dry_run = endpoint, baud, dry_run
        self.connection = None
        self.last_heartbeat = 0.0
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
                1,
            )
            self.align_yaw(detection)
            now = time.monotonic()
            if now - self.last_heartbeat >= 1:
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
        self.last_heartbeat = 0.0
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
    report_at = time.monotonic()
    detection = None
    while not stop.is_set():
        try:
            if target.connection is None and not args.dry_run:
                target.connect()
            target.drain()
            for image in frames.frames(stop):
                frames_seen += 1
                target.drain()
                detection = detector.detect(image)
                if detection:
                    detections += 1
                    target.send(detection)
                now = time.monotonic()
                if now - report_at >= 1:
                    detail = "marker not visible"
                    if detection:
                        x, y, z = detection.body_frd
                        roll, pitch, yaw = np.degrees(detection.body_rpy)
                        detail = (
                            f"id {detection.marker_id}, body FRD [{x:+.2f}, {y:+.2f}, {z:+.2f}] m, "
                            f"target RPY [{roll:+.1f}, {pitch:+.1f}, {yaw:+.1f}] deg"
                        )
                    print(f"Vision: {detections}/{frames_seen}; {detail}", flush=True)
                    detections = frames_seen = 0
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
