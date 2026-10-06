#!/usr/bin/env python3
"""Detect an ArUco landing marker and send MAVLink LANDING_TARGET messages.

The default source is the RGB stream published by mavlinkMST_ArucoCamera.slx.
The same program can use a real camera on a Raspberry Pi with --source camera.
"""

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
FRAME_MAGIC = b"APSM"
MAX_FRAME_BYTES = 32 * 1024 * 1024
REQUIRED_PARAMETERS = {
    "PLND_ENABLED": 1.0,
    "PLND_TYPE": 1.0,
    "PLND_EST_TYPE": 0.0,
    "PLND_ALT_MIN": 0.0,
    "PLND_ALT_MAX": 0.0,
    "SERIAL2_PROTOCOL": 2.0,
}


class BridgeDisconnected(RuntimeError):
    """Raised when a frame or MAVLink connection is lost."""


class FrameDisconnected(BridgeDisconnected):
    """Raised when only the camera-frame connection is lost."""


class FrameUnavailable(FrameDisconnected):
    """Raised while Simulink has not opened its frame server yet."""


@dataclass(frozen=True)
class Detection:
    marker_id: int
    marker_size_m: float
    corners: np.ndarray
    camera_xyz_m: np.ndarray

    @property
    def body_frd_m(self) -> np.ndarray:
        camera_x, camera_y, camera_z = self.camera_xyz_m
        return np.array([-camera_y, camera_x, camera_z], dtype=np.float32)

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.camera_xyz_m))


class ArucoDetector:
    def __init__(
        self,
        marker_family: str,
        marker_id: int,
        marker_size_m: float,
        outer_marker_size_m: float | None,
        horizontal_fov_deg: float,
        calibration_file: Path | None = None,
    ) -> None:
        if not hasattr(cv2.aruco, marker_family):
            raise ValueError(f"OpenCV does not know ArUco family {marker_family!r}")
        dictionary_id = getattr(cv2.aruco, marker_family)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.marker_id = marker_id
        self.marker_size_m = marker_size_m
        self.marker_sizes_m = [marker_size_m]
        if outer_marker_size_m is not None:
            self.marker_sizes_m.append(outer_marker_size_m)
        self.horizontal_fov_deg = horizontal_fov_deg
        self.calibration_file = calibration_file
        self._camera_matrix: np.ndarray | None = None
        self._distortion: np.ndarray | None = None
        self._calibration_size: tuple[int, int] | None = None

        self.unit_object_points = np.array(
            [
                [-0.5, 0.5, 0.0],
                [0.5, 0.5, 0.0],
                [0.5, -0.5, 0.0],
                [-0.5, -0.5, 0.0],
            ],
            dtype=np.float32,
        )
        if calibration_file is not None:
            calibration = json.loads(calibration_file.read_text(encoding="utf-8"))
            self._camera_matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64)
            self._distortion = np.asarray(calibration.get("distortion", []), dtype=np.float64)
            image_size = calibration.get("image_size")
            if image_size:
                self._calibration_size = (int(image_size[0]), int(image_size[1]))

    def intrinsics(self, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        if self._camera_matrix is not None:
            if self._calibration_size and self._calibration_size != (width, height):
                raise ValueError(
                    f"calibration image size {self._calibration_size} does not match frame {(width, height)}"
                )
            assert self._distortion is not None
            return self._camera_matrix, self._distortion

        focal_length = (width / 2.0) / math.tan(math.radians(self.horizontal_fov_deg) / 2.0)
        camera_matrix = np.array(
            [
                [focal_length, 0.0, (width - 1) / 2.0],
                [0.0, focal_length, (height - 1) / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        return camera_matrix, np.zeros(5, dtype=np.float64)

    def detect(
        self, image_rgb: np.ndarray, expected_camera_z_m: float | None = None
    ) -> Detection | None:
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, _rejected = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        matching = np.flatnonzero(ids.reshape(-1) == self.marker_id)
        if matching.size == 0:
            return None

        height, width = image_rgb.shape[:2]
        camera_matrix, distortion = self.intrinsics(width, height)

        # The eAruco landing target encodes the same ID at two physical
        # scales. A corner set alone cannot say which scale produced it, so
        # evaluate both pose hypotheses and compare them with vehicle height.
        # Until a height is available, withholding a multiscale detection is
        # safer than sending a target vector with a 9x scale error.
        if len(self.marker_sizes_m) > 1 and expected_camera_z_m is None:
            return None

        hypotheses: list[tuple[float, Detection]] = []
        for match in matching:
            image_points = corners[int(match)].reshape(4, 2).astype(np.float32)
            for marker_size_m in self.marker_sizes_m:
                success, _rotation, translation = cv2.solvePnP(
                    self.unit_object_points * marker_size_m,
                    image_points,
                    camera_matrix,
                    distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not success:
                    continue
                camera_xyz = translation.reshape(3).astype(np.float32)
                if not np.all(np.isfinite(camera_xyz)) or camera_xyz[2] <= 0:
                    continue
                if expected_camera_z_m is None:
                    score = 0.0
                else:
                    score = abs(math.log(float(camera_xyz[2]) / expected_camera_z_m))
                hypotheses.append(
                    (
                        score,
                        Detection(self.marker_id, marker_size_m, image_points, camera_xyz),
                    )
                )

        if not hypotheses:
            return None
        return min(hypotheses, key=lambda item: item[0])[1]


class SimulinkFrameSource:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.waiting_announced = False

    @staticmethod
    def _receive_exact(connection: socket.socket, size: int, stop_event: threading.Event) -> bytes:
        output = bytearray()
        while len(output) < size and not stop_event.is_set():
            try:
                chunk = connection.recv(size - len(output))
            except socket.timeout:
                continue
            if not chunk:
                raise FrameDisconnected("Simulink camera stream closed")
            output.extend(chunk)
        if len(output) != size:
            raise FrameDisconnected("camera stream stopped")
        return bytes(output)

    def frames(self, stop_event: threading.Event) -> Iterator[tuple[int, np.ndarray]]:
        if not self.waiting_announced:
            print(
                f"Camera: waiting for Simulink at {self.host}:{self.port} "
                "(start the mavlinkMST_ArucoCamera model)",
                flush=True,
            )
            self.waiting_announced = True
        try:
            with socket.create_connection((self.host, self.port), timeout=2.0) as connection:
                connection.settimeout(1.0)
                print("Camera: connected to Simulink RGB stream", flush=True)
                self.waiting_announced = False
                while not stop_event.is_set():
                    header = self._receive_exact(connection, FRAME_HEADER.size, stop_event)
                    magic, width, height, channels, frame_number, payload_size = FRAME_HEADER.unpack(header)
                    expected_size = width * height * channels
                    if magic != FRAME_MAGIC or channels != 3 or payload_size != expected_size:
                        raise FrameDisconnected("invalid Simulink camera frame header")
                    if payload_size <= 0 or payload_size > MAX_FRAME_BYTES:
                        raise FrameDisconnected(f"unsafe frame size {payload_size}")
                    payload = self._receive_exact(connection, payload_size, stop_event)
                    image = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, channels)
                    yield frame_number, image
        except ConnectionRefusedError as error:
            raise FrameUnavailable from error
        except OSError as error:
            raise FrameDisconnected(str(error)) from error


class CameraFrameSource:
    def __init__(self, camera_index: int) -> None:
        self.camera_index = camera_index

    def frames(self, stop_event: threading.Event) -> Iterator[tuple[int, np.ndarray]]:
        camera = cv2.VideoCapture(self.camera_index)
        if not camera.isOpened():
            raise FrameDisconnected(f"cannot open camera index {self.camera_index}")
        frame_number = 0
        try:
            while not stop_event.is_set():
                success, image_bgr = camera.read()
                if not success:
                    raise FrameDisconnected("camera read failed")
                frame_number += 1
                yield frame_number, cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        finally:
            camera.release()


class MavlinkLandingTarget:
    def __init__(self, endpoint: str, dry_run: bool = False) -> None:
        self.endpoint = endpoint
        self.dry_run = dry_run
        self.connection = None
        self.target_system = 1
        self.target_component = 1
        self.last_heartbeat = 0.0
        self.local_down_m: float | None = None

    def connect(self) -> None:
        if self.dry_run:
            print("MAVLink: dry-run mode; LANDING_TARGET messages will not be sent", flush=True)
            return
        print(f"MAVLink: connecting to {self.endpoint}", flush=True)
        self.connection = mavutil.mavlink_connection(
            self.endpoint,
            source_system=42,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
            autoreconnect=True,
        )
        heartbeat = self.connection.wait_heartbeat(timeout=5)
        if heartbeat is None:
            self.close()
            raise BridgeDisconnected("no ArduPilot heartbeat on companion link")
        self.target_system = self.connection.target_system
        self.target_component = self.connection.target_component
        print(
            f"MAVLink: connected to system {self.target_system}, component {self.target_component}",
            flush=True,
        )
        self._check_parameters()
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            50000,
            0,
            0,
            0,
            0,
            0,
        )

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        self.local_down_m = None

    def _check_parameters(self) -> None:
        assert self.connection is not None
        for name in REQUIRED_PARAMETERS:
            self.connection.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                name.encode("ascii"),
                -1,
            )

        received: dict[str, float] = {}
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and len(received) < len(REQUIRED_PARAMETERS):
            message = self.connection.recv_match(blocking=True, timeout=0.2)
            if message is None:
                continue
            if message.get_type() == "PARAM_VALUE":
                name = str(message.param_id).rstrip("\x00")
                if name in REQUIRED_PARAMETERS:
                    received[name] = float(message.param_value)
            self._print_status_text(message)

        incorrect = [
            f"{name}={received.get(name, 'missing')} (expected {expected:g})"
            for name, expected in REQUIRED_PARAMETERS.items()
            if name not in received or not math.isclose(received[name], expected, abs_tol=0.01)
        ]
        if incorrect:
            raise RuntimeError("SITL precision-landing parameters are incorrect: " + ", ".join(incorrect))
        print("MAVLink: ArduPilot precision-landing parameters verified", flush=True)

    @staticmethod
    def _print_status_text(message) -> None:
        if message.get_type() == "STATUSTEXT":
            text = str(message.text).rstrip("\x00")
            if "PrecLand" in text or "Plnd" in text:
                print(f"ArduPilot: {text}", flush=True)

    def _consume_message(self, message) -> None:
        self._print_status_text(message)
        if (
            message.get_type() == "LOCAL_POSITION_NED"
            and message.get_srcSystem() == self.target_system
        ):
            self.local_down_m = float(message.z)

    def drain(self) -> None:
        if self.connection is None:
            return
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            self._consume_message(message)

    def expected_camera_z_m(self, ground_down_m: float) -> float | None:
        if self.local_down_m is None:
            return None
        height = ground_down_m - self.local_down_m
        return height if height > 0.05 else None

    def send(self, detection: Detection) -> None:
        if self.dry_run:
            return
        if self.connection is None:
            raise BridgeDisconnected("MAVLink is not connected")

        camera_x, camera_y, camera_z = (float(value) for value in detection.camera_xyz_m)
        body_x, body_y, body_z = (float(value) for value in detection.body_frd_m)
        angle_x = math.atan2(camera_x, camera_z)
        angle_y = math.atan2(camera_y, camera_z)
        angular_size = 2.0 * math.atan2(detection.marker_size_m / 2.0, camera_z)
        target_type = mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL
        try:
            self.connection.mav.landing_target_send(
                time.monotonic_ns() // 1000,
                detection.marker_id,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                angle_x,
                angle_y,
                detection.distance_m,
                angular_size,
                angular_size,
                body_x,
                body_y,
                body_z,
                [1.0, 0.0, 0.0, 0.0],
                target_type,
                1,
            )
        except (OSError, socket.error) as error:
            self.close()
            raise BridgeDisconnected(f"MAVLink send failed: {error}") from error

        now = time.monotonic()
        if now - self.last_heartbeat >= 1.0:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.last_heartbeat = now


def parse_host_port(value: str) -> tuple[str, int]:
    host, separator, port_text = value.rpartition(":")
    if not separator or not host:
        raise argparse.ArgumentTypeError("endpoint must be HOST:PORT")
    try:
        port = int(port_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("endpoint port must be an integer") from error
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("endpoint port must be between 1 and 65535")
    return host, port


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="shared JSON camera/marker configuration")
    parser.add_argument("--source", choices=("simulink", "camera"), default="simulink")
    parser.add_argument("--frame-endpoint", type=parse_host_port, help="override Simulink HOST:PORT")
    parser.add_argument("--mavlink", help="override pymavlink endpoint, e.g. tcp:127.0.0.1:5763")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index for --source camera")
    parser.add_argument("--calibration", type=Path, help="real-camera calibration JSON")
    parser.add_argument("--dry-run", action="store_true", help="detect but do not send MAVLink")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    frame_endpoint = args.frame_endpoint or (config["frame_host"], int(config["frame_port"]))
    mavlink_endpoint = args.mavlink or config["mavlink_endpoint"]
    marker_size_m = float(config["marker_size_m"])
    outer_marker_size_m = (
        float(config["outer_marker_size_m"])
        if "outer_marker_size_m" in config
        else None
    )
    marker_image_width_m = float(config.get("marker_image_width_m", marker_size_m))
    if marker_size_m <= 0 or marker_image_width_m <= 0:
        raise ValueError("marker_size_m and marker_image_width_m must be positive")
    if outer_marker_size_m is not None and outer_marker_size_m <= marker_size_m:
        raise ValueError("outer_marker_size_m must exceed marker_size_m")
    largest_marker_size_m = outer_marker_size_m or marker_size_m
    if largest_marker_size_m > marker_image_width_m:
        raise ValueError("marker sizes cannot exceed marker_image_width_m")

    if outer_marker_size_m is None:
        scale_detail = f"{marker_size_m:.3f} m"
    else:
        scale_detail = f"{outer_marker_size_m:.3f} m outer / {marker_size_m:.3f} m embedded"
    print(f"Marker: ArUco ID {int(config['marker_id'])}, {scale_detail}", flush=True)
    detector = ArucoDetector(
        config["marker_family"],
        int(config["marker_id"]),
        marker_size_m,
        outer_marker_size_m,
        float(config["camera_hfov_deg"]),
        args.calibration,
    )
    if args.source == "simulink":
        frame_source = SimulinkFrameSource(*frame_endpoint)
    else:
        frame_source = CameraFrameSource(args.camera_index)

    stop_event = threading.Event()

    def stop(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    mavlink = MavlinkLandingTarget(mavlink_endpoint, args.dry_run)
    detections = 0
    frames = 0
    last_report = time.monotonic()
    last_detection: Detection | None = None
    if args.dry_run:
        mavlink.connect()

    while not stop_event.is_set():
        try:
            if mavlink.connection is None and not args.dry_run:
                mavlink.connect()

            for _frame_number, image in frame_source.frames(stop_event):
                frames += 1
                detection = detector.detect(
                    image,
                    mavlink.expected_camera_z_m(float(config["ground_down_m"])),
                )
                if detection is not None:
                    last_detection = detection
                    detections += 1
                    mavlink.send(detection)
                mavlink.drain()

                now = time.monotonic()
                if now - last_report >= 1.0:
                    if last_detection is None:
                        detail = "target not visible"
                    else:
                        vector = last_detection.body_frd_m
                        detail = (
                            f"body_frd=[{vector[0]:+.2f}, {vector[1]:+.2f}, {vector[2]:+.2f}] m "
                            f"distance={last_detection.distance_m:.2f} m "
                            f"scale={last_detection.marker_size_m:.3f} m"
                        )
                    print(f"Vision: {detections}/{frames} detections; {detail}", flush=True)
                    detections = 0
                    frames = 0
                    last_report = now
                    last_detection = None
        except FrameUnavailable:
            if stop_event.is_set():
                break
            stop_event.wait(1.0)
        except FrameDisconnected as error:
            if stop_event.is_set():
                break
            print(f"Camera: {error}; retrying in 1 s", file=sys.stderr, flush=True)
            stop_event.wait(1.0)
        except (BridgeDisconnected, ConnectionError, OSError, RuntimeError) as error:
            if stop_event.is_set():
                break
            print(f"Bridge: {error}; retrying in 1 s", file=sys.stderr, flush=True)
            mavlink.close()
            stop_event.wait(1.0)

    mavlink.close()
    print("Bridge: stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
