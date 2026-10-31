#!/usr/bin/env python3
"""Shared precision-landing code: ArUco pose estimation and MAVLink output.

This module runs on both the laptop (fed by Simulink) and the Raspberry Pi
(fed by the camera). It contains nothing Simulink- or Pi-specific; the
caller supplies the frame source. Dependencies: numpy, OpenCV with the
aruco module (OpenCV 4.7 or newer, or opencv-contrib), pymavlink.
"""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterator

import cv2
import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil


# Frame stream protocol shared with ArucoFrameServer.m and the Pi publisher.
# Little endian: magic "APSM", width, height, channels, frame number and
# payload byte count (all uint32), followed by packed RGB bytes.
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
    """Raised while the frame server has not started listening yet."""


def pack_frame_header(width: int, height: int, channels: int, frame_number: int, payload_size: int) -> bytes:
    return FRAME_HEADER.pack(FRAME_MAGIC, width, height, channels, frame_number, payload_size)


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
        marker_sizes_m: dict[int, float],
        horizontal_fov_deg: float,
        calibration_file: Path | None = None,
    ) -> None:
        """marker_sizes_m maps each ArUco ID on the landing pad to its side length."""
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("this OpenCV build has no aruco module (need OpenCV 4.7+ or opencv-contrib)")
        if not hasattr(cv2.aruco, marker_family):
            raise ValueError(f"OpenCV does not know ArUco family {marker_family!r}")
        dictionary_id = getattr(cv2.aruco, marker_family)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        if not marker_sizes_m:
            raise ValueError("at least one marker is needed")
        self.marker_sizes_m = {int(i): float(s) for i, s in marker_sizes_m.items()}
        self.marker_ids = sorted(self.marker_sizes_m)
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
        self.other_ids_seen: set[int] = set()
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

    def detect(self, image_rgb: np.ndarray) -> Detection | None:
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, _rejected = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        flat_ids = [int(i) for i in ids.reshape(-1)]
        # Remember every unknown ID seen so the status line can say "saw id 0"
        # instead of a bare "not visible" when the print and config disagree.
        self.other_ids_seen.update(i for i in flat_ids if i not in self.marker_sizes_m)

        height, width = image_rgb.shape[:2]
        camera_matrix, distortion = self.intrinsics(width, height)

        # Every marker on the pad has its own ID, so the ID alone fixes the
        # physical size and there is no scale to guess. When more than one
        # known marker is visible, use the one covering the most pixels: its
        # corners are located most precisely, and as the vehicle descends the
        # large marker leaves the frame and the small one takes over by itself.
        best: tuple[float, Detection] | None = None
        for index, marker_id in enumerate(flat_ids):
            marker_size_m = self.marker_sizes_m.get(marker_id)
            if marker_size_m is None:
                continue
            image_points = corners[index].reshape(4, 2).astype(np.float32)
            pixel_area = float(cv2.contourArea(image_points))
            if best is not None and pixel_area <= best[0]:
                continue
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
            best = (pixel_area, Detection(marker_id, marker_size_m, image_points, camera_xyz))

        return None if best is None else best[1]


class TcpFrameSource:
    """Client for the APSM frame stream (Simulink's ArucoFrameServer or the Pi publisher)."""

    def __init__(self, host: str, port: int, name: str = "frame server", hint: str = "") -> None:
        self.host = host
        self.port = port
        self.name = name
        self.hint = hint
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
                raise FrameDisconnected("camera stream closed")
            output.extend(chunk)
        if len(output) != size:
            raise FrameDisconnected("camera stream stopped")
        return bytes(output)

    def frames(self, stop_event: threading.Event) -> Iterator[tuple[int, np.ndarray]]:
        if not self.waiting_announced:
            hint = f" ({self.hint})" if self.hint else ""
            print(f"Camera: waiting for {self.name} at {self.host}:{self.port}{hint}", flush=True)
            self.waiting_announced = True
        try:
            with socket.create_connection((self.host, self.port), timeout=2.0) as connection:
                connection.settimeout(1.0)
                print(f"Camera: connected to {self.name} RGB stream", flush=True)
                self.waiting_announced = False
                while not stop_event.is_set():
                    header = self._receive_exact(connection, FRAME_HEADER.size, stop_event)
                    magic, width, height, channels, frame_number, payload_size = FRAME_HEADER.unpack(header)
                    expected_size = width * height * channels
                    if magic != FRAME_MAGIC or channels != 3 or payload_size != expected_size:
                        raise FrameDisconnected("invalid camera frame header")
                    if payload_size <= 0 or payload_size > MAX_FRAME_BYTES:
                        raise FrameDisconnected(f"unsafe frame size {payload_size}")
                    payload = self._receive_exact(connection, payload_size, stop_event)
                    image = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, channels)
                    yield frame_number, image
        except (ConnectionRefusedError, socket.timeout, TimeoutError) as error:
            raise FrameUnavailable from error
        except OSError as error:
            raise FrameDisconnected(str(error)) from error


class MavlinkLandingTarget:
    def __init__(self, endpoint: str, dry_run: bool = False) -> None:
        self.endpoint = endpoint
        self.dry_run = dry_run
        self.connection = None
        self.target_system = 1
        self.target_component = 1
        self.last_heartbeat = 0.0

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

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None

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

    def drain(self) -> None:
        """Read whatever ArduPilot sent so its PrecLand status messages get printed."""
        if self.connection is None:
            return
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            self._print_status_text(message)

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
        raise ValueError("endpoint must be HOST:PORT")
    try:
        port = int(port_text)
    except ValueError as error:
        raise ValueError("endpoint port must be an integer") from error
    if not 1 <= port <= 65535:
        raise ValueError("endpoint port must be between 1 and 65535")
    return host, port


def load_config(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_markers(config: dict[str, Any]) -> dict[int, float]:
    """Return {marker_id: side_length_m} from the config's "markers" list.

    Every marker on the landing pad has its own ID, so the ID seen in the
    image fixes the physical size and no vehicle height is needed to tell a
    large outer marker from a small inner one.
    """
    markers = config.get("markers")
    if markers is None:
        old_keys = [k for k in ("marker_id", "marker_size_m", "outer_marker_size_m") if k in config]
        hint = f" (found old keys {', '.join(old_keys)})" if old_keys else ""
        raise ValueError(
            'config needs a "markers" list such as '
            '[{"id": 9, "size_m": 2.2}, {"id": 10, "size_m": 0.3}]' + hint
        )
    if not isinstance(markers, list) or not markers:
        raise ValueError('"markers" must be a non-empty list of {"id", "size_m"} entries')
    sizes: dict[int, float] = {}
    for entry in markers:
        if not isinstance(entry, dict) or "id" not in entry or "size_m" not in entry:
            raise ValueError(f'each marker needs "id" and "size_m", got {entry!r}')
        marker_id = int(entry["id"])
        size_m = float(entry["size_m"])
        if marker_id < 0:
            raise ValueError(f"marker id {marker_id} must not be negative")
        if size_m <= 0:
            raise ValueError(f"marker {marker_id}: size_m must be positive")
        if marker_id in sizes:
            raise ValueError(f"marker id {marker_id} is listed twice; each id has exactly one size")
        sizes[marker_id] = size_m
    return sizes


def build_detector(config: dict[str, Any], calibration_file: Path | None = None) -> ArucoDetector:
    marker_sizes_m = parse_markers(config)
    largest_m = max(marker_sizes_m.values())
    if "marker_image_width_m" in config:
        marker_image_width_m = float(config["marker_image_width_m"])
        if marker_image_width_m < largest_m:
            raise ValueError("marker_image_width_m must be at least the largest marker size")

    listing = ", ".join(f"ID {i} at {s:.3f} m" for i, s in sorted(marker_sizes_m.items()))
    print(f"Markers: {config['marker_family']}, {listing}", flush=True)
    return ArucoDetector(
        config["marker_family"],
        marker_sizes_m,
        float(config["camera_hfov_deg"]),
        calibration_file,
    )


def build_mavlink(config: dict[str, Any], endpoint_override: str | None, dry_run: bool) -> MavlinkLandingTarget:
    return MavlinkLandingTarget(endpoint_override or config["mavlink_endpoint"], dry_run)


FrameCallback = Callable[[np.ndarray, "Detection | None"], None]


def run_bridge(
    config: dict[str, Any],
    detector: ArucoDetector,
    frame_source,
    mavlink: MavlinkLandingTarget,
    stop_event: threading.Event,
    on_frame: FrameCallback | None = None,
) -> int:
    """Detect on every frame, send every detection, retry on any disconnect.

    frame_source must provide frames(stop_event) yielding (frame_number, rgb_image).
    on_frame, if given, is called with every image and its detection (or None);
    the Pi uses it to publish frames to a laptop viewer.
    """
    detections = 0
    frames = 0
    last_report = time.monotonic()
    last_detection: Detection | None = None
    if mavlink.dry_run:
        mavlink.connect()

    while not stop_event.is_set():
        try:
            if mavlink.connection is None and not mavlink.dry_run:
                mavlink.connect()

            for _frame_number, image in frame_source.frames(stop_event):
                frames += 1
                detection = detector.detect(image)
                if detection is not None:
                    last_detection = detection
                    detections += 1
                    mavlink.send(detection)
                if on_frame is not None:
                    on_frame(image, detection)
                mavlink.drain()

                now = time.monotonic()
                if now - last_report >= 1.0:
                    if last_detection is None:
                        wanted = ", ".join(str(i) for i in detector.marker_ids)
                        detail = f"marker ids {wanted} not visible"
                        if detector.other_ids_seen:
                            others = ", ".join(str(i) for i in sorted(detector.other_ids_seen))
                            detail += f" (saw other marker ids: {others}; check the markers list in the config)"
                    else:
                        vector = last_detection.body_frd_m
                        detail = (
                            f"id {last_detection.marker_id} ({last_detection.marker_size_m:.3f} m) "
                            f"body_frd=[{vector[0]:+.2f}, {vector[1]:+.2f}, {vector[2]:+.2f}] m "
                            f"distance={last_detection.distance_m:.2f} m"
                        )
                    print(f"Vision: {detections}/{frames} detections; {detail}", flush=True)
                    detector.other_ids_seen.clear()
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
