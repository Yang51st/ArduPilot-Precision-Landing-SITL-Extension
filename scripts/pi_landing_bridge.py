#!/usr/bin/env python3
"""Raspberry Pi bridge: camera frames in, MAVLink LANDING_TARGET out.

Runs on the Pi with precland_core.py beside it. Frames come from the Pi
camera through Picamera2, or from a USB webcam through OpenCV with
--source opencv. With --publish HOST:PORT the frames, with the detection
drawn on them, are also served to a laptop running view_frames.py over the
same APSM stream protocol that Simulink uses, so the laptop needs nothing new.

Until a flight controller is wired up, always run with --dry-run:

    .venv/bin/python pi_landing_bridge.py --config pi_landing_config.json \
        --dry-run --publish 0.0.0.0:5600
"""

from __future__ import annotations

import argparse
import signal
import socket
import threading
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np

from precland_core import (
    Detection,
    FrameDisconnected,
    build_detector,
    build_mavlink,
    load_config,
    pack_frame_header,
    parse_host_port,
    run_bridge,
)


class Picamera2FrameSource:
    """Frames from the Pi camera (CSI ribbon) through libcamera / Picamera2."""

    def __init__(self, width: int, height: int, fps: float, sensor_size: tuple[int, int] | None) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.sensor_size = sensor_size

    def frames(self, stop_event: threading.Event) -> Iterator[tuple[int, np.ndarray]]:
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise FrameDisconnected(
                "picamera2 is not importable; the venv must be created with "
                "--system-site-packages after apt install python3-picamera2 (see setup_pi.sh)"
            ) from error

        try:
            camera = Picamera2()
            # libcamera names 24-bit formats by their memory order read as a
            # little-endian word, so "BGR888" delivers R,G,B byte order, which
            # is what precland_core expects. If colours look swapped in
            # view_frames.py, change this to "RGB888". Detection converts to
            # grayscale and is unaffected either way.
            main = {"size": (self.width, self.height), "format": "BGR888"}
            extra = {}
            if self.sensor_size is not None:
                # Request the binned full-field-of-view sensor mode; without it
                # libcamera may pick a cropped mode for small outputs.
                extra["raw"] = {"size": self.sensor_size}
            configuration = camera.create_video_configuration(
                main=main, controls={"FrameRate": self.fps}, **extra
            )
            camera.configure(configuration)
            camera.start()
        except Exception as error:  # Picamera2 raises several types here
            raise FrameDisconnected(f"cannot start the Pi camera: {error}") from error

        print(f"Camera: Picamera2 {self.width}x{self.height} at {self.fps:g} fps", flush=True)
        frame_number = 0
        try:
            while not stop_event.is_set():
                image = camera.capture_array("main")
                frame_number += 1
                yield frame_number, image
        finally:
            camera.stop()
            camera.close()


class OpenCvFrameSource:
    """Frames from a USB webcam through OpenCV. Not for the CSI camera."""

    def __init__(self, camera_index: int) -> None:
        self.camera_index = camera_index

    def frames(self, stop_event: threading.Event) -> Iterator[tuple[int, np.ndarray]]:
        camera = cv2.VideoCapture(self.camera_index)
        if not camera.isOpened():
            raise FrameDisconnected(f"cannot open camera index {self.camera_index}")
        print(f"Camera: OpenCV camera index {self.camera_index}", flush=True)
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


class FramePublisher:
    """Serve the newest frame to one laptop viewer over TCP using the APSM header.

    Only the latest frame is kept, so a slow USB link drops frames instead of
    building up lag. Publishing costs nothing while no viewer is connected.
    """

    def __init__(self, host: str, port: int, scale: float = 1.0, overlay: bool = True) -> None:
        self.host = host
        self.port = port
        self.scale = scale
        self.overlay = overlay
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(1)
        self._server.settimeout(0.5)
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._latest: tuple[int, np.ndarray] | None = None
        self._frame_number = 0
        self._viewer_connected = False
        self._stop_event: threading.Event | None = None
        self._thread: threading.Thread | None = None

    def start(self, stop_event: threading.Event) -> None:
        self._stop_event = stop_event
        self._thread = threading.Thread(target=self._serve, name="frame-publisher", daemon=True)
        self._thread.start()
        print(
            f"Publisher: serving frames on {self.host}:{self.port} "
            "(run scripts/view_frames.py on the laptop)",
            flush=True,
        )

    def close(self) -> None:
        self._server.close()

    def on_frame(self, image: np.ndarray, detection: Detection | None) -> None:
        if not self._viewer_connected:
            return
        frame = image
        if self.overlay and detection is not None:
            frame = image.copy()
            points = detection.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [points], True, (0, 255, 0), 2)
            label = f"id {detection.marker_id}  z={float(detection.camera_xyz_m[2]):.2f} m"
            origin = (int(points[0, 0, 0]), max(int(points[0, 0, 1]) - 8, 12))
            cv2.putText(frame, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        frame = np.ascontiguousarray(frame)
        with self._lock:
            self._frame_number += 1
            self._latest = (self._frame_number, frame)
        self._ready.set()

    def _serve(self) -> None:
        assert self._stop_event is not None
        while not self._stop_event.is_set():
            try:
                client, address = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"Publisher: viewer connected from {address[0]}", flush=True)
            client.settimeout(5.0)
            self._viewer_connected = True
            try:
                while not self._stop_event.is_set():
                    if not self._ready.wait(0.5):
                        continue
                    with self._lock:
                        self._ready.clear()
                        item = self._latest
                    if item is None:
                        continue
                    number, frame = item
                    height, width, channels = frame.shape
                    header = pack_frame_header(width, height, channels, number, frame.nbytes)
                    client.sendall(header + frame.tobytes())
            except OSError as error:
                print(f"Publisher: viewer disconnected ({error})", flush=True)
            finally:
                self._viewer_connected = False
                client.close()


def endpoint_argument(value: str) -> tuple[str, int]:
    try:
        return parse_host_port(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, required=True, help="JSON camera/marker configuration")
    parser.add_argument("--source", choices=("picamera2", "opencv"), default="picamera2")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index for --source opencv")
    parser.add_argument("--publish", type=endpoint_argument, metavar="HOST:PORT", help="serve frames to a laptop viewer, e.g. 0.0.0.0:5600")
    parser.add_argument("--publish-scale", type=float, default=1.0, help="shrink published frames, e.g. 0.5")
    parser.add_argument("--no-overlay", action="store_true", help="publish raw frames without the detection drawn")
    parser.add_argument("--calibration", type=Path, help="camera calibration JSON")
    parser.add_argument("--mavlink", help="override pymavlink endpoint (deferred until a flight controller is wired)")
    parser.add_argument("--dry-run", action="store_true", help="detect but do not send MAVLink")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    config = load_config(args.config)
    detector = build_detector(config, args.calibration)

    if args.source == "picamera2":
        sensor_size = None
        if "camera_sensor_width" in config and "camera_sensor_height" in config:
            sensor_size = (int(config["camera_sensor_width"]), int(config["camera_sensor_height"]))
        frame_source = Picamera2FrameSource(
            int(config["camera_width"]),
            int(config["camera_height"]),
            float(config["camera_fps"]),
            sensor_size,
        )
    else:
        frame_source = OpenCvFrameSource(args.camera_index)

    if not args.dry_run:
        print(
            "Warning: no flight controller link has been set up yet; "
            "use --dry-run until the UART work is done",
            flush=True,
        )
    mavlink = build_mavlink(config, args.mavlink, args.dry_run)

    stop_event = threading.Event()

    def stop(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    publisher = None
    if args.publish is not None:
        publisher = FramePublisher(*args.publish, scale=args.publish_scale, overlay=not args.no_overlay)
        publisher.start(stop_event)

    try:
        return run_bridge(
            config,
            detector,
            frame_source,
            mavlink,
            stop_event,
            publisher.on_frame if publisher is not None else None,
        )
    finally:
        if publisher is not None:
            publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
