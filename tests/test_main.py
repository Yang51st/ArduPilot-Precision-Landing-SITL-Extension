#!/usr/bin/env python3
"""Unit tests for camera-resolution and grayscale detection behavior."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import threading
import time
import types
import unittest
from datetime import datetime
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

import main


class ArucoDetectorTests(unittest.TestCase):
    def config(self) -> dict:
        return {
            "marker_family": "DICT_4X4_250",
            "markers": [{"id": 9, "size_m": 0.18}],
            "camera_hfov_deg": 62.2,
        }

    def test_scales_same_aspect_ratio_calibration(self) -> None:
        calibration = {
            "camera_matrix": [[500.0, 0.0, 320.0], [0.0, 501.0, 240.0], [0.0, 0.0, 1.0]],
            "distortion": [0.1, -0.2, 0.0, 0.0, 0.0],
            "image_size": [640, 480],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            path.write_text(json.dumps(calibration), encoding="utf-8")
            detector = main.ArucoDetector(self.config(), path)
            matrix, distortion = detector.intrinsics(960, 720)

        np.testing.assert_allclose(
            matrix,
            [[750.0, 0.0, 480.0], [0.0, 751.5, 360.0], [0.0, 0.0, 1.0]],
        )
        np.testing.assert_allclose(distortion, calibration["distortion"])

    def test_rejects_different_aspect_ratio_calibration(self) -> None:
        calibration = {
            "camera_matrix": [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
            "distortion": [],
            "image_size": [640, 480],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            path.write_text(json.dumps(calibration), encoding="utf-8")
            detector = main.ArucoDetector(self.config(), path)
            with self.assertRaisesRegex(ValueError, "aspect ratio changed"):
                detector.intrinsics(1280, 720)

    def test_rejects_calibration_from_cropped_sensor_mode(self) -> None:
        calibration = {
            "camera_matrix": [[1259.0, 0.0, 320.0], [0.0, 1259.0, 240.0], [0.0, 0.0, 1.0]],
            "distortion": [],
            "image_size": [640, 480],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            path.write_text(json.dumps(calibration), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "same sensor mode"):
                main.ArucoDetector(self.config(), path)

    def test_detects_directly_from_grayscale_luma(self) -> None:
        detector = main.ArucoDetector(self.config(), None)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        marker = cv2.aruco.generateImageMarker(dictionary, 9, 180)
        image = np.full((720, 960), 255, dtype=np.uint8)
        image[270:450, 390:570] = marker

        detection = detector.detect(image)

        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 9)
        self.assertGreater(detection.distance, 0)

    def test_selects_smallest_configured_visible_marker(self) -> None:
        config = self.config()
        config["markers"] = [
            {"id": 9, "size_m": 0.18},
            {"id": 7, "size_m": 0.06},
        ]
        detector = main.ArucoDetector(config, None)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        image = np.full((720, 960), 255, dtype=np.uint8)
        image[100:320, 100:320] = cv2.aruco.generateImageMarker(dictionary, 9, 220)
        image[400:500, 700:800] = cv2.aruco.generateImageMarker(dictionary, 7, 100)

        detection = detector.detect(image)

        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 7)
        self.assertEqual(detection.size_m, 0.06)


class LandingTargetTests(unittest.TestCase):
    def test_qland_is_active_only_when_armed_in_qland_mode(self) -> None:
        target = main.LandingTarget("unused", None, True, {})

        target.flight_mode = "QLAND"
        self.assertFalse(target.qland_active)
        target.armed = True
        self.assertTrue(target.qland_active)
        target.flight_mode = "QRTL"
        self.assertFalse(target.qland_active)

    def test_sends_metric_body_frd_position_and_independent_heartbeat(self) -> None:
        class FakeMav:
            def __init__(self) -> None:
                self.landing_target = None
                self.heartbeats = []

            def landing_target_send(self, *fields) -> None:
                self.landing_target = fields

            def heartbeat_send(self, *fields) -> None:
                self.heartbeats.append(fields)

        class FakeConnection:
            def __init__(self) -> None:
                self.mav = FakeMav()

            def close(self) -> None:
                pass

        target = main.LandingTarget("unused", None, False, {})
        target.connection = FakeConnection()
        detection = main.Detection(
            marker_id=7,
            size_m=0.06,
            camera_xyz=np.array([1.0, 2.0, 3.0], dtype=np.float32),
            body_rpy=np.zeros(3, dtype=np.float32),
            body_quaternion=(1.0, 0.0, 0.0, 0.0),
        )

        target.send(detection)
        target.send_heartbeat()

        fields = target.connection.mav.landing_target
        self.assertEqual(fields[1], 7)
        self.assertEqual(fields[2], main.mavutil.mavlink.MAV_FRAME_BODY_FRD)
        self.assertAlmostEqual(fields[5], math.sqrt(14), places=6)
        np.testing.assert_allclose(fields[8:11], [2.0, -1.0, 3.0])
        self.assertEqual(fields[13], 1)
        self.assertEqual(len(target.connection.mav.heartbeats), 1)


class QLandVideoLoggerTests(unittest.TestCase):
    def test_background_logger_writes_only_while_qland_is_active(self) -> None:
        written = threading.Event()
        released = threading.Event()
        writers = []

        class FakeWriter:
            def __init__(self, path, _codec, fps, size) -> None:
                self.path = Path(path)
                self.fps = fps
                self.size = size
                self.frames = []
                writers.append(self)

            def isOpened(self) -> bool:
                return True

            def write(self, frame: np.ndarray) -> None:
                self.frames.append(frame)
                written.set()

            def release(self) -> None:
                released.set()

        detection = main.Detection(
            marker_id=7,
            size_m=0.06,
            camera_xyz=np.array([1.0, 2.0, 3.0], dtype=np.float32),
            body_rpy=np.zeros(3, dtype=np.float32),
            body_quaternion=(1.0, 0.0, 0.0, 0.0),
            image_corners=np.array([[10, 10], [90, 10], [90, 90], [10, 90]], dtype=np.float32),
        )
        image = np.full((120, 160), 128, dtype=np.uint8)

        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            main.cv2, "VideoWriter", side_effect=FakeWriter
        ):
            logger = main.QLandVideoLogger(
                {
                    "qland_video_directory": "logs",
                    "camera_fps": 30,
                    "qland_video_fps": 30,
                    "qland_video_width": 80,
                },
                Path(directory),
            )
            logger.update(False, image, detection)
            self.assertFalse(written.wait(0.05))
            logger.update(True, image, detection)
            self.assertTrue(written.wait(2))
            logger.update(False)
            self.assertTrue(released.wait(2))
            logger.close()

        self.assertEqual(len(writers), 1)
        self.assertEqual(writers[0].path.parent.name, "logs")
        self.assertEqual(writers[0].path.suffix, ".mp4")
        self.assertEqual(writers[0].fps, 30)
        self.assertEqual(writers[0].size, (80, 60))
        self.assertEqual(writers[0].frames[0].shape, (60, 80, 3))

    def test_annotation_labels_a_lost_target(self) -> None:
        frame = main.QLandVideoLogger._annotate(
            np.zeros((48, 64), dtype=np.uint8),
            None,
            datetime.now().astimezone(),
            1.5,
            (64, 48),
        )

        self.assertEqual(frame.shape, (48, 64, 3))
        self.assertGreater(int(frame.max()), 0)


class PiFramesTests(unittest.TestCase):
    def test_yuv_capture_returns_only_latest_luma_plane(self) -> None:
        class FakeCamera:
            def __init__(self) -> None:
                self.stopped = False
                self.closed = False
                self.value = 0

            def create_video_configuration(self, **options):
                return options

            def configure(self, _setup) -> None:
                pass

            def start(self) -> None:
                pass

            def capture_array(self, _stream) -> np.ndarray:
                time.sleep(0.002)
                self.value += 1
                return np.full((6, 4), self.value, dtype=np.uint8)

            def stop(self) -> None:
                self.stopped = True

            def close(self) -> None:
                self.closed = True

        camera = FakeCamera()
        fake_module = types.SimpleNamespace(Picamera2=lambda: camera)
        config = {
            "camera_width": 4,
            "camera_height": 4,
            "camera_format": "YUV420",
            "camera_sensor_width": 8,
            "camera_sensor_height": 8,
            "camera_fps": 30,
        }
        source = main.PiFrames(config)
        stop = threading.Event()
        with mock.patch.dict(sys.modules, {"picamera2": fake_module}):
            generator = source.frames(stop)
            image = next(generator)
            generator.close()

        self.assertEqual(image.shape, (4, 4))
        self.assertTrue(camera.stopped)
        self.assertTrue(camera.closed)


if __name__ == "__main__":
    unittest.main()
