#!/usr/bin/env python3
"""Laptop regression test for precland_core.py. No MATLAB, no network, no Pi.

Renders the configured ArUco marker into a synthetic camera frame at a known
pixel size, runs the detector, and checks the recovered pose and the
two-scale resolution logic. Also round-trips the frame stream header.

Run from the repository root with the laptop venv active:

    python scripts\\test_core.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from precland_core import (  # noqa: E402
    FRAME_HEADER,
    FRAME_MAGIC,
    build_detector,
    load_config,
    pack_frame_header,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
MARKER_PIXELS = 120


def synthetic_frame(config: dict) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["marker_family"]))
    marker = cv2.aruco.generateImageMarker(dictionary, int(config["marker_id"]), MARKER_PIXELS)
    frame = np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype=np.uint8)
    top = (FRAME_HEIGHT - MARKER_PIXELS) // 2
    left = (FRAME_WIDTH - MARKER_PIXELS) // 2
    frame[top : top + MARKER_PIXELS, left : left + MARKER_PIXELS] = marker
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)


def check(condition: bool, message: str) -> None:
    status = "ok  " if condition else "FAIL"
    print(f"  {status} {message}")
    if not condition:
        raise SystemExit(1)


def main() -> int:
    config = load_config(REPO_ROOT / "aruco_landing_config.json")
    detector = build_detector(config)
    frame = synthetic_frame(config)

    focal_px = (FRAME_WIDTH / 2.0) / math.tan(math.radians(float(config["camera_hfov_deg"])) / 2.0)
    embedded_m = float(config["marker_size_m"])
    outer_m = float(config["outer_marker_size_m"])
    expected_z_embedded = focal_px * embedded_m / MARKER_PIXELS
    expected_z_outer = focal_px * outer_m / MARKER_PIXELS

    print("Detection without vehicle height:")
    detection = detector.detect(frame, None)
    check(detection is not None, "marker detected")
    assert detection is not None
    check(not detection.scale_resolved, "scale is reported as unresolved")
    check(detection.marker_size_m == embedded_m, "unresolved pose uses the embedded marker size")
    z = float(detection.camera_xyz_m[2])
    check(
        math.isclose(z, expected_z_embedded, rel_tol=0.05),
        f"camera z {z:.3f} m within 5% of {expected_z_embedded:.3f} m",
    )
    lateral = float(np.hypot(detection.camera_xyz_m[0], detection.camera_xyz_m[1]))
    check(lateral < 0.02, f"marker centred: lateral offset {lateral * 100:.1f} cm")

    print("Detection with vehicle height near the embedded-scale estimate:")
    detection = detector.detect(frame, expected_z_embedded)
    check(detection is not None and detection.scale_resolved, "scale resolved")
    assert detection is not None
    check(detection.marker_size_m == embedded_m, "embedded scale chosen")

    print("Detection with vehicle height near the outer-scale estimate:")
    detection = detector.detect(frame, expected_z_outer)
    check(detection is not None and detection.scale_resolved, "scale resolved")
    assert detection is not None
    check(detection.marker_size_m == outer_m, "outer scale chosen")
    z = float(detection.camera_xyz_m[2])
    check(
        math.isclose(z, expected_z_outer, rel_tol=0.05),
        f"camera z {z:.3f} m within 5% of {expected_z_outer:.3f} m",
    )

    print("Body frame conversion:")
    body = detection.body_frd_m
    cam = detection.camera_xyz_m
    check(
        math.isclose(float(body[0]), -float(cam[1]), abs_tol=1e-6)
        and math.isclose(float(body[1]), float(cam[0]), abs_tol=1e-6)
        and math.isclose(float(body[2]), float(cam[2]), abs_tol=1e-6),
        "camera xyz maps to body FRD as (-y, x, z)",
    )

    print("Frame stream header:")
    header = pack_frame_header(FRAME_WIDTH, FRAME_HEIGHT, 3, 7, FRAME_WIDTH * FRAME_HEIGHT * 3)
    check(len(header) == 24, "header is 24 bytes")
    magic, width, height, channels, number, size = FRAME_HEADER.unpack(header)
    check(
        magic == FRAME_MAGIC
        and (width, height, channels, number, size) == (FRAME_WIDTH, FRAME_HEIGHT, 3, 7, FRAME_WIDTH * FRAME_HEIGHT * 3),
        "header round-trips",
    )

    print("All checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
