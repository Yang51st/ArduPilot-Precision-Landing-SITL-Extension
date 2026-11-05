#!/usr/bin/env python3
"""Laptop regression test for precland_core.py. No MATLAB, no network, no Pi.

Renders the configured ArUco markers into synthetic camera frames at known
pixel sizes, runs the detector, and checks the recovered pose, the physical
size looked up from each marker's ID, and the choice made when two markers
are visible at once. Also round-trips the frame stream header.

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
    build_mavlink,
    load_config,
    pack_frame_header,
    parse_markers,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
LARGE_PIXELS = 120
SMALL_PIXELS = 60


def blank_frame() -> np.ndarray:
    return np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype=np.uint8)


def paste_marker(frame: np.ndarray, dictionary, marker_id: int, pixels: int, centre_x: int, centre_y: int) -> None:
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    top = centre_y - pixels // 2
    left = centre_x - pixels // 2
    frame[top : top + pixels, left : left + pixels] = marker


def to_rgb(frame: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)


def check(condition: bool, message: str) -> None:
    status = "ok  " if condition else "FAIL"
    print(f"  {status} {message}")
    if not condition:
        raise SystemExit(1)


def main() -> int:
    config = load_config(REPO_ROOT / "aruco_landing_config.json")
    sizes = parse_markers(config)
    check(len(sizes) >= 2, f"config lists at least two markers (found {len(sizes)})")
    first_id, second_id = sorted(sizes)[:2]
    detector = build_detector(config)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["marker_family"]))

    focal_px = (FRAME_WIDTH / 2.0) / math.tan(math.radians(float(config["camera_hfov_deg"])) / 2.0)
    centre = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
    left = (FRAME_WIDTH // 4, FRAME_HEIGHT // 2)
    right = (3 * FRAME_WIDTH // 4, FRAME_HEIGHT // 2)

    detection = None
    for marker_id in (first_id, second_id):
        print(f"Marker ID {marker_id} alone, centred, {LARGE_PIXELS} px wide:")
        frame = blank_frame()
        paste_marker(frame, dictionary, marker_id, LARGE_PIXELS, *centre)
        detection = detector.detect(to_rgb(frame))
        check(detection is not None, "marker detected")
        assert detection is not None
        check(detection.marker_id == marker_id, f"reported as id {marker_id}")
        check(
            detection.marker_size_m == sizes[marker_id],
            f"size taken from the config for that id: {sizes[marker_id]:.3f} m",
        )
        expected_z = focal_px * sizes[marker_id] / LARGE_PIXELS
        z = float(detection.camera_xyz_m[2])
        check(math.isclose(z, expected_z, rel_tol=0.05), f"camera z {z:.3f} m within 5% of {expected_z:.3f} m")
        lateral = float(np.hypot(detection.camera_xyz_m[0], detection.camera_xyz_m[1]))
        check(lateral < 0.01 * z, f"marker centred: lateral offset {lateral * 100:.1f} cm")

    print("Two known markers visible: the one covering more pixels is used:")
    frame = blank_frame()
    paste_marker(frame, dictionary, first_id, LARGE_PIXELS, *left)
    paste_marker(frame, dictionary, second_id, SMALL_PIXELS, *right)
    detection = detector.detect(to_rgb(frame))
    check(detection is not None and detection.marker_id == first_id, f"id {first_id} chosen when it is larger in the image")
    frame = blank_frame()
    paste_marker(frame, dictionary, first_id, SMALL_PIXELS, *left)
    paste_marker(frame, dictionary, second_id, LARGE_PIXELS, *right)
    detection = detector.detect(to_rgb(frame))
    check(detection is not None and detection.marker_id == second_id, f"id {second_id} chosen when it is larger in the image")
    assert detection is not None
    check(detection.marker_size_m == sizes[second_id], "and its own size is used for the pose")
    check(float(detection.camera_xyz_m[0]) > 0, "marker on the right of the image gives positive camera x")

    print("Marker with an ID that is not in the config:")
    dictionary_size = int(dictionary.bytesList.shape[0])
    unknown_id = next(i for i in range(dictionary_size) if i not in sizes)
    frame = blank_frame()
    paste_marker(frame, dictionary, unknown_id, LARGE_PIXELS, *centre)
    detector.other_ids_seen.clear()
    check(detector.detect(to_rgb(frame)) is None, f"id {unknown_id} is ignored")
    check(unknown_id in detector.other_ids_seen, "but remembered for the status line")

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

    print("MAVLink endpoint and baud from the config:")
    check(build_mavlink(config, None, True).baud is None, "no mavlink_baud in the SITL config leaves the default")
    pi_config = load_config(REPO_ROOT / "pi" / "pi_landing_config.json")
    pi_mavlink = build_mavlink(pi_config, None, True)
    check(
        pi_mavlink.endpoint == pi_config["mavlink_endpoint"] and pi_mavlink.baud == pi_config["mavlink_baud"],
        f"Pi config gives {pi_mavlink.endpoint} at {pi_mavlink.baud} baud",
    )
    try:
        build_mavlink({"mavlink_endpoint": "/dev/serial0", "mavlink_baud": 921}, None, True)
        shorthand_rejected = False
    except ValueError:
        shorthand_rejected = True
    check(shorthand_rejected, "ArduPilot shorthand 921 is rejected as a baud rate")

    print("All checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
