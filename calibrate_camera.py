#!/usr/bin/env python3
"""Capture checkerboard pictures and calibrate a camera."""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def find_corners(gray, size, fast=False):
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ADAPTIVE_THRESH
    if fast:
        flags |= cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, size, flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def camera_frames(width, height, camera_index):
    """Use Picamera2 when available, otherwise a normal OpenCV camera."""
    try:
        from picamera2 import Picamera2
    except ImportError:
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            raise RuntimeError(f"cannot open camera {camera_index}")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        print(f"Using OpenCV camera {camera_index}", file=sys.stderr)
        try:
            while True:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("camera stopped returning frames")
                yield cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        finally:
            camera.release()
        return

    camera = Picamera2()
    # Picamera2's BGR888 format returns RGB byte order.
    config = camera.create_still_configuration(main={"size": (width, height), "format": "BGR888"})
    camera.configure(config)
    camera.start()
    print("Using Picamera2", file=sys.stderr)
    try:
        while True:
            yield cv2.cvtColor(camera.capture_array("main"), cv2.COLOR_RGB2GRAY)
    finally:
        camera.stop()
        camera.close()


def capture(args):
    args.images.mkdir(parents=True, exist_ok=True)
    first_number = len(list(args.images.glob("calibration_*.png"))) + 1
    saved, last_save, previous = 0, 0.0, []
    print("Move the checkerboard around. Press Ctrl+C to stop early.", file=sys.stderr)
    try:
        for gray in camera_frames(args.width, args.height, args.camera):
            corners = find_corners(gray, (args.columns, args.rows), fast=True)
            if corners is not None:
                points = corners.reshape(-1, 2)
                scale = np.array([gray.shape[1], gray.shape[0]])
                pose = points[[0, args.columns - 1, -args.columns, -1]].ravel() / np.tile(scale, 4)
                different = not previous or min(np.linalg.norm(pose - old) for old in previous) > 0.08
                if different and time.monotonic() - last_save > 1.0:
                    path = args.images / f"calibration_{first_number + saved:02d}.png"
                    if not cv2.imwrite(str(path), gray):
                        raise RuntimeError(f"could not write {path}")
                    saved += 1
                    last_save = time.monotonic()
                    previous.append(pose)
                    print(f"Saved {saved}/{args.count}: {path}", file=sys.stderr)

            if args.stream:
                preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                if corners is not None:
                    cv2.drawChessboardCorners(
                        preview, (args.columns, args.rows), corners, True
                    )
                cv2.putText(
                    preview,
                    f"saved {saved}/{args.count}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                ok, jpeg = cv2.imencode(".jpg", preview)
                if not ok:
                    raise RuntimeError("could not encode stream frame")
                sys.stdout.buffer.write(jpeg)
                sys.stdout.buffer.flush()

            if saved == args.count:
                break
    except KeyboardInterrupt:
        print("\nCapture stopped", file=sys.stderr)
    calibrate(args)


def calibrate(args):
    if not args.images.is_dir():
        raise RuntimeError(f"image directory does not exist: {args.images}")
    suffixes = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}
    paths = sorted(p for p in args.images.iterdir() if p.suffix.lower() in suffixes)
    board = np.zeros((args.columns * args.rows, 3), np.float32)
    board[:, :2] = np.mgrid[0 : args.columns, 0 : args.rows].T.reshape(-1, 2)
    object_points, image_points, image_size = [], [], None

    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and size != image_size:
            print(f"Skipping {path}: expected {image_size}, got {size}", file=sys.stderr)
            continue
        corners = find_corners(gray, (args.columns, args.rows))
        if corners is not None:
            image_size = size
            object_points.append(board)
            image_points.append(corners)
            print(f"Using {path}", file=sys.stderr)

    if len(image_points) < 5:
        raise RuntimeError(f"checkerboard found in only {len(image_points)} images; need at least 5")
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    result = {
        "camera_matrix": matrix.tolist(),
        "distortion": distortion.ravel().tolist(),
        "image_size": list(image_size),
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Calibration RMS error: {rms:.3f} pixels", file=sys.stderr)
    print(f"Wrote {args.output}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "calibrate"))
    parser.add_argument("--images", type=Path, default=Path("calibration_images"))
    parser.add_argument("--output", type=Path, default=Path("camera.json"))
    parser.add_argument("--columns", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--stream", action="store_true", help="write annotated MJPEG to stdout")
    args = parser.parse_args()
    if args.stream and args.mode != "capture":
        parser.error("--stream requires capture mode")
    try:
        capture(args) if args.mode == "capture" else calibrate(args)
    except (OSError, RuntimeError, cv2.error) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
