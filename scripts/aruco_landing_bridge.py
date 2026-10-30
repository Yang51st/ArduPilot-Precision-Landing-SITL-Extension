#!/usr/bin/env python3
"""Laptop bridge: Simulink camera frames in, MAVLink LANDING_TARGET to SITL out.

Reads the RGB stream published by mavlinkMST_ArucoCamera.slx, detects the
ArUco landing marker and sends LANDING_TARGET messages to the SITL vehicle
started from Mission Planner. The detection and MAVLink code lives in
precland_core.py, which is shared with the Raspberry Pi bridge.

Point --frame-endpoint at a Raspberry Pi running pi_landing_bridge.py
--publish to run the same detection on real camera frames from the laptop.
"""

from __future__ import annotations

import argparse
import signal
import threading
from pathlib import Path

from precland_core import (
    TcpFrameSource,
    build_detector,
    build_mavlink,
    load_config,
    parse_host_port,
    run_bridge,
)


def endpoint_argument(value: str) -> tuple[str, int]:
    try:
        return parse_host_port(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, required=True, help="shared JSON camera/marker configuration")
    parser.add_argument("--frame-endpoint", type=endpoint_argument, help="override frame server HOST:PORT")
    parser.add_argument("--mavlink", help="override pymavlink endpoint, e.g. tcp:127.0.0.1:5763")
    parser.add_argument("--calibration", type=Path, help="camera calibration JSON (real cameras only)")
    parser.add_argument("--dry-run", action="store_true", help="detect but do not send MAVLink")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    config = load_config(args.config)
    frame_host, frame_port = args.frame_endpoint or (config["frame_host"], int(config["frame_port"]))

    detector = build_detector(config, args.calibration)
    frame_source = TcpFrameSource(
        frame_host,
        frame_port,
        name="Simulink",
        hint="start the mavlinkMST_ArucoCamera model",
    )
    mavlink = build_mavlink(config, args.mavlink, args.dry_run)

    stop_event = threading.Event()

    def stop(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    return run_bridge(config, detector, frame_source, mavlink, stop_event)


if __name__ == "__main__":
    raise SystemExit(main())
