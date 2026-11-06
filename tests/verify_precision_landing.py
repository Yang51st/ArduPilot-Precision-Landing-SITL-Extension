#!/usr/bin/env python3
"""Assert closed-loop SITL landing position and marker-relative yaw accuracy."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

ROOT_DIR = Path(__file__).resolve().parent.parent


class FlightVerifier:
    def __init__(self, endpoint: str) -> None:
        self.connection = mavutil.mavlink_connection(
            endpoint,
            source_system=250,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.vehicle_system = 1
        self.target_found = False
        self.last_position = None
        self.last_yaw = None

    def connect(self) -> None:
        print("Verifier: waiting for SITL heartbeat", flush=True)
        heartbeat = self.connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise RuntimeError("no SITL heartbeat")
        self.vehicle_system = self.connection.target_system
        message_ids = (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        )
        for message_id in message_ids:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                100000,
                0,
                0,
                0,
                0,
                0,
            )
        print(f"Verifier: connected to SITL system {self.vehicle_system}", flush=True)

    def receive(self, timeout: float = 1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT", "HEARTBEAT"],
                blocking=True,
                timeout=min(0.2, max(0.0, deadline - time.monotonic())),
            )
            if message is None or message.get_srcSystem() != self.vehicle_system:
                continue
            message_type = message.get_type()
            if message_type == "STATUSTEXT":
                text = str(message.text).rstrip("\x00")
                print(f"SITL: {text}", flush=True)
                self.target_found |= "PrecLand: Target Found" in text
            elif message_type == "LOCAL_POSITION_NED":
                self.last_position = message
                return message
            elif message_type == "ATTITUDE":
                self.last_yaw = float(message.yaw)
            return message
        return None

    def wait_mode(self, mode_name: str, timeout: float = 10.0) -> None:
        wanted_mode = self.connection.mode_mapping()[mode_name]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.receive(1.0)
            if message and message.get_type() == "HEARTBEAT" and message.custom_mode == wanted_mode:
                return
        raise RuntimeError(f"mode {mode_name} was not reached")

    def set_mode(self, mode_name: str) -> None:
        self.connection.set_mode_apm(mode_name)
        self.wait_mode(mode_name)

    def arm(self) -> None:
        for _attempt in range(8):
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                message = self.receive(0.25)
                if (
                    message
                    and message.get_type() == "HEARTBEAT"
                    and message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ):
                    return
        raise RuntimeError("vehicle failed to arm")

    def takeoff(self, altitude_m: float) -> None:
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            position = self.receive(1.0)
            if position and position.get_type() == "LOCAL_POSITION_NED" and position.z < -(altitude_m - 0.4):
                return
        raise RuntimeError("takeoff altitude was not reached")

    def move_to(self, north_m: float, east_m: float, altitude_m: float) -> None:
        position_only_mask = (
            (1 << 3)
            | (1 << 4)
            | (1 << 5)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
            | (1 << 10)
            | (1 << 11)
        )
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            self.connection.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000),
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                position_only_mask,
                north_m,
                east_m,
                -altitude_m,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            position = self.receive(0.3)
            if position and position.get_type() == "LOCAL_POSITION_NED":
                horizontal_error = math.hypot(position.x - north_m, position.y - east_m)
                if horizontal_error < 0.15 and abs(position.z + altitude_m) < 0.25:
                    return
        raise RuntimeError("offset position was not reached")

    def wait_landed(self, timeout: float = 50.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.receive(0.5)
            if (
                message
                and message.get_type() == "HEARTBEAT"
                and not message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ):
                return
        raise RuntimeError("vehicle did not land and disarm")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--north", type=float, default=1.0)
    parser.add_argument("--east", type=float, default=0.7)
    parser.add_argument("--max-error", type=float, default=0.30)
    parser.add_argument("--expected-yaw", type=float, help="expected final heading in degrees")
    parser.add_argument("--max-yaw-error", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = arguments()
    verifier = FlightVerifier(args.endpoint)
    verifier.connect()
    try:
        print("Verifier: entering GUIDED and arming", flush=True)
        verifier.set_mode("GUIDED")
        verifier.arm()
        print(f"Verifier: taking off to {args.altitude:.1f} m", flush=True)
        verifier.takeoff(args.altitude)
        print(f"Verifier: moving to N/E offset [{args.north:.1f}, {args.east:.1f}] m", flush=True)
        verifier.move_to(args.north, args.east, args.altitude)
        print("Verifier: entering LAND", flush=True)
        verifier.set_mode("LAND")
        verifier.wait_landed()
    except Exception:
        print("Verifier: test failed; commanding LAND", flush=True)
        verifier.connection.set_mode_apm("LAND")
        raise

    if verifier.last_position is None:
        raise RuntimeError("no final local position was received")
    if verifier.last_yaw is None:
        raise RuntimeError("no final attitude was received")
    if args.expected_yaw is None:
        config = json.loads((ROOT_DIR / "landing_config.json").read_text(encoding="utf-8"))["sim"]
        args.expected_yaw = config["marker_yaw_deg"] + config["landing_yaw_offset_deg"]
    horizontal_error = math.hypot(verifier.last_position.x, verifier.last_position.y)
    yaw_deg = math.degrees(verifier.last_yaw) % 360
    yaw_error = abs((yaw_deg - args.expected_yaw + 180) % 360 - 180)
    print(
        "PRECISION_LANDING_RESULT "
        f"final_ned=[{verifier.last_position.x:.3f}, {verifier.last_position.y:.3f}, "
        f"{verifier.last_position.z:.3f}] horizontal_error={horizontal_error:.3f}m "
        f"yaw={yaw_deg:.1f}deg yaw_error={yaw_error:.1f}deg target_found={verifier.target_found}",
        flush=True,
    )
    if not verifier.target_found:
        raise RuntimeError("ArduPilot never reported precision-landing target acquisition")
    if horizontal_error > args.max_error:
        raise RuntimeError(f"landing error {horizontal_error:.3f} m exceeds {args.max_error:.3f} m")
    if yaw_error > args.max_yaw_error:
        raise RuntimeError(f"yaw error {yaw_error:.1f} deg exceeds {args.max_yaw_error:.1f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
