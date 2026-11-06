#!/usr/bin/env python3
"""Start a disposable SITL instance and verify its MAVLink heartbeat."""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent
SITL_BINARY = ROOT_DIR / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
DEFAULT_PARAMS = ROOT_DIR / "ardupilot" / "Tools" / "autotest" / "default_params" / "copter.parm"
VERIFY_PORT = 14551


def heartbeat_in(data: bytes):
    """Return (MAVLink version, system id, component id) when found."""
    for index, magic in enumerate(data):
        if magic == 0xFD and len(data) >= index + 10:
            message_id = int.from_bytes(data[index + 7 : index + 10], "little")
            if message_id == 0:
                return 2, data[index + 5], data[index + 6]
        if magic == 0xFE and len(data) >= index + 6:
            if data[index + 5] == 0:
                return 1, data[index + 3], data[index + 4]
    return None


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=3)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        process.kill()
        process.wait(timeout=3)


def main() -> int:
    if not SITL_BINARY.is_file():
        print(f"SITL is not built at {SITL_BINARY}.", file=sys.stderr)
        return 1

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        listener.bind(("127.0.0.1", VERIFY_PORT))
    except OSError as error:
        print(f"Cannot listen on UDP {VERIFY_PORT}: {error}", file=sys.stderr)
        return 1
    listener.settimeout(0.5)

    with tempfile.TemporaryDirectory(prefix="apsm-sitl-") as state_dir:
        command = [
            str(SITL_BINARY),
            "-w",
            "--model", "+",
            "--speedup", "1",
            "--slave", "0",
            "--home", "43.6532,-79.3832,76,0",
            f"--serial0=udpclient:127.0.0.1:{VERIFY_PORT}",
            "--defaults", str(DEFAULT_PARAMS),
            "--sim-address=127.0.0.1",
            "--instance", "3",
        ]
        process = subprocess.Popen(
            command,
            cwd=state_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

        deadline = time.monotonic() + 15
        packet_count = 0
        heartbeat = None
        try:
            while time.monotonic() < deadline and process.poll() is None:
                try:
                    data, _address = listener.recvfrom(65535)
                except socket.timeout:
                    continue
                packet_count += 1
                heartbeat = heartbeat_in(data)
                if heartbeat:
                    break
        finally:
            listener.close()
            stop_process(process)

        output = process.stdout.read() if process.stdout else ""
        if heartbeat:
            mavlink_version, system_id, component_id = heartbeat
            print(
                "PASS: ArduCopter SITL emitted a MAVLink "
                f"v{mavlink_version} heartbeat on UDP {VERIFY_PORT} "
                f"(system {system_id}, component {component_id}; "
                f"{packet_count} datagram(s) observed)."
            )
            return 0

        print("FAIL: no MAVLink heartbeat arrived within 15 seconds.", file=sys.stderr)
        if output:
            print("Simulator output:", file=sys.stderr)
            print("\n".join(output.splitlines()[-20:]), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
