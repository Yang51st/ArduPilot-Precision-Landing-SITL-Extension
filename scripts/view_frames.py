#!/usr/bin/env python3
"""Show an APSM RGB frame stream in a window on the laptop.

Works against the Raspberry Pi bridge started with --publish, and against
Simulink's ArucoFrameServer. Press q or Esc in the window to quit.

    python scripts\\view_frames.py --endpoint watarrow-pi:5600
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))

from precland_core import FrameDisconnected, FrameUnavailable, TcpFrameSource, parse_host_port  # noqa: E402


def endpoint_argument(value: str) -> tuple[str, int]:
    try:
        return parse_host_port(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--endpoint", type=endpoint_argument, default=("127.0.0.1", 5600), metavar="HOST:PORT")
    parser.add_argument("--title", default="precland frames")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    host, port = args.endpoint
    source = TcpFrameSource(host, port, name="frame publisher", hint="start the Pi bridge with --publish, or the Simulink model")
    stop_event = threading.Event()

    def stop(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    cv2.namedWindow(args.title, cv2.WINDOW_AUTOSIZE)
    received = 0
    window_start = time.monotonic()
    fps_text = "fps: --"
    while not stop_event.is_set():
        try:
            for frame_number, image in source.frames(stop_event):
                received += 1
                now = time.monotonic()
                if now - window_start >= 1.0:
                    fps_text = f"fps: {received / (now - window_start):.1f}"
                    received = 0
                    window_start = now
                display = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.putText(
                    display,
                    f"{fps_text}  frame {frame_number}  {image.shape[1]}x{image.shape[0]}",
                    (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow(args.title, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    stop_event.set()
                    break
        except FrameUnavailable:
            if stop_event.is_set():
                break
            # Keep the window responsive while waiting for the server.
            if (cv2.waitKey(1000) & 0xFF) in (27, ord("q")):
                break
        except FrameDisconnected as error:
            if stop_event.is_set():
                break
            print(f"Viewer: {error}; retrying in 1 s", file=sys.stderr, flush=True)
            if (cv2.waitKey(1000) & 0xFF) in (27, ord("q")):
                break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
