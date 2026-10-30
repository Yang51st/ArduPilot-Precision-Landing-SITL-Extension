#!/usr/bin/env bash
# One-time setup of the precision-landing bridge on a Raspberry Pi.
#
# Usage, on the Pi, from ~/precland:
#     ./setup_pi.sh          install packages, create the venv, then check
#     ./setup_pi.sh check    only run the checks (used after every deploy)
#
# OpenCV, numpy and Picamera2 come from apt, not pip: the Zero 2W has 512 MB
# of RAM and no spare CPU for building wheels, Raspberry Pi OS Trixie ships
# OpenCV 4.10 which already includes the aruco module, and Picamera2 is only
# distributed as an apt package. The venv is created with
# --system-site-packages so those apt packages are visible inside it; pip is
# used only for pymavlink.
#
# The Pi needs internet access for the install step. Over the USB gadget link
# it has none by default; see README.md for the Wi-Fi route.

set -euo pipefail

cd "$(dirname "$0")"
VENV=".venv"

check() {
    echo "== Python packages in the venv =="
    if [ ! -x "$VENV/bin/python" ]; then
        echo "venv missing: run ./setup_pi.sh (without arguments) first"
        return 1
    fi
    "$VENV/bin/python" - <<'PY'
import sys
print("python", sys.version.split()[0])
import numpy
print("numpy", numpy.__version__)
import cv2
print("opencv", cv2.__version__, "aruco:", hasattr(cv2, "aruco"))
import pymavlink
print("pymavlink", pymavlink.__version__)
from picamera2 import Picamera2
print("picamera2 importable")
PY
    echo "== Cameras seen by Picamera2 (what the bridge uses) =="
    "$VENV/bin/python" - <<'PY' 2>&1 | grep -vE "^\[[0-9]+:[0-9]+:[0-9]+\.[0-9]+\]"
from picamera2 import Picamera2
cameras = Picamera2.global_camera_info()
if not cameras:
    print("no camera found (check the ribbon cable orientation and that the camera is enabled)")
for camera in cameras:
    print(camera.get("Num"), camera.get("Model"), camera.get("Id"))
PY
    echo "== Cameras seen by rpicam-hello (diagnostic only) =="
    if command -v rpicam-hello >/dev/null 2>&1; then
        rpicam-hello --list-cameras 2>&1 || echo "rpicam-hello failed; if it reports a symbol lookup error, rerun ./setup_pi.sh so rpicam-apps-lite matches libcamera"
    else
        echo "rpicam-hello is not installed"
    fi
    echo "== Memory and disk =="
    free -m | head -2
    df -h / | tail -1
}

if [ "${1:-}" = "check" ]; then
    check
    exit 0
fi

echo "== Checking internet access =="
if ! ping -c 1 -W 3 deb.debian.org >/dev/null 2>&1; then
    echo "The Pi cannot reach deb.debian.org. Connect Wi-Fi first (see README.md), then rerun."
    exit 1
fi

echo "== Installing apt packages (asks for sudo) =="
# --no-install-recommends keeps Qt, GTK and other desktop extras off a
# headless board; the first attempt without it tried to pull in ~2600 lines
# of packages including pyqt5 and qtwayland.
sudo apt-get update
# rpicam-apps-lite is listed so it is upgraded together with libcamera;
# otherwise the preinstalled rpicam-hello breaks with a symbol lookup error
# after python3-picamera2 pulls in a newer libcamera.
sudo apt-get install -y --no-install-recommends \
    python3-venv python3-opencv python3-numpy python3-picamera2 rpicam-apps-lite

echo "== Creating the venv with access to the apt packages =="
if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install pymavlink

check
echo
echo "Setup done. Bench test with:"
echo "  $VENV/bin/python pi_landing_bridge.py --config pi_landing_config.json --dry-run --publish 0.0.0.0:5600"
