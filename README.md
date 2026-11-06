# ArUco precision landing

This project detects an ArUco landing pad and sends MAVLink `LANDING_TARGET`
messages to ArduPilot. The same [landing.py](landing.py) program runs against
either the included Simulink camera model or a Raspberry Pi Camera V2.

## Files

- `landing.py` — detection, pose estimation, camera input, and MAVLink output.
- `landing_config.json` — marker sizes, camera settings, and endpoints.
- `mavlinkMST_ArucoCamera.slx` plus the three `.m` files — SITL camera simulation.
- `landing-pad.png` — landing image rendered by the simulation.
- `precision-landing.parm` — ArduPilot precision-landing parameters.

Python requires NumPy, OpenCV with its ArUco module, and pymavlink.

## Simulink/SITL

Create a virtual environment and install the dependencies:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install numpy opencv-contrib-python pymavlink
```

Then:

1. Start the bridge with `python landing.py sim`. It waits for the other parts.
2. Start Copter SITL in Mission Planner.
3. Load `precision-landing.parm`, write the parameters, then restart SITL with
   **Wipe unchecked**.
4. Open `mavlinkMST_ArucoCamera.slx` in MATLAB and run it.
5. In Mission Planner, create a mission ending in `LAND` over the home position.

The defaults use TCP ports 5762 for Simulink/MAVLink, 5763 for the bridge, and
5600 for camera frames.

## Raspberry Pi

The tested hardware is a Pi Zero 2 W with Raspberry Pi OS Lite, Camera V2, and
a Pixhawk 6C connected to TELEM2. Install the packages manually so the board's
setup remains visible and easy to repair:

```bash
sudo apt update
sudo apt install python3-venv python3-numpy python3-opencv python3-picamera2
python3 -m venv --system-site-packages .venv
.venv/bin/pip install pymavlink
```

Copy only `landing.py` and `landing_config.json` to the Pi. First verify camera
detection without touching the flight controller:

```bash
.venv/bin/python landing.py pi --dry-run
```

After the marker is detected reliably and the Pixhawk has the values from
`precision-landing.parm`, run `.venv/bin/python landing.py pi`. Nothing else,
including MAVProxy, may hold `/dev/serial0` at the same time. Stop with
`Ctrl+C`.

Before any flight, bench-test with propellers removed. The program refuses to
send targets unless the required `PLND_` and `SERIAL2_PROTOCOL` values can be
read back from ArduPilot.

## Configuration

`landing_config.json` has separate `sim` and `pi` sections because the physical
markers and cameras differ. Each marker's `size_m` is the measured black-border
side length. The detector accepts only listed IDs and uses the largest visible
known marker. `camera_hfov_deg` is a pinhole approximation; pass a calibration
file with `--calibration camera.json` for a calibrated real camera. Use
`--mavlink ENDPOINT` only when temporarily overriding the configured link.
