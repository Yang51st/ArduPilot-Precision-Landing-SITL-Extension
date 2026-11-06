# ArUco precision landing

This project detects an ArUco landing pad and sends MAVLink `LANDING_TARGET`
messages to ArduPilot. The same [main.py](main.py) program uses either the
Simulink camera model or a Raspberry Pi Camera V2.

## Layout

- `main.py` — detection, pose estimation, camera input, and MAVLink output.
- `landing_config.json` — shared marker, camera, and connection settings.
- `precision-landing.parm` — ArduPilot precision-landing parameters.
- `simulation/` — the Simulink model, its MATLAB classes, landing-pad image,
  and the optional macOS SITL launcher.
- `tests/` — SITL and closed-loop landing verification helpers.

Python requires NumPy, OpenCV with its ArUco module, and pymavlink.

## Python setup

On Windows:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install numpy opencv-contrib-python pymavlink
```

On macOS or Linux, use `.venv/bin/python` after creating the environment with
`python3 -m venv .venv`.

## Simulink/SITL

The portable workflow is:

1. Start the bridge with `python main.py sim`. It waits for the other parts.
2. Start Copter SITL in Mission Planner.
3. Load `precision-landing.parm`, write the parameters, then restart SITL with
   **Wipe unchecked**.
4. Open `simulation/mavlinkMST_ArucoCamera.slx` in MATLAB and run it.
5. In Mission Planner, create a mission ending in `LAND` over the home position.

On macOS, `./simulation/start-sitl.command` can instead launch a locally built
`ardupilot/` checkout, QGroundControl, and the bridge. Run it first, then start
the Simulink model. Use `--help` to see its options.

The defaults use TCP ports 5762 for Simulink/MAVLink, 5763 for the bridge, and
5600 for camera frames.

## Raspberry Pi

The tested hardware is a Pi Zero 2 W with Raspberry Pi OS Lite, Camera V2, and
a Pixhawk 6C connected to TELEM2. Install the dependencies manually:

```bash
sudo apt update
sudo apt install python3-venv python3-numpy python3-opencv python3-picamera2
python3 -m venv --system-site-packages .venv
.venv/bin/pip install pymavlink
```

Copy only `main.py` and `landing_config.json` to the Pi. First test camera
detection without sending MAVLink:

```bash
.venv/bin/python main.py pi --dry-run
```

After detection is reliable and the Pixhawk has the values from
`precision-landing.parm`, run `.venv/bin/python main.py pi`. Nothing else,
including MAVProxy, may hold `/dev/serial0` at the same time. Stop with
`Ctrl+C` and bench-test with propellers removed before any flight.

## Configuration and tests

`landing_config.json` has separate `sim` and `pi` sections. Each marker's
`size_m` is the measured black-border side length. The detector accepts only
listed IDs and uses the largest visible marker with a unique ID. A frame is
rejected if the same configured ID appears more than once, because one ID
cannot safely represent differently sized markers. The top edge of the printed
marker defines its forward direction. `landing_yaw_offset_deg` sets the desired
vehicle heading relative to that direction: zero aligns the vehicle forward,
and positive angles are clockwise when viewed from above. Yaw commands are sent
only while armed in one of `yaw_align_modes` (AUTO, LAND, and RTL by default).

The detector recovers roll, pitch, and yaw. All three axes are placed in the
`LANDING_TARGET` quaternion, while yaw is also enforced with
`MAV_CMD_CONDITION_YAW` because ArduPilot precision landing does not consume the
quaternion itself. For a calibrated camera, pass `--calibration camera.json`.

The optional verification helpers are run with:

```bash
.venv/bin/python tests/verify_sitl.py
.venv/bin/python tests/verify_precision_landing.py
```
