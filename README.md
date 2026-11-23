# ArUco precision landing

This project detects an ArUco landing pad and sends MAVLink `LANDING_TARGET`
messages to ArduPilot. The same [main.py](main.py) program uses either the
Simulink camera model or a Raspberry Pi Camera V2.

## Layout

- `main.py` — detection, pose estimation, camera input, and MAVLink output.
- `calibrate_camera.py` — checkerboard image capture and camera calibration.
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

The Pi defaults use a 960x720 YUV420 stream at 30 fps from the Camera V2's
1640x1232 sensor mode. Detection reads the Y (grayscale) plane directly, while
a one-frame capture queue overlaps camera readout with processing and drops
stale frames. The once-per-second `Vision` line reports the actual processed
rate, mean detection time, and number of stale frames dropped. A few drops are
expected and preferable to sending old target positions. If the Zero 2 W
cannot sustain at least 10-20 processed frames per second, reduce only
`camera_width` and `camera_height` to 800x600 or 640x480; keep the 4:3 aspect
ratio.

### QLAND video logs

When the armed aircraft reports `QLAND` mode, the bridge creates a timestamped
MP4 in `flight_logs/`. Recording stops as soon as the aircraft leaves `QLAND`
or disarms; no video is written during ordinary flight, while idle, or in dry
run mode. Each frame shows the selected marker outline and ID, body-FRD target
position, target attitude, and a prominent `TARGET LOST` message when decoding
fails.

Video annotation and encoding run in a background thread with a single pending
frame. If the encoder falls behind it drops video frames instead of delaying
target detection. The Pi logger targets the camera's full 30 fps but scales the
960x720 camera image to 320x240 to minimize resize and encode load. The actual
recorded rate cannot exceed the rate at which detection processes camera frames.
These settings are controlled by
`qland_video_logging`, `qland_video_directory`, `qland_video_fps`,
`qland_video_width`, and `qland_video_codec` in `landing_config.json`.

## Configuration and tests

`landing_config.json` has separate `sim` and `pi` sections. Each marker's
`size_m` is the measured black-border side length. For a nested target, list
every marker ID and size in `markers`; the detector uses the smallest physical
marker visible in the current frame. If two visible configured markers have the
same physical size, it uses the one with the larger image area, then lower ID.
A frame is rejected if the same configured ID appears more than once, because
one ID cannot safely represent differently sized markers. For example:

```json
"markers": [
  {"id": 0, "size_m": 0.18},
  {"id": 1, "size_m": 0.09},
  {"id": 2, "size_m": 0.045}
]
```

Each ArUco pattern must remain independently decodable; placing one marker over
the encoded cells of another will corrupt the larger marker. The top edge of
each marker defines its forward direction, so all nested markers must have the
same physical orientation and center. `landing_yaw_offset_deg` sets the desired
vehicle heading relative to that direction: zero aligns the vehicle forward,
and positive angles are clockwise when viewed from above. Yaw commands are sent
only while armed in one of `yaw_align_modes` (AUTO, LAND, and RTL by default).

The detector recovers roll, pitch, and yaw. All three axes are placed in the
`LANDING_TARGET` quaternion, while yaw is also enforced with
`MAV_CMD_CONDITION_YAW` because ArduPilot precision landing does not consume the
quaternion itself. For a calibrated camera, pass `--calibration camera.json`.
A calibration is automatically scaled when the configured resolution changes
but retains the same aspect ratio. Recalibrating at the flight resolution is
still recommended for the best metric distance estimate.

Only a current detection produces a `LANDING_TARGET` message. When no configured
marker is decoded, the bridge immediately pauses that stream instead of
re-sending a stale position or inventing a zero target. ArduPilot then expires
the missing measurement using its own precision-landing timeout. The companion
heartbeat continues independently while the target is out of view.

### Camera calibration

Use a flat 10x7 inner-corner checkerboard. This captures 20 different views and
then calibrates (Picamera2 is selected automatically on a Pi). Capture new
images after this update: the checked-in 640x480 images and `camera.json` came
from the Camera V2's cropped sensor mode and are not valid for the full-FOV
1640x1232 sensor mode used in flight. The calibration command checks that the
resulting horizontal field of view is reasonably close to the Camera V2's
62.2-degree specification before writing it.

```bash
.venv/bin/python calibrate_camera.py capture
```

To rerun calibration on the new full-FOV image set without recapturing it:

```bash
.venv/bin/python calibrate_camera.py calibrate
```

To watch the annotated camera feed on a computer connected over SSH, install
`ffplay` there and run:

```bash
ssh pi@PI_ADDRESS 'cd /path/to/APSM && .venv/bin/python calibrate_camera.py capture --stream' \
  | ffplay -f mjpeg -i -
```

The video shows detected checkerboard corners and the saved-image count.
Progress messages remain in the terminal because only video is sent through
standard output.

Use `--columns` and `--rows` for another checkerboard. Set `--width 960
--height 720` (or keep both equal to the camera resolution in
`landing_config.json`). Keep `--sensor-width 1640 --sensor-height 1232` equal
to the flight configuration, then run `main.py pi --calibration camera.json`.

The optional verification helpers are run with:

```bash
.venv/bin/python tests/verify_sitl.py
.venv/bin/python tests/verify_precision_landing.py
```
