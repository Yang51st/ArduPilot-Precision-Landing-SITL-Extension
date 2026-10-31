# Next steps: from bench to flight

Written 2026-10-29 at the end of the Pi bring-up session, updated 2026-10-31. This file says what comes next and how to prove each step before the drone flies. Update it as items get done.

Related: [`docs/status.md`](status.md) is the current snapshot (read it first), [`docs/process-log.md`](process-log.md) says what happened, [`README.md`](../README.md) has the commands, [`CLAUDE.md`](../CLAUDE.md) has the Pi rules.

## How to demo what exists today

Two demos, a few minutes each.

**1. SITL on the laptop.** Run the regression test first, then follow "Run the simulation" in README.md exactly as before: Python bridge, Mission Planner SITL with the parameters written, Simulink model, a mission ending in a land command. The drone corrects onto the marker during landing. This shows the refactor kept the working pipeline working.

```powershell
python scripts\test_core.py
```

**2. Pi camera live on the laptop.** Pi plugged in by USB. Start the Pi bridge in dry-run with publishing and open the viewer on the laptop (commands in README.md under "Check the camera stream from your laptop"). Hold the printed ArUco marker (family, ID and size as set in `pi/pi_landing_config.json`, currently `DICT_4X4_250` ID 9 at 3.2 cm) in front of the camera: a green outline and a distance appear in the window, and the Pi terminal reports the ID, its configured size and a body-frame vector. Move the marker to a measured distance and check the printed distance agrees.

**Combined.** Run the laptop bridge against the Pi stream with `--frame-endpoint watarrow-pi:5600`. Real camera, simulated vehicle.

## Remaining work, in order

### 1. Marker matches the print

Done 2026-10-31: `marker_family` is `DICT_4X4_250` and `marker_id` is 9 in both configs, deployed to the Pi. The Pi status line now reports any other marker IDs it sees, so a mismatch reads as "saw other marker ids: ..." instead of "not visible".

Still open:

- Confirm the print is detected with these settings and record the result in `docs/status.md`.
- Confirm the 3.2 cm `size_m` for ID 9 in `pi/pi_landing_config.json` with a ruler on the black border of the print. Done 2026-10-31 from Josef's stated size; not yet measured.
- Design the competition pad: the SAE 2027 rules allow an 8 ft by 8 ft (2.44 m) passive coroplast pad with no electronics, so the `markers` list in `aruco_landing_config.json` (ID 9 at 2.2 m, ID 10 at 0.3 m) is a first guess. A 4x4 marker only has room for a smaller marker inside one of its white cells (about 0.37 m for a 2.2 m marker), and a marker that fills that cell turns the cell black for the outer decoder, so the inner size or placement needs a test.
- Regenerate the SITL marker image to match. `markers/eAruco.png` is still the 6x6 ID 0 picture, so the SITL demo will not detect until a 4x4 picture with IDs 9 and 10 at the configured sizes replaces it and `marker_image_width_m` is set to that picture's real width.

### 2. Camera calibration

The Pi config uses the nominal 62.2 degree field of view of the Pi Camera V2, and the pose estimate is only as good as that guess. Print a chessboard, capture about 20 images with the Pi camera at the bridge resolution, run OpenCV's calibration, save the camera matrix and distortion as JSON, and pass it with `--calibration`. The core already reads that file and refuses it if the image size does not match. About half a day.

### 3. Flight controller link

Right now the Pi detects the marker but has nobody to tell. The link is a serial cable: three wires from the Pi's GPIO header to a telemetry port on the flight controller. Pi TX to controller RX, Pi RX to controller TX, ground to ground. Do not connect 5 V unless the decision has been made that the controller powers the Pi.

Pi side, three one-time settings, best done by a small script run once with Josef's approval (it edits `config.txt`, which is a system change under the rules):

- Enable the UART. By default the header pins carry no serial port.
- Take the Linux login console off that port, or it will send boot text into the flight controller.
- Disable Bluetooth. The Zero 2W has one proper UART and one mini UART. Bluetooth takes the proper one by default, and the mini UART changes speed with the CPU clock, which corrupts MAVLink. Disabling Bluetooth gives the proper UART to the pins.

Flight controller side, two parameters on the chosen telemetry port: protocol MAVLink 2, baud matching the Pi (57600 is the safe default). Add them to `precision-landing.parm`.

Software side: the core already speaks MAVLink. It needs two config keys, `mavlink_endpoint` as a serial device path and `mavlink_baud`, passed through to pymavlink. This was deferred on 2026-10-29 and goes back into `precland_core.py` when the hardware arrives.

Pass condition: start the bridge without `--dry-run`; it prints "connected to system 1" and then "parameters verified", the same two lines seen in SITL.

### 4. Parameter check for real hardware

The bridge refuses to run unless the vehicle's precision-landing settings are right. One of the checks is `SERIAL2_PROTOCOL`, the SITL port, which will be wrong on a real controller using a different port. Move the required-parameter list into the config file so the SITL config checks SITL's port and the Pi config checks the real one. About ten lines in `precland_core.py`.

### 5. Bench test with the controller connected

Once the controller is connected, every detection is sent as LANDING_TARGET straight away: each marker ID has one size, so no vehicle height is needed. Hold a printed marker in front of the camera with the controller powered and Mission Planner attached. The Pi terminal shows the ID, size and body-frame vector, and Mission Planner's messages tab shows precision-landing status. That proves the whole chain: camera, detection, MAVLink, controller.

### 6. Autostart

On the drone there is no laptop to start the bridge. A systemd unit starts it on boot and restarts it if it crashes, about ten lines. Install it last; until everything above works, a bridge that starts by itself only gets in the way.

### 7. Flight test

Three stages, each only after the previous one looks right:

1. Hover in Loiter over the marker at a few metres. Watch Mission Planner for the target-acquired message and the Pi log for detection distance tracking the altitude. Nothing lands.
2. Land with precision landing enabled from a hover already over the marker. Keep a hand on the mode switch to abort to Loiter.
3. Land after flying away and returning, which is what the real mission does.

Real vehicles need tuning SITL did not: how far the controller trusts the target, how fast it corrects, and the lowest height at which it still uses the camera. These are the `PLND_` parameters and will change between flights.

### 8. Housekeeping

Commit the 2026-10-31 changes (configs, core status line, docs), decide what to do with the untracked rpi-imager files in the repo root (`cache/`, `create_local_json.py`, `os_list_local.rpi-imager-manifest`), keep the process log and `docs/status.md` going.

## Verifying the connection before flying

Every link in the chain can be proven on the bench with props off. Each step has one line of output that says it passed.

| Step | What you do | Pass signal | If it fails |
| --- | --- | --- | --- |
| 1. Serial wiring | Start the bridge without `--dry-run` | "connected to system 1" within 5 s | "no ArduPilot heartbeat": swap TX and RX, or check baud. Nothing can be damaged by this. |
| 2. Parameters | Same run, next line | "parameters verified" | A list of which parameters are wrong and their expected values |
| 3. Status text flowing back | Same run, watch the Pi terminal | Lines prefixed "ArduPilot:" appear when the controller reports PrecLand status | Controller not sending STATUSTEXT on that port; check the serial protocol parameter |
| 4. Target messages going out | Marker in front of the camera | Pi shows the marker ID, its size and a body-frame vector; Mission Planner messages show target acknowledged | Move the marker left; the sideways value should change sign. If not, the camera orientation is wrong |
| 5. Controller decision, still on the bench | Arm with props off, switch to Land | Status messages report a valid target | Check `PLND_` parameters and that the marker ID matches |
| 6. Hover test, props on | Loiter at a few metres over the marker, do not land | Target-acquired message; detection distance tracks altitude | Stay in Loiter and land manually |

Only after all six should it be allowed to land itself.

Lab setup for all of this: Pi on USB to the laptop, Pi on serial to the controller, controller on USB to Mission Planner. Three screens at once, one per hop.

## Small tools to build for the bench

- **Other IDs in the status line.** Done 2026-10-31: when the target is not visible the Pi prints which other marker IDs it saw.
- **Logging flag.** Write every detection and every MAVLink message to a file, so a failed test leaves evidence.
- **Echo mode.** A Pi-side option that prints every MAVLink message the controller sends and nothing else. Turns the serial wiring check into something readable instead of inferred.
- **UART setup script.** The three Pi-side settings from item 3, idempotent, backing up `config.txt` with a dated copy first.
