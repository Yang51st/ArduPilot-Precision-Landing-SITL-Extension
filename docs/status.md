# Project status

Start here in a new session. This page is the snapshot; the other documents hold the detail. Update the date and the tables whenever something changes hands.

**Last updated: 2026-11-05.**

## Where things are written down

| Read this | When |
| --- | --- |
| `docs/status.md` (this page) | First thing in a new session, to see what works and what is next |
| [`README.md`](../README.md) | To run anything: SITL simulation, Pi install, camera stream, stopping and unplugging |
| [`CLAUDE.md`](../CLAUDE.md) | Rules for working on the Pi and for Claude's SSH access. Read before touching the Pi |
| [`docs/next-steps.md`](next-steps.md) | The roadmap: remaining work in order and the bench checks to pass before flying |
| [`docs/process-log.md`](process-log.md) | The history: what was done each session, what failed and why |
| [`docs/pi-setup-record.md`](pi-setup-record.md) | Exactly what is on the Pi and on the flight controller, and the checklist to rebuild it from a blank SD card |

## What works today

| Piece | State | Proven by |
| --- | --- | --- |
| SITL pipeline on the laptop (Mission Planner, Simulink camera, Python bridge) | Working as of the 2026-10-29 refactor | Full SITL run per README "Run the simulation" is Josef's regression check; `scripts/test_core.py` covers the shared core |
| Shared core `scripts/precland_core.py` | Working. Config now lists markers as `"markers": [{"id", "size_m"}, ...]`, one size per ID, no vehicle height needed | `python scripts\test_core.py` passes, last run 2026-11-04 with the 4x4 family, IDs 9 and 10, plus the new endpoint and baud checks |
| Pi Zero 2W over USB gadget SSH, key-based login | Working | `hostname && uptime` check, 2026-10-31 |
| Pi venv with OpenCV, Picamera2, pymavlink, MAVProxy | Installed. OpenCV is now the pip 5.0 wheel that MAVProxy pulled in, not the apt 4.10 build; aruco smoke test passes | `./setup_pi.sh check` on the Pi, 2026-10-31 evening |
| Pi UART to the flight controller | **Working.** `enable_uart=1`, serial console off, `/dev/serial0` is the mini UART at 921600 baud. MAVProxy received 35,973 messages from a Pixhawk 6C running ArduPlane 4.7.1 over six minutes | `~/precland/MyCopter/logs/2026-10-31/flight1/` on the Pi; details in `docs/pi-setup-record.md` |
| Pi camera stream to the laptop viewer | Worked at about 20 fps at 640x480 on 2026-10-29. Camera reconnected: `./setup_pi.sh check` lists the imx219 again on 2026-11-04. Stream not re-tested since | Bench test 2026-10-29, process log |
| Pi bridge in dry-run | Running on demand, nothing sent | Process log |
| Deploy script `pi\deploy_to_pi.ps1` | Working | Last used 2026-11-04 |

## In progress

| Item | State |
| --- | --- |
| Flight controller precision-landing parameters | All six values the bridge checks are set on the Pixhawk as of 2026-11-04 (`PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_EST_TYPE 0`, `PLND_ALT_MIN 0`, `PLND_ALT_MAX 0`, `SERIAL2_PROTOCOL 2`), read from Mission Planner's tlog of 19:01. The messages reach the controller (see the row below), but whether ArduPlane's precision-landing code acts on them is unproven: no precision-landing status text appeared. Firmware is ArduPlane (QuadPlane), SITL work was Copter |
| Bridge serial endpoint | Written and deployed to the Pi on 2026-11-04. **First run without `--dry-run` the same day worked end to end:** Mission Planner's tlog `logs\FIXED_WING\1\2026-11-04 19-08-38.tlog` holds 2,253 `LANDING_TARGET` messages from system 42 (ID 9, about 16 per second while the marker was in view), forwarded by the Pixhawk from TELEM2 to USB. The MAVLink inspector did not display them, cause unknown. The bridge sends its heartbeat only alongside a detection. `pi/pi_landing_config.json` has `mavlink_endpoint` `/dev/serial0` and `mavlink_baud` 921600 (Josef), and `precland_core.py` now passes the baud to pymavlink (it ignored the key before). Next: make sure MAVProxy is not running, start the bridge without `--dry-run`, expect "connected to system 1" |
| Printed marker matches the config | Family changed to `DICT_4X4_250` and ID to 9 in both configs and deployed to the Pi on 2026-10-31. **Detection of the printed marker with these settings is not yet confirmed in the log.** If the Pi says "target id 9 not visible (saw other marker ids: ...)" the print and the config still disagree |
| Marker sizes | `pi/pi_landing_config.json` lists ID 9 at 0.032 m, the size Josef gave for the print on 2026-10-31. Confirm with a ruler. The earlier 1.4 m reading was the 0.469 m SITL size applied to a 3.2 cm print |
| Competition pad | `aruco_landing_config.json` lists ID 9 at 2.2 m and ID 10 at 0.3 m as a first guess for the 2.44 m SAE pad. The SAE 2027 rules say nothing about ArUco; the pad must be passive coroplast with no electronics. Nested placement inside a 4x4 marker still needs a design check |
| SITL config consistency | `aruco_landing_config.json` now describes 4x4 IDs 9 and 10, but `markers/eAruco.png`, which Simulink renders, is still the 6x6 ID 0 image. The SITL demo will not detect until a matching image is generated |

## Uncommitted work

Nothing tracked is uncommitted as of 2026-11-05: the serial endpoint and baud work of 2026-11-04 is committed and pushed on `pi-bridge`. Untracked and awaiting a decision: `cache/`, `create_local_json.py`, `os_list_local.rpi-imager-manifest` (rpi-imager helper files).

## Deferred

The flight controller is now wired and the Pi UART is enabled (done by Josef on 2026-10-31). The serial endpoint and baud are in the config and code and on the Pi as of 2026-11-04. Still deferred: autostart on boot, the real-hardware parameter check, and `docs/next-steps.md` from item 4 onward. The first run without `--dry-run` passed on 2026-11-04 (next-steps item 3).

## Fastest way to pick up where this left off

1. Read the "In progress" table above.
2. Plug in the Pi, wait two to three minutes, then follow README "Check the camera stream from your laptop".
3. Hold the printed marker up and read the Pi status line. Then update this page with the result.
