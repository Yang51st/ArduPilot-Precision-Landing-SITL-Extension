# Project status

Start here in a new session. This page is the snapshot; the other documents hold the detail. Update the date and the tables whenever something changes hands.

**Last updated: 2026-10-31.**

## Where things are written down

| Read this | When |
| --- | --- |
| `docs/status.md` (this page) | First thing in a new session, to see what works and what is next |
| [`README.md`](../README.md) | To run anything: SITL simulation, Pi install, camera stream, stopping and unplugging |
| [`CLAUDE.md`](../CLAUDE.md) | Rules for working on the Pi and for Claude's SSH access. Read before touching the Pi |
| [`docs/next-steps.md`](next-steps.md) | The roadmap: remaining work in order and the bench checks to pass before flying |
| [`docs/process-log.md`](process-log.md) | The history: what was done each session, what failed and why |

## What works today

| Piece | State | Proven by |
| --- | --- | --- |
| SITL pipeline on the laptop (Mission Planner, Simulink camera, Python bridge) | Working as of the 2026-10-29 refactor | Full SITL run per README "Run the simulation" is Josef's regression check; `scripts/test_core.py` covers the shared core |
| Shared core `scripts/precland_core.py` | Working. Config now lists markers as `"markers": [{"id", "size_m"}, ...]`, one size per ID, no vehicle height needed | `python scripts\test_core.py` passes, last run 2026-10-31 with the 4x4 family, IDs 9 and 10 |
| Pi Zero 2W over USB gadget SSH, key-based login | Working | `hostname && uptime` check, 2026-10-31 |
| Pi venv with OpenCV 4.10 (aruco), Picamera2, pymavlink | Installed | `./setup_pi.sh check` on the Pi, 2026-10-31 |
| Pi camera stream to the laptop viewer | Working, about 20 fps at 640x480 | Bench test 2026-10-29, process log |
| Pi bridge in dry-run | Running on demand, nothing sent | Process log |
| Deploy script `pi\deploy_to_pi.ps1` | Working | Used three times on 2026-10-31 |

## In progress

| Item | State |
| --- | --- |
| Printed marker matches the config | Family changed to `DICT_4X4_250` and ID to 9 in both configs and deployed to the Pi on 2026-10-31. **Detection of the printed marker with these settings is not yet confirmed in the log.** If the Pi says "target id 9 not visible (saw other marker ids: ...)" the print and the config still disagree |
| Marker sizes | `pi/pi_landing_config.json` lists ID 9 at 0.032 m, the size Josef gave for the print on 2026-10-31. Confirm with a ruler. The earlier 1.4 m reading was the 0.469 m SITL size applied to a 3.2 cm print |
| Competition pad | `aruco_landing_config.json` lists ID 9 at 2.2 m and ID 10 at 0.3 m as a first guess for the 2.44 m SAE pad. The SAE 2027 rules say nothing about ArUco; the pad must be passive coroplast with no electronics. Nested placement inside a 4x4 marker still needs a design check |
| SITL config consistency | `aruco_landing_config.json` now describes 4x4 IDs 9 and 10, but `markers/eAruco.png`, which Simulink renders, is still the 6x6 ID 0 image. The SITL demo will not detect until a matching image is generated |

## Uncommitted work

As of 2026-10-31 these are modified and not committed: both config files (new `markers` list), `scripts/precland_core.py` and `scripts/test_core.py` (one size per marker ID, height-based scale logic removed), `scripts/pi_landing_bridge.py`, `aruco_camera_config.m`, `README.md`, `docs/`. Untracked and awaiting a decision: `cache/`, `create_local_json.py`, `os_list_local.rpi-imager-manifest` (rpi-imager helper files). Last commit is `fb86663`.

## Deferred until a flight controller is wired

UART enable on the Pi, serial MAVLink endpoint and baud, autostart on boot, the real-hardware parameter check, and everything in `docs/next-steps.md` from item 3 onward. The Pi bridge always runs with `--dry-run` until then.

## Fastest way to pick up where this left off

1. Read the "In progress" table above.
2. Plug in the Pi, wait two to three minutes, then follow README "Check the camera stream from your laptop".
3. Hold the printed marker up and read the Pi status line. Then update this page with the result.
