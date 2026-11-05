# Process log

A running record of what was actually done, in order, including the dead ends. Newest entries at the bottom. Add an entry for every session that touches the Pi or the Python bridge.

Related: [`docs/status.md`](status.md) is the current snapshot, [`docs/next-steps.md`](next-steps.md) is the roadmap, [`README.md`](../README.md) is the how-to, [`CLAUDE.md`](../CLAUDE.md) holds the Pi rules.

## 2026-10-29: split the bridge, prepare the Pi

### Starting point

- SITL pipeline working: Mission Planner SITL, Simulink camera model, one Python script `scripts/aruco_landing_bridge.py` doing frame reading, detection and MAVLink output.
- Pi Zero 2W flashed with Raspberry Pi OS Trixie Lite, USB gadget mode SSH enabled per the Raspberry Pi blog guide. Hostname `watarrow-pi`, user `watarrow`.

### Rules agreed before any code

Written to `CLAUDE.md`: repo is the only source of truth for Pi files, never touch Pi networking, system changes need Josef's OK, regression test before deploy, verify after deploy, Pi bridge runs in dry-run only, flight controller wiring deferred.

### Code split (laptop, no Pi needed)

1. Confirmed the working bridge was already committed unchanged in git, so a restore point existed without a new commit.
2. Moved detection, pose, MAVLink and the retry loop into `scripts/precland_core.py`. One behaviour change: a marker seen without a vehicle height is reported as "scale unresolved" instead of silently dropped. It is still never sent.
3. Rewrote `scripts/aruco_landing_bridge.py` as a thin Simulink-only wrapper. Run command unchanged.
4. New `scripts/pi_landing_bridge.py` (Picamera2 source, OpenCV fallback, frame publisher with overlay), `scripts/view_frames.py` (laptop viewer), `scripts/test_core.py` (regression test), `pi/pi_landing_config.json`, `pi/setup_pi.sh`, `pi/deploy_to_pi.ps1`.
5. Verified: `test_core.py` passes (distance within 1% at both marker scales); dry-run start of the Simulink bridge prints the same waiting line as before; a localhost loopback of publisher to reader passed with the marker still detectable through the overlaid frame.

### SSH key setup (the part that took several tries)

What went wrong, so nobody repeats it:

- The `!` prefix in the Claude prompt runs Git Bash, not PowerShell. A PowerShell-style `$env:USERPROFILE` path did not expand, and the key pair was written into the repo folder under a literal name starting with `:USERPROFILE`. Moved into `~/.ssh`, repo cleaned.
- The `!` prefix also has no terminal attached, so SSH cannot show a password prompt. It fails with "Permission denied" after three empty attempts. Anything that needs a password must run in a separate PowerShell window.
- Generating the key from PowerShell with `-N '""'` produced a key with a passphrase. A passphrase-protected key is useless for Claude. Keys are now generated from Git Bash with `-N ""` and verified with `ssh-keygen -y -P ""`.
- The Pi's host key was already trusted but saved under `watarrow-pi.local`, not `watarrow-pi`, so strict host checking refused the plain name. Registered under the plain name with `StrictHostKeyChecking=accept-new` once.
- Private key file had mode 644 after the move; set to 600.

Working sequence, now in `CLAUDE.md`: Claude generates the key in Bash, Josef installs it from PowerShell with one `ssh` command that embeds the public key, Claude then uses `ssh -o BatchMode=yes`.

### Pi inspection (read-only)

| Item | Found |
| --- | --- |
| Board | Raspberry Pi Zero 2 W Rev 1.0, aarch64 |
| OS | Debian 13 Trixie, Python 3.13.5 |
| RAM | 415 MB total, 273 MB available at idle |
| Disk | 53 GB free |
| Camera | imx219 detected; 640x480 is a cropped mode, 1640x1232 is full field of view |
| apt | python3-opencv 4.10, python3-picamera2 0.3.37, python3-numpy 2.2 available |
| Network | usb0 at 10.12.194.1/28, no internet, Wi-Fi radio off in software |
| Serial | `/dev/serial0` absent (expected, UART deferred) |

The camera modes confirmed the Pi config choice of a 1640x1232 sensor request with 640x480 output.

### Deploy

`pi\deploy_to_pi.ps1` copied the four files to `~/precland` and ran `setup_pi.sh check`, which reported the venv missing. Expected before the setup script has run.

### Getting the Pi internet (Josef's actions, over SSH in PowerShell)

- `sudo` over `ssh host "command"` failed with "a terminal is required". Fix: `ssh -t`.
- `nmcli radio wifi on` worked, but the immediate `nmcli dev wifi connect` failed with "No network with SSID found" because the first scan had not finished. A scan a minute later showed the network at good signal on 2.4 GHz. Retrying the connect alone is the fix.
- Wi-Fi was chosen over Windows internet sharing because it leaves the USB gadget link untouched.

### First setup attempt failed: no network

`./setup_pi.sh` was run before Wi-Fi had connected. The Pi had no wireless connection saved and no default route, so `apt-get update` could not resolve any host and every package download failed. Nothing was installed and nothing was damaged. The saved terminal output (246 KB) also showed apt trying to pull in Qt, GTK and other desktop extras as recommended packages for OpenCV and Picamera2.

Changes made in the repo and redeployed:

- `setup_pi.sh` now checks for internet first and stops with a clear message instead of running apt blind.
- `apt-get install` uses `--no-install-recommends` to keep desktop libraries off a headless board.

### Wi-Fi connected

Second `nmcli dev wifi connect` attempt failed on a one-digit typo in the network name; the third succeeded and `ping deb.debian.org` returned replies. The USB gadget link stayed up throughout. The Wi-Fi connection is saved by NetworkManager, so it reconnects on boot while in range.

### Setup script succeeded, with one side effect

`./setup_pi.sh` installed the apt packages, created the venv, installed pymavlink, and the import check passed: Python 3.13.5, numpy 2.2.4, OpenCV 4.10.0 with aruco, pymavlink 2.4.50, Picamera2 importable. Disk used after install: 3.7 GB.

Side effect: `rpicam-hello --list-cameras` failed with a symbol lookup error in `librpicam_app.so.1`. Cause: `python3-picamera2` pulled libcamera up to 0.7.2+rpt20261009, while the preinstalled `rpicam-apps-lite` stayed at 1.13.0 and needs 1.14.0 to match. Picamera2 itself is unaffected and lists the imx219, so the bridge works. Fix: install `rpicam-apps-lite` so it upgrades with libcamera. The setup script now includes it in the install list, and its check uses Picamera2 as the primary camera test since that is what the bridge uses.

### Bench test: Pi camera stream to the laptop

Starting the bridge over SSH took three tries. `pkill -f pi_landing_bridge.py` matched the remote shell's own command line (the launch command is in it) and killed the shell before anything ran, twice. Working pattern: anchor on the Python process, `pkill -f "^\.venv/bin/python pi_landing_bridge"`, and start with `setsid nohup ... < /dev/null > bridge.log 2>&1 &` with the pid saved to `bridge.pid`.

Result with the bridge in `--dry-run --publish 0.0.0.0:5600` and the laptop reading over the USB link:

| Measure | Value |
| --- | --- |
| Pi frame rate processed | about 21 frames per second at 640x480 |
| Laptop receive rate | 19.7 fps, 2 of 42 frames dropped |
| Pi CPU | one core at about 92 percent (of four) |
| Pi RAM used by the bridge | about 180 MB of 415 MB |
| Image | real scene, mean brightness 130, not blank |

No marker was in view, so detection reported "target not visible" as expected. `view_frames.py` on the laptop shows the live image.

### rpicam-apps upgraded to match libcamera

Josef ran `sudo apt-get install -y --no-install-recommends rpicam-apps-lite` (approved system change). It upgraded `librpicam-app1`, `rpicam-apps-core` and `rpicam-apps-lite` from 1.13.0 to 1.14.0, 556 kB. One earlier attempt failed with "Unable to locate package" because stray Escape and Backspace keystrokes had mangled the pasted command; a clean retry worked.

### End of day summary

**What worked**

- Splitting the bridge into a shared core plus two thin wrappers. The Simulink-side command is unchanged and the Pi carries four files.
- The regression test on the laptop. It caught nothing today, but it proved the refactor before anything went near the board, and it runs in two seconds with no MATLAB.
- Reusing the Simulink frame protocol for Pi-to-laptop frames. The laptop needed no new code to read the Pi, and the loopback test on the laptop predicted the real result.
- apt for OpenCV, numpy and Picamera2, venv with `--system-site-packages`, pip only for pymavlink. Install took a few minutes on the Zero 2W with no build steps.
- Picamera2 with a 1640x1232 sensor request and 640x480 output: about 21 frames per second processed on one core, 20 fps received over the USB link, almost no drops.
- Wi-Fi for internet instead of Windows connection sharing. The USB link never went down.
- The deploy script's post-deploy check. It told us immediately what state the board was in each time.
- The rules. Every system change went through Josef, nothing was edited on the Pi by hand, and the only two things that broke were both outside `~/precland` and both cheap to fix.

**What did not work, and why**

| Attempt | Failure | Cause | Fix |
| --- | --- | --- | --- |
| Key generation via the `!` prefix with PowerShell syntax | Key written into the repo folder under a literal `:USERPROFILE` name | `!` runs Git Bash, so `$env:USERPROFILE` never expanded | Moved the files; key generation now happens in Bash |
| Key install via the `!` prefix | "Permission denied" after three empty password tries | `!` has no terminal, so SSH cannot prompt | Password steps run in a separate PowerShell window |
| Key generation from PowerShell with `-N '""'` | Key ended up with a passphrase | PowerShell does not pass an empty argument reliably | Regenerated in Bash; verified with `ssh-keygen -y -P ""` |
| Strict host checking on `watarrow-pi` | "Host key verification failed" | Key was saved under `watarrow-pi.local` only | Registered under the plain name once with `accept-new` |
| `sudo` over `ssh host "cmd"` | "a terminal is required" | No tty allocated | `ssh -t` |
| `nmcli dev wifi connect` right after `radio wifi on` | "No network with SSID found" | First scan had not completed | Wait a few seconds, retry the connect alone |
| Second connect attempt | Same error | One-digit typo in the network name | Checked against the scan list |
| First `setup_pi.sh` run | Every download failed on name resolution | Run before Wi-Fi was connected | Script now checks for internet first and stops |
| `pkill -f pi_landing_bridge.py` over SSH, twice | The remote shell died silently before launching anything | The shell's own command line contained the script name | Anchor the pattern on the Python process: `pkill -f "^\.venv/bin/python pi_landing_bridge"` |
| `rpicam-hello` after the install | Symbol lookup error in `librpicam_app.so.1` | Picamera2 pulled a newer libcamera; preinstalled rpicam-apps stayed behind | Install `rpicam-apps-lite` alongside; now in the setup script |
| One `apt-get install` paste | "Unable to locate package" with an empty name | Stray Escape and Backspace keystrokes in the pasted line | Clean retry |

**What we learned**

1. The `!` prefix in the Claude prompt is Git Bash with no terminal. Use it for things that need no password and no PowerShell syntax. Anything interactive goes in a real PowerShell window.
2. Claude can only reach the Pi with a passphrase-free key, and only Claude's Bash tool generates one reliably. Verify with `ssh-keygen -y -P ""` before assuming.
3. Always use `ssh -t` when the remote command uses `sudo`.
4. The Zero 2W's 640x480 native camera mode is a crop with a narrower field of view. Full field of view at low resolution means requesting the 1640x1232 binned mode and letting libcamera scale.
5. On Trixie, apt's OpenCV 4.10 already has the aruco module. No contrib build needed on the Pi.
6. Installing `python3-picamera2` upgrades libcamera. Anything else linked against libcamera must be upgraded in the same apt call or it breaks.
7. `--no-install-recommends` matters on a headless board. Without it, OpenCV and Picamera2 drag in Qt, GTK and a desktop stack.
8. Over SSH, `pkill -f` with a bare script name matches the invoking shell when the launch command is in the same line. Anchor the pattern or use a pid file.
9. The bridge uses about 180 MB of the Pi's 415 MB and most of one core at 640x480. There is headroom, but not for a second heavy process; keep the preview scaled down if the link gets busy.
10. Logging failures as they happen made this section possible. Writing it at the end would have lost half the table.

**Numbers worth keeping**

| Measure | Value |
| --- | --- |
| Pi RAM | 415 MB total, 273 MB available before the bridge starts |
| Disk after install | 3.7 GB used, 51 GB free |
| Camera | imx219, full field of view only from the 1640x1232 binned mode |
| Bridge throughput | about 21 fps processed, 20 fps received on the laptop |
| Bridge footprint | about 180 MB RAM, one core at about 92 percent |
| Package versions | OpenCV 4.10.0, numpy 2.2.4, pymavlink 2.4.50, Picamera2 0.3.38, libcamera 0.7.2, rpicam-apps 1.14.0 |

### Still to do

- Update `pi/pi_landing_config.json` marker sizes to the printed marker; the 4 m values are from SITL.
- Hold a printed marker in front of the Pi camera and confirm the overlay and the "scale unresolved" message on the bench.
- Make a camera calibration file for the Pi Camera V2 and pass it with `--calibration`; the nominal 62.2 degree field of view is a stopgap.
- Decide what to do with the untracked rpi-imager files (`cache/`, `create_local_json.py`, `os_list_local.rpi-imager-manifest`).
- Commit.
- Deferred until a flight controller is wired: UART, serial endpoint and baud, autostart on boot, the real-hardware parameter check.

## 2026-10-31: marker ID changed to 9

Josef changed `marker_id` from 0 to 9 in both `aruco_landing_config.json` and `pi/pi_landing_config.json` to match the printed marker. Claude tidied stray spaces in the Pi config, ran `scripts/test_core.py` (passes; the test builds its marker from the config so it now exercises ID 9), confirmed the Pi was up, and ran `pi/deploy_to_pi.ps1`. Post-deploy check unchanged from 2026-10-29 (OpenCV 4.10.0 aruco True, imx219 listed). The Pi config now reads ID 9.

The bridge was already running on the Pi from Josef's SSH session with the old config loaded, so it still looks for ID 0 until restarted. Lesson: a config deploy does not take effect until the bridge is stopped and started again.

Open question: `aruco_landing_config.json` now says ID 9 but `markers/eAruco.png`, which Simulink renders, is still the ID 0 image. The SITL bridge will not detect until that image is regenerated with ID 9 or the SITL config goes back to 0.

**Marker not detected after the ID change.** Josef restarted the bridge (new pid, started 23:06) and reported no detections. Claude could not grab a frame to look at: the publisher serves one viewer at a time and Josef's viewer held the slot, so a second connection accepted but never received data. The bridge gave no clue either, because a marker with a different ID or dictionary was reported as a bare "target not visible". Fix: `ArucoDetector` now remembers every non-target ID it sees and the status line prints "target id 9 not visible (saw other marker ids: 0; check marker_id in the config)". Test passes; redeployed. Takes effect once the bridge is restarted.

**Marker family changed to DICT_4X4_250.** Josef changed `marker_family` in both configs from `DICT_6X6_250` to `DICT_4X4_250` to match the printed marker, then ran the test and deploy himself. Confirmed afterwards: `test_core.py` passes with the 4x4 family, the Pi config reads 4x4 ID 9, and the deployed `precland_core.py` is byte-identical to the repo copy.

**Truncated command.** A bridge start failed with `FileNotFoundError: 'pi_landing_config'`. Cause: the command had been copied from a message that was cut off mid-line, so `.json` was missing. Not a bug.

**Docs restructured.** Added `docs/status.md` as the entry point for new sessions, cross-linked all four documents, and updated the roadmap. Whether the 4x4 ID 9 print is now detected was not recorded by the end of the session; `docs/status.md` flags it as unconfirmed.

## 2026-10-31: one size per marker ID, height-based scale logic removed

### Why

Josef reported the Pi dry run printing "scale unresolved" and a 1.4 m distance for a 3.2 cm print. Investigation: the message was structural, not a detection problem. The Pi config still carried the SITL pair `marker_size_m` 0.469 m and `outer_marker_size_m` 4.0 m, which put the core in two-size mode, and a dry run never receives a vehicle height, so every detection stayed unresolved. The 1.4 m came from the 0.469 m size applied to a 3.2 cm print (ratio 14.7, so the real distance was about 9.5 cm). Before the split the same case was silently dropped; the split only made it visible.

Checked the 2027 SAE Aero Design rules PDF (version 2027.0, 57 pages) for ArUco guidance: no mention of ArUco, nested markers or fiducials anywhere. Relevant constraints are in section 8.6: the DLZ is an 8 ft by 8 ft coroplast pad the team builds, no electronics, batteries or magnets in it, no material under it, and section 2.20 bans lasers for marking a landing zone. Marker design is entirely the team's choice.

### Change

Josef asked to replace the size pair with a list of markers, each ID with its own size. Done:

- `scripts/precland_core.py`: new `parse_markers` reads `"markers": [{"id": 9, "size_m": 2.2}, {"id": 10, "size_m": 0.3}]` and rejects the old keys with a message naming them, empty lists, duplicate IDs and non-positive sizes. `ArucoDetector` takes a `{id: size}` dict; `detect` looks the size up from the ID, so there is nothing to disambiguate, and when several listed markers are visible it uses the one covering the most pixels. Removed: `scale_resolved`, `expected_camera_z_m`, `local_down_m`, the LOCAL_POSITION_NED message-interval request and the "unresolved" counter. The status line now reads `id 9 (0.032 m) body_frd=[...] m distance=... m`, or `marker ids 9, 10 not visible (saw other marker ids: 0; ...)`.
- `scripts/pi_landing_bridge.py`: overlay no longer appends "(scale?)".
- `scripts/test_core.py`: rewritten. Each configured marker alone gives its own size and a distance within 5 percent; with two markers visible the larger in pixels wins regardless of physical size; an unlisted ID is ignored but remembered; body-frame mapping and header round-trip as before.
- `aruco_landing_config.json`: `markers` ID 9 at 2.2 m and ID 10 at 0.3 m as a first guess for the 2.44 m SAE pad. `marker_image_width_m` and `ground_down_m` stay because Simulink reads them. `markers/eAruco.png` still does not match and must be regenerated.
- `pi/pi_landing_config.json`: `markers` ID 9 at 0.032 m, the size Josef gave for the print; `ground_down_m` and `marker_image_width_m` dropped.
- `aruco_camera_config.m`: falls back to the largest `markers` size when `marker_image_width_m` is absent; comment updated.
- README, `docs/next-steps.md`, `docs/status.md` updated to match.

Restore point: the pre-change working tree was saved as a patch in the session scratchpad; the last commit `fb86663` predates the 2026-10-31 edits.

### Verified

`python scripts\test_core.py` passes (22 checks). Bad configs are rejected with the intended messages. The Simulink bridge starts in dry run and prints `Markers: DICT_4X4_250, ID 9 at 2.200 m, ID 10 at 0.300 m`. Deployed with `pi\deploy_to_pi.ps1`; post-deploy check unchanged (OpenCV 4.10.0 aruco True, imx219 listed) and the three Pi files are byte-identical to the repo by md5.

The bridge running on the Pi (pid 3614, in Josef's SSH terminal) still has the old code loaded. It must be stopped with Ctrl+C and started again before the new config takes effect. Claude did not kill it because it belongs to an interactive session.

### Open

- Nested placement for the competition pad: a 2.2 m 4x4 marker has 0.367 m cells, and a 0.3 m marker inside a white cell leaves too little quiet zone and flips that cell black for the outer decoder. Needs a smaller inner marker, a different placement, or a test of the dictionary's error correction.
- Regenerate the SITL marker picture for IDs 9 and 10 and set `marker_image_width_m` to its width.
- Measure the 3.2 cm print with a ruler to confirm `size_m`.

## 2026-10-31 evening: flight controller wired, MAVLink proven, Pi audited

### What Josef did (reconstructed from the Pi, not watched)

Following https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html:

1. `sudo raspi-config`, Serial Port: login shell No, hardware Yes, reboot. Result: `enable_uart=1` in `/boot/firmware/config.txt`, `console=serial0,115200` removed from `cmdline.txt`, `/dev/serial0` pointing at `ttyS0` (mini UART, Bluetooth untouched, core clock fixed at 400 MHz).
2. Wired the Pixhawk 6C TELEM2 to the Pi header with three wires: TX, RX, GND. No 5 V from the controller; the Pi stays on USB power (Josef's note, added after the audit).
3. `.venv/bin/pip install MAVProxy` from `~/precland` at 18:34 Pi time. MAVProxy 1.8.75 brought `opencv-python 5.0.0.93`, matplotlib, vtk and friends into the bridge venv. `pandas-stubs` was installed by name at 18:42 to fix an issue the install hit with the system's `types-seaborn` stub package (Josef's note).
4. `mavproxy.py --master=/dev/serial0 --baudrate 921600 --aircraft MyCopter` from `~/precland`, 18:46:49 to 18:52:41. MAVProxy's log shows 35,973 messages from system 1: `ArduPlane V4.7.1 (dbe79216)`, `Pixhawk6C-bdshot`, ELRS receiver on CRSF, `PreArm: Waiting for RC` throughout. Mission Planner 1.3.83 was connected at the same time, presumably over USB. Controller has `SERIAL2_PROTOCOL 2`, `SERIAL2_BAUD 921`, `PLND_ENABLED 0`.

### Audit (Claude, read-only over SSH)

Read `config.txt`, `cmdline.txt`, device-tree aliases, `vcgencmd`, systemd unit states, group membership, venv package list with install times and `REQUESTED` markers, `pip show` dependency chains, MAVProxy's `mav.parm`, `defaults.parm` and `flight.tlog` (parsed with pymavlink on the Pi), and ran `setup_pi.sh check` plus an ArUco smoke test with the venv's OpenCV. Nothing on the Pi was changed. Written up as `docs/pi-setup-record.md` with a rebuild checklist.

Findings:

- **Camera absent.** `rpicam-hello --list-cameras`: "No cameras available"; `vcgencmd get_camera`: `detected=0`; Picamera2 lists nothing. The same check listed the imx219 at 00:11 Pi time. Josef later confirmed the camera was unplugged on purpose during the flight controller wiring, so this is expected and not a fault.
- **OpenCV swapped under the bridge.** The venv now imports the pip 5.0.0 wheel instead of apt 4.10. `aruco` works (ID 9 detected, pose solved, 0.085 m for a 200 px marker at the assumed 0.032 m size). Frame rate not re-measured.
- **Clock caveat.** Without an RTC, files written before NTP syncs carry the fake-hwclock time (2026-10-30 00:30). Boot-file dates are meaningless; the journal is volatile so the reboot count is not visible either.
- The shell history on disk ends at `sudo raspi-config` because the current SSH session has not logged out yet; the later commands were inferred from package metadata and MAVProxy's own files.

Docs updated: `status.md` (UART row, camera row, three new in-progress rows, deferred section), `next-steps.md` item 3 and item 4, `CLAUDE.md` rules 7 and 8, README "Not yet done".

### Open

- Reconnect the camera (Pi off), rerun `./setup_pi.sh check`.
- `docs/pi-fc-link.html`, a planning diagram drawn before the wiring, was deleted at Josef's request on 2026-10-31.
- Decide on the OpenCV wheel (keep, uninstall, or move MAVProxy to its own venv).
- Add `mavlink_endpoint` `/dev/serial0` and `mavlink_baud` 921600 to the core and Pi config.
- Set the `PLND_` parameters on the Pixhawk and check Plane's precision-landing behaviour against the Copter-tested bridge.

## 2026-11-04: serial endpoint and baud for the real link

### What Josef did

- Edited `pi/pi_landing_config.json`: `mavlink_endpoint` from the placeholder to `/dev/serial0`, and a new key `mavlink_baud` 921600.
- Changed one parameter on the Pixhawk (his report; which one is not recorded and Claude has not read it back).

### What Claude found

The config edit alone did nothing for the baud. `build_mavlink` passed only the endpoint to pymavlink, so `mavlink_baud` was ignored and the port would have opened at pymavlink's default of 115200. Against a controller at 921600 that ends in "no ArduPilot heartbeat on companion link" after five seconds, which looks like a wiring fault but is not.

Also cleared up on the way: `SERIAL2_BAUD 921` on the Pixhawk is ArduPilot shorthand. pymavlink takes the real rate, so the Pi side is 921600.

### What Claude changed (laptop only, nothing deployed)

- `scripts/precland_core.py`: `MavlinkLandingTarget` takes an optional `baud` and passes it to `mavutil.mavlink_connection`; `build_mavlink` reads `mavlink_baud` from the config and rejects values under 1200 with a message naming the full rate. The SITL config has no such key and behaves as before. The parameter error now says "ArduPilot" instead of "SITL".
- `scripts/pi_landing_bridge.py`: removed the warning that said no flight controller link exists, and updated the docstring and `--mavlink` help to match.
- `scripts/test_core.py`: three new checks (SITL config leaves the default, Pi config gives `/dev/serial0` at 921600, 921 is rejected).
- `python scripts\test_core.py`: all checks pass.

### Deploy (20:20 Pi time)

Josef first typed `.\pi\deploy_to_pi.ps1` into the Pi's SSH shell, which fails with "command not found": the script is a laptop PowerShell script that pushes to the Pi. Claude ran it from the laptop instead.

- Before: `test_core.py` passed; `hostname && uptime` gave `watarrow-pi`, up 32 min; `pgrep` found no bridge and no MAVProxy running.
- `pi\deploy_to_pi.ps1`: "Deploy complete."
- `./setup_pi.sh check`: python 3.13.5, numpy 2.2.4, opencv 5.0.0 with aruco, pymavlink 2.4.50, picamera2 importable. **The camera is back**: Picamera2 and `rpicam-hello` both list the imx219. 271 MB memory available, disk 9% used.
- A first `pgrep` attempt with a quoted `a|b` pattern was mangled by PowerShell (the quotes were stripped and the `|` became a pipe on the Pi). Use one `pgrep -af [p]attern` per process instead.

### First run without `--dry-run` (20:31 Pi time, started by Josef)

Josef started `pi_landing_bridge.py --config pi_landing_config.json --publish 0.0.0.0:5600` in his SSH terminal, then reported no `LANDING_TARGET` in Mission Planner's MAVLink inspector. Claude checked from the laptop, read-only:

- `ps`: the bridge (pid 2898) is running without `--dry-run`. `fuser /dev/ttyS0` names that pid and `stty -F /dev/ttyS0 speed` gives 921600, so the new baud code is in effect.
- A probe script read ten frames from port 5600. Frames only reach the publisher from inside the frame loop, which the core enters only after the heartbeat arrives and the parameter check passes. **So the link is up and the six parameters are verified, by inference from the code path.** The terminal lines "connected to system 1" and "parameters verified" are in Josef's window and were not seen by Claude.
- The detector, run on the laptop over those ten frames with the Pi config, found no marker and no other marker IDs. The saved frame shows a person and a room, no printed marker. With no detection the bridge sends nothing at all, not even its heartbeat (the heartbeat is sent from inside `send`), which is why the inspector shows no Vehicle 42.
- The probe makes the Pi terminal print one "viewer connected" and one "viewer disconnected" line.

### Detections on the Pi, nothing in the inspector (19:54 laptop time)

Josef held the marker in view: the Pi terminal showed detections, the MAVLink inspector still showed no `LANDING_TARGET`. Claude read Mission Planner's telemetry logs in `Documents\Mission Planner\logs` with pymavlink (on copies, read-only):

- `2026-11-04 19-01-03.tlog` (19:01:05 to 19:08:20, before the bridge ran): only system 1 and Mission Planner (255). It records the controller's parameters: `PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_EST_TYPE 0`, `PLND_ALT_MIN 0`, `PLND_ALT_MAX 0`, `SERIAL2_PROTOCOL 2`, `SERIAL2_BAUD 921`. So all six required values were set by 19:08. `SERIAL2_OPTIONS`, `MAV_OPTIONS` and `MAV1/2/3_OPTIONS` are all 0, so MAVLink forwarding is not switched off on any port.
- `2026-11-04 19-08-38.tlog` is the log of the current connection and covers the bridge run. Mission Planner holds it with an exclusive lock, so it cannot be read until Mission Planner disconnects. **Whether anything from system 42 reached Mission Planner is therefore still unknown.**
- The Pi clock runs about 50 minutes ahead of the laptop (no RTC, no NTP over the USB link), so Pi times in this log are not laptop times.

### The closed tlog: the whole chain works (19:59 laptop time)

Josef disconnected Mission Planner at 19:58:42. On disconnect Mission Planner moves the log to `logs\FIXED_WING\1\`. Scan of `2026-11-04 19-08-38.tlog` (19:08:40 to 19:58:42):

- **System 42, component 191: 2,253 `LANDING_TARGET` and 120 `HEARTBEAT`.** So Pi camera, detection, serial link, Pixhawk and the Pixhawk's forwarding to USB all work. This is the first proof of the chain on real hardware, and it meets the pass condition of next-steps item 3.
- Arrival times: 954 messages in 19:50 (about 16 per second), 39 in 19:51, none until 19:57, then 471 and 789. The gaps are the marker being out of view. Only 120 heartbeats in 50 minutes confirms the heartbeat goes out only with detections.
- Contents: `target_num 9`, `frame 12` (body FRD), `type 2`, `position_valid 1`. Distance 0.085 to 0.413 m, median 0.158 m, for the 3.2 cm print held by hand.
- System 1 sent no status text mentioning precision landing in that window, so nothing yet shows the controller acting on the target.

Why the inspector showed nothing is not explained. The messages were in Mission Planner's own log, so it is a display question, not a link fault.

### Open

- Find where the inspector shows system 42, or confirm it does not.
- Evidence that ArduPlane's precision-landing code uses the target, not just that the message arrives. The firmware is Plane and the bridge was only tested against Copter in SITL.
- The bridge's heartbeat only goes out alongside a detection. A companion that is silent whenever the marker is out of view looks the same as a dead one; consider sending the heartbeat every second regardless.
- The parameter check needs all six of `PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_EST_TYPE 0`, `PLND_ALT_MIN 0`, `PLND_ALT_MAX 0`, `SERIAL2_PROTOCOL 2`. One changed parameter cannot satisfy it, since `PLND_ENABLED` was 0 and the others were at defaults. The `PLND_` family may only show up in Mission Planner after `PLND_ENABLED` is set to 1 and the parameters are refreshed or the controller rebooted.
- First run without `--dry-run` (next-steps item 3 pass condition): MAVProxy must be stopped first.
