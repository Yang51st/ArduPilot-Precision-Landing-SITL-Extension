# Process log

A running record of what was actually done, in order, including the dead ends. README.md says how to do things; this file says what happened. Newest entries at the bottom. Add an entry for every session that touches the Pi or the Python bridge.

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
