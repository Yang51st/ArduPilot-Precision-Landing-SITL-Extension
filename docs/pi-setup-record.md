# Raspberry Pi: current state and how to rebuild it

Written 2026-10-31 from a read-only audit of the board over SSH, after Josef wired the Pi to the flight controller and proved the MAVLink link with MAVProxy following https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html. Everything below was read from the Pi itself (boot files, device tree, installed packages, MAVProxy's saved logs), not assumed. Items marked **confirm** could not be read from the board and need Josef's word.

Purpose: if the SD card dies or the board is swapped, follow the rebuild checklist at the end and the Pi comes back to exactly this state.

Related: [`status.md`](status.md) is the snapshot, [`next-steps.md`](next-steps.md) the roadmap, [`process-log.md`](process-log.md) the history, [`../README.md`](../README.md) the how-to, [`../CLAUDE.md`](../CLAUDE.md) the rules.

## Hardware

| Item | Value |
| --- | --- |
| Board | Raspberry Pi Zero 2 W Rev 1.0, 512 MB |
| Camera | Raspberry Pi Camera V2 (imx219) on the CSI ribbon. Unplugged on purpose while the flight controller was wired (Josef, 2026-10-31); not detected during the audit for that reason. Reconnect with the Pi powered off |
| Flight controller | Holybro Pixhawk 6C (`Pixhawk6C-bdshot` firmware target), ArduPlane V4.7.1 (git dbe79216), ChibiOS d103983f, IOMCU present |
| Airframe as configured on the controller | QuadPlane (`Q_ENABLE 1`, `Q_FRAME_CLASS 7`, tricopter VTOL motors), ELRS receiver on CRSF (`SERIAL1_PROTOCOL 23`), VL53L0X I2C rangefinder (`RNGFND1_TYPE 16`, address 41) |
| Pi to controller link | Pi UART on the 40-pin header to the controller's SERIAL2, which is the TELEM2 port on a Pixhawk 6C. **Confirm** which pins: the standard is Pi pin 8 (GPIO14, TXD) to TELEM2 RX, Pi pin 10 (GPIO15, RXD) to TELEM2 TX, Pi pin 6 (GND) to TELEM2 GND |
| Pi power | **Confirm.** The ArduPilot page advises against taking 5 V from the TELEM connector. Until now the Pi has been powered from the laptop's USB cable |
| Laptop link | USB gadget Ethernet, hostname `watarrow-pi`, user `watarrow`, SSH key login (see CLAUDE.md) |

## Operating system and network

Unchanged from the 2026-10-29 bring-up (process log): Raspberry Pi OS Trixie Lite 64-bit, Debian 13, Python 3.13.5, flashed with Raspberry Pi Imager with SSH and USB gadget mode enabled. Wi-Fi joined with `nmcli` for internet; NetworkManager keeps the connection. USB gadget link `usb0` at 10.12.194.1/28. **Never touch either network setting** (CLAUDE.md rule 2).

Clock note: the Zero 2 W has no real-time clock. File dates set before Wi-Fi brings NTP up are wrong (they show the last shutdown time saved by fake-hwclock, currently 2026-10-30 00:30). Dates on `/boot/firmware/config.txt`, `cmdline.txt` and `.bash_history` are therefore not evidence of when a change happened.

## Layer 1: precision-landing bridge (from the repo)

Installed by `pi/setup_pi.sh` on 2026-10-29, deployed with `pi/deploy_to_pi.ps1`. Lives entirely in `~/precland`:

| Path | Origin |
| --- | --- |
| `precland_core.py`, `pi_landing_bridge.py`, `pi_landing_config.json`, `setup_pi.sh` | Copied from this repo by the deploy script; md5 matched the repo on 2026-10-31 after the `markers` change |
| `.venv/` | `python3 -m venv --system-site-packages`, created by `setup_pi.sh` |
| `bridge.log`, `bridge.pid` | Left by a detached bridge run (README "Stopping") |
| `MyCopter/` | Created by MAVProxy's `--aircraft MyCopter` on 2026-10-31 (layer 2 below) |

apt packages added by `setup_pi.sh` with `--no-install-recommends`: `python3-venv python3-opencv python3-numpy python3-picamera2 rpicam-apps-lite`. pip inside the venv: `pymavlink` (2.4.50).

## Layer 2: MAVLink link to the flight controller (added 2026-10-31 by Josef)

What the ArduPilot page asks for, and what is on the board now.

### 2a. UART enabled, serial console disabled

Done with `sudo raspi-config` (it is the last line of the saved shell history): Interface Options, Serial Port, "login shell over serial" = No, "serial port hardware enabled" = Yes, then reboot. Resulting state, verified:

| Check | Result |
| --- | --- |
| `/boot/firmware/config.txt` | `enable_uart=1` appended under `[all]`, after the imager's `dtoverlay=dwc2,dr_mode=peripheral`. No other change |
| `/boot/firmware/cmdline.txt` | `console=tty1 root=PARTUUID=24edfd81-02 rootfstype=ext4 fsck.repair=yes rootwait cfg80211.ieee80211_regdom=CA ds=nocloud;i=rpi-imager-1793118895752`. The default `console=serial0,115200` entry is gone |
| `raspi-config nonint get_serial_cons` / `get_serial_hw` | `1 0`, meaning console off, hardware on |
| `/dev/serial0` | Symlink to `/dev/ttyS0`, group `dialout`, mode 660. `watarrow` is in `dialout` |
| `serial-getty@ttyS0` | Inactive. Nothing else holds the port when the bridge and MAVProxy are stopped |
| Which UART | `serial0` is the **mini UART** (`/soc/serial@7e215040`). The full PL011 (`serial1`, `/soc/serial@7e201000`) is still assigned to Bluetooth; `bluetooth.service` is enabled, `dtoverlay=disable-bt` was not added |
| Core clock | `core_freq=400`, fixed by `enable_uart=1`, so the mini UART's baud rate is stable. Measured 400 MHz |

The mini UART worked at 921600 baud for the six-minute MAVProxy test below. If the link ever shows CRC errors or drops under load, the known fix is `dtoverlay=disable-bt` in `config.txt` (moves the PL011 to the header pins and makes `serial0` point at `ttyAMA0`). That is a system change and needs Josef's OK first.

### 2b. MAVProxy installed into the bridge venv

Done from `~/precland` at 18:34 Pi time (the `REQUESTED` marker on the package says it was asked for by name, not pulled in):

```bash
cd ~/precland
.venv/bin/pip install MAVProxy        # 1.8.75
```

Side effects to know about, because they changed the bridge's own environment:

| Package pulled in | Why it matters |
| --- | --- |
| `opencv-python 5.0.0.93` (pip wheel) | MAVProxy lists it as a hard dependency. Inside the venv it now **shadows the apt OpenCV 4.10** that `setup_pi.sh` chose on purpose. `import cv2` in the venv gives 5.0.0; `aruco` is present and a smoke test detected ID 9 and solved a pose. `setup_pi.sh check` now prints `opencv 5.0.0`. Frame rate with this build has not been measured yet (the apt build did about 21 fps at 640x480) |
| `matplotlib 3.11.2`, `vtk 9.7.1`, `quantized-mesh-tile`, `pynmeagps`, `defusedxml`, `contourpy`, `fonttools`, `kiwisolver`, `cycler`, `pillow` | MAVProxy dependencies. Harmless, about 150 MB of disk |
| `pandas-stubs 3.1.0` | Installed separately at 18:42, by name. Josef: it fixed an issue the MAVProxy install hit with `types-seaborn`, which is one of the many `types-*` stub packages the system Python ships. Keep it; a rebuild needs it only if the same error reappears |

Options if the OpenCV change turns out to matter: `.venv/bin/pip uninstall opencv-python` restores the apt 4.10 build (MAVProxy's map and camera modules then fail to import, but the console and `--master`/`--out` routing still work), or install MAVProxy in its own venv outside `~/precland`. Decision pending; nothing was changed during the audit.

### 2c. Link test with MAVProxy

Run from `~/precland` (that is where `MyCopter/` was created), per the ArduPilot page:

```bash
cd ~/precland
.venv/bin/mavproxy.py --master=/dev/serial0 --baudrate 921600 --aircraft MyCopter
```

Result, read from `~/precland/MyCopter/logs/2026-10-31/flight1/`:

| Item | Value |
| --- | --- |
| Session | 18:46:49 to 18:52:41 Pi time, 35,973 MAVLink messages received |
| Vehicle | system 1, component 1, type fixed wing, autopilot ArduPilot |
| Firmware banner | `ArduPlane V4.7.1 (dbe79216)`, `Pixhawk6C-bdshot`, `IOMCU: 410 2003 411FC231`, `IMU0: fast sampling 2.0kHz/2.0kHz` |
| Also connected at the time | Mission Planner 1.3.83 (its banner appears in the status text), presumably over USB |
| Receiver | `RCInput: decoding CRSF`, `ELRS: bootstrap complete, fw 1.00` |
| Pre-arm | `PreArm: Waiting for RC` throughout (transmitter off); no arm attempted from MAVProxy |
| Files | `mav.parm` (full parameter dump), `defaults.parm`, `flight.tlog` 1.4 MB, `flight.tlog.raw` |

### 2d. Flight controller parameters relevant to this project

From `mav.parm`. Only `SERIAL2_BAUD` differs from the board default among the serial settings.

| Parameter | Value | Meaning |
| --- | --- | --- |
| `SERIAL2_PROTOCOL` | 2 | MAVLink 2 on TELEM2 |
| `SERIAL2_BAUD` | 921 | 921600 baud, as the ArduPilot page prescribes |
| `SERIAL1_PROTOCOL` | 23 | RC input (CRSF/ELRS) on TELEM1 |
| `SERIAL0_BAUD` / `SERIAL7_PROTOCOL` | 115 / 2 | USB console and second USB MAVLink endpoint, used by Mission Planner |
| `PLND_ENABLED` | **0** | Precision landing is not enabled on the real vehicle yet. The bridge's parameter check will refuse to run until `PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_EST_TYPE 0`, `PLND_ALT_MIN 0`, `PLND_ALT_MAX 0`, `SERIAL2_PROTOCOL 2` are all set |
| `RC5_OPTION` | 153 | Arm/disarm switch |
| `ARMING_SKIPCHK` | 37406 | Several pre-arm checks are skipped. Review before any flight |
| `BRD_SAFETY_DEFLT` | 0 | Safety switch disabled at boot |
| `STAT_BOOTCNT` / `STAT_RUNTIME` | 28 / 9576 s | Board has been powered 28 times, 2.7 h total |

Note the firmware is ArduPlane, not ArduCopter. Precision landing in Plane applies to QuadPlane VTOL landings (`QLAND`, `QRTL`) and the parameter set is the same `PLND_` family, but all SITL work so far was on Copter. The bridge's `REQUIRED_PARAMETERS` list in `precland_core.py` is Copter-tested only.

## Findings from the audit that need action

1. **The camera is unplugged.** Josef disconnected the Pi camera temporarily while wiring the flight controller, which is why `rpicam-hello --list-cameras`, `vcgencmd get_camera` and `setup_pi.sh check` all reported no camera during the audit. Not a fault. Reconnect it with the Pi powered off, then rerun `./setup_pi.sh check` and expect the imx219 listed again.
2. **OpenCV in the venv is now the pip 5.0 wheel**, see 2b. Decide whether to keep it. Re-measure bridge frame rate once the camera is back.
3. **Only one program may open `/dev/serial0` at a time.** MAVProxy and the bridge cannot both talk to the controller over the same UART. Stop MAVProxy before starting the bridge without `--dry-run`, or route through MAVProxy's `--out` later.
4. **`PLND_` parameters are unset on the controller.** Item 2d.

## Rebuild checklist (fresh SD card to this state)

1. Flash Raspberry Pi OS Trixie Lite 64-bit with Raspberry Pi Imager: hostname `watarrow-pi`, user `watarrow`, SSH on, USB gadget mode on (README "Raspberry Pi setup").
2. Boot on USB, wait 3 minutes, install the laptop's SSH key (CLAUDE.md, step 2 of Claude's protocol).
3. Join Wi-Fi for internet: `nmcli radio wifi on`, wait a minute for the scan, `nmcli dev wifi connect "<ssid>" password "<pw>"`. Retry once if the SSID is not found yet (process log 2026-10-29).
4. From the laptop: `.\pi\deploy_to_pi.ps1` (copies the four files; the check will say the venv is missing).
5. On the Pi: `cd ~/precland && ./setup_pi.sh` (apt packages, venv, pymavlink, check). Expect `opencv 4.10.0 aruco: True` and the imx219 listed.
6. `sudo raspi-config`: Interface Options, Serial Port, login shell No, hardware Yes. Finish, reboot. Verify `ls -l /dev/serial0` shows `ttyS0` and `cmdline.txt` has no `console=serial0`.
7. Optional diagnostic tool: `cd ~/precland && .venv/bin/pip install MAVProxy` (brings OpenCV 5.0 into the venv, see 2b). If it fails with an error mentioning `types-seaborn`, run `.venv/bin/pip install pandas-stubs` and retry; that is what fixed it on 2026-10-31. Alternatively `python3 -m venv ~/mavproxy-venv && ~/mavproxy-venv/bin/pip install MAVProxy` keeps the bridge venv untouched.
8. Wire TELEM2 to the header as in "Hardware" (TX to RX crossed, GND to GND, no 5 V unless decided). Set `SERIAL2_PROTOCOL 2`, `SERIAL2_BAUD 921` on the controller.
9. Prove the link: `mavproxy.py --master=/dev/serial0 --baudrate 921600 --aircraft MyCopter`, expect the `ArduPlane V4.7.1` banner within a few seconds. If nothing arrives, swap TX and RX.
10. Record the result in `process-log.md` and update `status.md`.
