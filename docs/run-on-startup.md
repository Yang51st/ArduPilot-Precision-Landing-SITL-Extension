# Run the bridge on startup

Set up and proven on 2026-11-11. The Pi starts the precision-landing bridge by itself on every boot, using a systemd service. This page is the how-to; `docs/process-log.md` has the blow-by-blow.

## What is installed

| Item | Value |
| --- | --- |
| Service | `precland.service`, at `/etc/systemd/system/precland.service` on the Pi |
| Copy in the repo | `pi/precland.service` (on the `pi-bridge` branch) |
| What it runs | `/home/watarrow/APSM/.venv/bin/python main.py pi`, from `/home/watarrow/APSM`, as user `watarrow` |
| Code | `main.py` from the `reduced` branch. `landing_config.json` on the Pi lists marker ID 0 at 0.18 m, an edit that is not in git |
| On a crash or exit | Restarted after 2 seconds |
| State | Enabled and running |

The unit file:

```ini
[Unit]
Description=ArUco precision landing bridge
After=multi-user.target

[Service]
User=watarrow
WorkingDirectory=/home/watarrow/APSM
ExecStart=/home/watarrow/APSM/.venv/bin/python main.py pi
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

## Why systemd

The starting point was the Dexter Industries article listing five ways to run a program at startup: https://www.dexterindustries.com/howto/run-a-program-on-your-raspberry-pi-at-startup/

| Method | Verdict |
| --- | --- |
| systemd | **Chosen.** The only one that restarts the program after a crash, starts it at a defined point in boot and keeps a log |
| rc.local | Simple, but no restart, no log, runs as root |
| .bashrc | Runs on login, not on boot, and starts another copy on every SSH login |
| init.d | The older mechanism systemd replaced. The article's example would not work as written |
| crontab `@reboot` | Works, but no restart and no easy stop |

Changes from the article's systemd example: added `Restart=always`, `User=watarrow` and `WorkingDirectory`; the file goes in `/etc/systemd/system`, not `/lib/systemd/system`; and the article's `> file 2>&1` logging tip does not work, because systemd does not run the command through a shell. Use `journalctl` instead.

## Everyday commands

All run on the Pi. The ones with `sudo` ask for the Pi password.

```bash
systemctl status precland.service           # is it running
journalctl -u precland.service -f           # live log, Ctrl+C to leave
journalctl -b -u precland.service           # log since this boot
sudo systemctl stop precland.service        # stop until next boot
sudo systemctl start precland.service       # start again
sudo systemctl restart precland.service     # after changing main.py or the config
sudo systemctl disable precland.service     # do not start on boot
sudo systemctl enable precland.service      # start on boot again
```

**The service sends to the Pixhawk and holds `/dev/serial0` from boot.** Stop it before running MAVProxy, a `--dry-run` test or the bridge by hand.

## What a healthy log looks like

```
Bridge: starting
MAVLink: connecting to /dev/serial0
MAVLink: connected; precision-landing parameters verified
Camera: Picamera2 640x480
Vision: 0/21; marker not visible
Vision: 21/21; id 0, body FRD [-0.11, +0.13, +0.60] m, target RPY [-1.0, -7.3, +1.2] deg
```

| If the log says | It means |
| --- | --- |
| "no ArduPilot heartbeat; retrying", every 6 s | Nothing is arriving from the Pixhawk: it is off, TELEM2 is unplugged or mis-wired, or the baud rates differ. It connects by itself once the controller answers |
| "incorrect or missing ArduPilot parameters: ..." | Set the named parameters in Mission Planner |
| "marker not visible" with the marker in view | The print's ID is not the one in `landing_config.json` |
| `status` shows `activating (auto-restart)` | The program crashes at launch. Usually a wrong path in the unit or a missing package in the venv |

## How to set it up again (fresh SD card)

1. Put the `reduced` files in `/home/watarrow/APSM` and build its venv, per that branch's README.
2. Check both paths exist:

   ```bash
   ls /home/watarrow/APSM/main.py /home/watarrow/APSM/.venv/bin/python
   ```

3. Copy `pi/precland.service` from the repo to `/etc/systemd/system/precland.service` on the Pi (owner root, mode 644).
4. Make sure nothing holds the serial port: `pgrep -af mavproxy` and `pgrep -af main.py` should print nothing.
5. Load it and start it once by hand:

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl start precland.service
   journalctl -u precland.service -f
   ```

6. When the log shows "connected; precision-landing parameters verified", enable it and reboot:

   ```bash
   sudo systemctl enable precland.service
   sudo reboot
   ```

7. After two to three minutes, SSH back in and check `systemctl status precland.service` and the log again.

After any edit to the unit file: `sudo systemctl daemon-reload`, then `sudo systemctl restart precland.service`.

## What was proven on 2026-11-11

| Check | Result |
| --- | --- |
| Files on the Pi | `main.py` and `camera.json` match `origin/reduced` byte for byte; the venv imports OpenCV 5.0.0 with ArUco, pymavlink and Picamera2 |
| First start, controller off | Service active, no restarts, retrying with "no ArduPilot heartbeat". The UART had received zero bytes |
| Controller powered on | Connected without a restart, parameters verified, camera at about 20 frames per second |
| Marker ID 0 in view | Detected in 20 or 21 of 21 frames at about 0.60 m. The UART transmit counter rose from 213 to 17,613 bytes, so targets were written to the port |
| Enable and reboot | SSH back after about 50 s. Service started by itself and logged "connected; precision-landing parameters verified" 4 s after starting |

Notes from the session:

- The first version of the unit pointed at `~/precland`, the old folder. The original is kept on the Pi as `~/precland.service.bak-2026-11-11`.
- Python takes 4 to 8 seconds from the service starting to its first log line on the Zero 2 W.
- The Pi has no clock battery, so journal dates are wrong (it read Nov 06). Times within one boot are still consistent.
- The mini UART counted 55 receive overruns in about six minutes. If that climbs fast or the link drops, the known fix is `dtoverlay=disable-bt` (a system change, needs Josef's OK).

## Not yet checked

Autostart works. These are about the landing system as a whole, and are listed in `docs/status.md`:

- A Mission Planner tlog showing the Pixhawk receives the targets from this code.
- A marker in view after the reboot, and the Pi booting before the Pixhawk.
- The distance scale against a ruler (the nominal camera model and `camera.json` differ by 2.4 times).
- Whether ArduPlane acts on the target in `QLAND` or `QRTL`.
- `SERIAL2_BAUD 57` in the Proto3 transfer file, which would cut the link if loaded.
