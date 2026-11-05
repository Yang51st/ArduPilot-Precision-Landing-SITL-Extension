# Working notes for this repo

## Raspberry Pi SSH access

The Pi (Zero 2W, Raspberry Pi OS Trixie Lite) is reached over USB gadget mode: plug its USB data port into the laptop and it appears as a network device. Allow two to three minutes after power-up before trying to connect.

- Hostname: `watarrow-pi` (fallbacks: `watarrow-pi.local`, or the gadget address noted in README.md)
- Username: `watarrow`
- Password: the team default. It is deliberately not written here because this file is in git.
- Project folder on the Pi: `~/precland`

### Manual login (a person at the keyboard)

```powershell
ssh watarrow@watarrow-pi
```

If SSH refuses with a long "REMOTE HOST IDENTIFICATION HAS CHANGED" warning, the board was reflashed or swapped with another one using the same hostname. Clear the stale fingerprint and retry:

```powershell
ssh-keygen -R watarrow-pi
```

### Claude's protocol (no prompts possible)

Claude cannot type a password, so every board needs the laptop's public key installed once. Split the work this way:

1. **Claude generates the key** (only if `~/.ssh/id_ed25519` is missing) from its Bash tool, where an empty passphrase is reliable. Do not generate it from PowerShell: passing an empty passphrase there is unreliable and a key with a passphrase is useless to Claude.

   ```bash
   ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 -q && chmod 600 ~/.ssh/id_ed25519
   ssh-keygen -y -P "" -f ~/.ssh/id_ed25519   # must print the key, proving no passphrase
   ```

2. **Josef installs it on the Pi** from a separate PowerShell window, not through the Claude prompt: the `!` prefix has no terminal attached, so SSH cannot show the password prompt and fails with "Permission denied" after three empty tries. This asks for the Pi password once.

   ```powershell
   ssh watarrow@watarrow-pi "mkdir -p ~/.ssh && echo '$(Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub)' >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
   ```

Repeat step 2 after every reflash, after clearing the old fingerprint with `ssh-keygen -R watarrow-pi`.

Once the key is in place, Claude runs commands and copies files like this:

```powershell
ssh -o BatchMode=yes -o ConnectTimeout=5 watarrow@watarrow-pi 'hostname && uptime'
scp -o BatchMode=yes scripts/precland_core.py watarrow@watarrow-pi:~/precland/
```

Rules for Claude:

1. Start every Pi session with the `hostname && uptime` check above. If it fails, stop and report; do not retry with other hostnames in a loop. Ask Josef whether the board is plugged in and booted.
2. Always pass `BatchMode=yes` so a missing key fails in seconds instead of hanging on a password prompt.
3. Never store the Pi password in this repo, in memory, or in any script.
4. Never change anything that could break the USB link: no edits to the gadget configuration, `cmdline.txt`, `/etc/network`, dhcpcd, or NetworkManager on the Pi. The USB link is the only way in.
5. Before Josef unplugs the Pi: stop the bridge, confirm it is gone with `pgrep`, run `sync`, then tell him to run `sudo poweroff` from PowerShell (needs sudo, so Claude cannot) and wait for the activity LED to stop before pulling the cable. The USB cable is also the power supply, so an unplug without shutdown risks SD card corruption.
6. From PowerShell, remote commands containing `$` must be in single quotes, or PowerShell expands them locally before SSH sends them.

## Pi code editing rules

These exist so a bad change on the Pi is always cheap to undo.

1. **This repo is the only source of truth.** Everything the Pi runs is edited here and copied over with `pi\deploy_to_pi.ps1`. Never edit a file in place on the Pi, not even for a quick debug. If something needs changing, change it here and redeploy.
2. **Never touch Pi networking.** Same rule as above, repeated because it is the one that strands the board.
3. **System changes on the Pi need Josef's OK every time.** That means apt installs, `config.txt`, raspi-config, systemd units, anything outside `~/precland`. The one standing exception is `pi/setup_pi.sh`: once Josef has read it, running it on a fresh board is approved.
4. **Git restore point before refactoring the working bridge.** Confirm the current `scripts/aruco_landing_bridge.py` is committed before splitting or rewriting it.
5. **Run `python scripts\test_core.py` on the laptop before any deploy.** It must pass. It needs no MATLAB, no network, and no Pi.
6. **Verify after every deploy.** Run `./setup_pi.sh check` on the Pi (venv imports plus camera list) and report the real output, not an assumption.
7. **The Pi bridge still runs with `--dry-run` by default.** A Pixhawk 6C is wired to `/dev/serial0` since 2026-10-31 and the config names that port at 921600 baud since 2026-11-04, and the first run without `--dry-run` passed that day with the `PLND_` parameters set. Running without `--dry-run` sends to the real controller, so it stays Josef's call, not Claude's. Bench testing must stay possible without a drone. Never run the bridge without `--dry-run` while MAVProxy holds `/dev/serial0`.
8. **Done by Josef, do not redo:** UART enable and serial console removal via `raspi-config`. **Still deferred:** systemd autostart, real-hardware parameter checks, `dtoverlay=disable-bt`. What is on the board is recorded in `docs/pi-setup-record.md`; read it before touching anything outside `~/precland`.

What goes on the Pi is exactly four files, all in `~/precland`: `precland_core.py`, `pi_landing_bridge.py`, `pi_landing_config.json`, `setup_pi.sh`. Everything Simulink- or MATLAB-related stays on the laptop.

## Status, process log and roadmap

`docs/status.md` is the snapshot: what works, what is in progress, what is uncommitted. Read it first in every new session and update it whenever the state changes, including the "Last updated" date.

`docs/process-log.md` records what was actually done, in order, including things that failed and why. README.md is the how-to; the log is the history. Add to it during every session that touches the Pi or the Python bridge, as the work happens rather than at the end, so a session that stops early still leaves a record.

`docs/next-steps.md` is the roadmap: how to demo what exists, the remaining work in order, and the bench verification ladder to climb before any flight. Read it at the start of a session to pick up where things left off, and tick items off as they get done.
