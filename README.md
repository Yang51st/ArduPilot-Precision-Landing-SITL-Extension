# ArduPilot Precision Landing SITL Extension

This project adds a simulated downward-facing ArUco camera and a Python precision-landing bridge to an ArduPilot Copter SITL session started from Mission Planner. Simulink renders the camera image, and the Python bridge detects the marker and sends MAVLink `LANDING_TARGET` messages to SITL.

## Prerequisites

- Python 3 and a virtual environment
- Mission Planner with Copter SITL
- MATLAB and Simulink with the toolboxes required by the model's MAVLink, TCP/IP, and video-viewer blocks

The default configuration uses local TCP ports `5762` (Simulink/MAVLink), `5763` (Python/MAVLink), and `5600` (Simulink camera frames). Make sure these ports are available.

## One-time Python setup

From the repository root, create and activate a virtual environment, then install the bridge dependencies:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install numpy opencv-contrib-python pymavlink
```

`opencv-contrib-python` is required because the bridge uses OpenCV's ArUco detector.

## Run the simulation

Start the components in this order:

1. **Start the Python bridge first.** It will wait and retry while Mission Planner and Simulink start.

   ```powershell
   .\.venv\Scripts\Activate.ps1
   python scripts\aruco_landing_bridge.py --config aruco_landing_config.json
   ```

2. **Start Mission Planner second.** Open Mission Planner, start a Copter SITL simulation, and wait for it to connect.

3. **Write the precision-landing parameters to SITL.** In Mission Planner, open the Full Parameter List/Tree, load `precision-landing.parm`, and select **Write Params**.

4. **Reboot SITL without wiping its parameters.** Return to Mission Planner's **Simulation** tab and click the **Multicopter** icon again. Before starting it, make sure the **Wipe** checkbox is **unchecked**; otherwise, the precision-landing parameters written in the previous step will be lost. Wait for SITL to reconnect.

5. **Start Simulink last.** Open `mavlinkMST_ArucoCamera.slx` in MATLAB and click **Run**. The model connects to SITL, renders the simulated camera, and publishes RGB frames to the waiting Python bridge. The Python bridge checks that the required precision-landing settings are present when it connects.

When everything is connected, the Python terminal reports verified ArduPilot parameters, a connected Simulink RGB stream, and marker detections. Stop the Simulink model and press `Ctrl+C` in the Python terminal when finished.

6. **Setting up a mission** Go to the **Plan** tab in Mission Planner and place a takeoff command, followed by a series of waypoints to fly to, and finally a land command near the original takeoff position. The Simulink camera simulation places the ArUco marker at the original home position of the SITL drone, which should coincide with the original takeoff position. To complete the process, click the **Write** button to load the mission onto the SITL drone.

7. **Running a mission** Go to the **Data** tab and select the **Actions** panel on the left side of the window. Arm the drone by clicking the **Arm/Disarm** button, then click the **Resume Mission** button, accept all prompts that pop up, and then set the flight mode to **Auto**. The drone should now be executing its mission, and you should be able to seethe Simulink camera view change according to the drone's simulated flight.

## Configuration

Edit `aruco_landing_config.json` to change the marker, camera, ground position, or Python MAVLink/frame endpoints. Stop and restart the Python bridge and Simulink model after changing it.

## Raspberry Pi setup

The end goal of this project is to take the tested-in-SITL code and put it into a Raspberry Pi board (RPi Zero 2W) hooked up with an RPiCam V2, which will allow for precision landing functionality on a real-world drone. To prepare the Raspberry Pi board for this, follow the excellent guide at ```https://www.raspberrypi.com/news/usb-gadget-mode-in-raspberry-pi-os-ssh-over-usb/```. But TLDR:

1. **Download the Raspberry Pi Imager software** This software runs on all standard operating systems.

2. **Enable USB gadget mode functionality option** By default, enabling USB gadget mode is not an option for the Raspberry Pi Imager. Go to
```https://github.com/raspberrypi/rpi-imager/tree/main/doc/local_json```
and download and run the Python file according to the instructions.

3. **Flash the 64-bit Trixie OS with no desktop interface** Use the Raspberry Pi Imager to do this, follow the prompts and be sure to select SSH and USB gadget functionalities.

4. Insert the flashed micro SD card into the RPi, wait 2 to 3 minutes for the initial boot-up of the device, and then connect to the board via its USB port. You can now login via SSH using the username and hostname (or default address of ```10.12.194.1```) you had previously set up. For Windows users, a device driver needs to be installed from ```https://github.com/raspberrypi/rpi-usb-gadget/releases```.

**Note** if all the RPi boards have the same username and hostname, the ssh protocol may think that some shady stuff is going on an refuse to connect after giving you a long warning. To clear this error, run ```ssh-keygen -R hostname```, where you replace ```hostname``` with what your chosen hostname or default address is.

### What runs where

The Python code is split so the Pi carries only what it needs. Nothing MATLAB- or Simulink-related goes on the board.

| File | Laptop | Pi | Purpose |
| --- | :-: | :-: | --- |
| `scripts/precland_core.py` | yes | yes | ArUco detection, pose estimate, MAVLink output, frame stream protocol |
| `scripts/aruco_landing_bridge.py` | yes | | Reads Simulink frames, feeds SITL (the command in "Run the simulation" is unchanged) |
| `scripts/pi_landing_bridge.py` | | yes | Reads the Pi camera, optionally publishes frames to the laptop |
| `scripts/view_frames.py` | yes | | Shows the Pi's (or Simulink's) frame stream in a window |
| `scripts/test_core.py` | yes | | Regression test for the shared core, no MATLAB or Pi needed |
| `pi/pi_landing_config.json` | | yes | Camera and marker settings for the real camera |
| `pi/setup_pi.sh` | | yes | Installs packages and creates the venv on the Pi |
| `pi/deploy_to_pi.ps1` | yes | | Copies the four Pi files over SSH and runs the post-deploy check |

The Pi and the laptop talk over the same raw RGB frame protocol that Simulink already uses, so `aruco_landing_bridge.py --frame-endpoint watarrow-pi:5600` can also run the laptop-side detection on real camera frames.

### SSH without password prompts

The deploy script never asks for a password, so install the laptop's public key on the Pi once per board. If `~\.ssh\id_ed25519` does not exist yet, create it from Git Bash, where an empty passphrase is reliable (PowerShell tends to set one by accident):

```bash
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 -q && chmod 600 ~/.ssh/id_ed25519
```

Then install it from PowerShell. This asks for the Pi password once:

```powershell
ssh watarrow@watarrow-pi "mkdir -p ~/.ssh && echo '$(Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub)' >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

### Give the Pi internet for the install

Over the USB gadget link the Pi has no internet, and `setup_pi.sh` needs it for apt and pip. The simplest route is the Zero 2W's own Wi-Fi, which leaves the USB link untouched:

```bash
sudo nmcli dev wifi connect "<network name>" password "<password>"
```

Wi-Fi is only needed for the install; the bridge itself runs without it.

### Install the bridge on the Pi

1. Run the regression test on the laptop first. It must print `All checks passed.`

   ```powershell
   .\.venv\Scripts\Activate.ps1
   python scripts\test_core.py
   ```

2. Copy the Pi files over. The script creates `~/precland` on the Pi, copies the four files, and runs the post-deploy check. On a fresh board the check reports that the venv is missing, which is expected.

   ```powershell
   .\pi\deploy_to_pi.ps1
   ```

3. On the Pi, run the setup once. It installs OpenCV, numpy and Picamera2 from apt, creates a venv that can see those packages, installs pymavlink into it, then prints the package versions and the cameras libcamera can see.

   ```bash
   ssh watarrow@watarrow-pi
   cd ~/precland
   ./setup_pi.sh
   ```

   Later deploys only need step 2; the script reruns the check each time with `./setup_pi.sh check`.

### Check the camera stream from your laptop

No flight controller is involved yet, so the Pi bridge runs in dry-run mode: it detects the marker but sends no MAVLink. With `--publish` it also serves its frames, with the detection drawn on, to the laptop.

1. On the Pi:

   ```bash
   cd ~/precland
   .venv/bin/python pi_landing_bridge.py --config pi_landing_config.json --dry-run --publish 0.0.0.0:5600
   ```

2. On the laptop:

   ```powershell
   python scripts\view_frames.py --endpoint watarrow-pi:5600
   ```

   A window opens with the live camera image and a frame rate counter. Hold a printed marker in front of the camera: a green outline and the estimated distance appear on the laptop, and the Pi terminal reports detections with `scale unresolved (no vehicle height yet, not sent)`. That text is correct for bench testing: the eAruco target carries the same ID at two sizes, and without a vehicle height the bridge refuses to guess which one it is seeing. Press `q` in the window to quit.

   If the stream stutters over the USB link, add `--publish-scale 0.5` on the Pi. If colours look swapped, see the comment in `Picamera2FrameSource` in `scripts/pi_landing_bridge.py`.

3. Stopping. Press `q` or Esc in the viewer window. On the Pi, `Ctrl+C` stops a bridge started in your SSH session. To run the bridge so it survives the SSH session closing, and to stop it afterwards:

   ```bash
   cd ~/precland
   setsid nohup .venv/bin/python pi_landing_bridge.py --config pi_landing_config.json --dry-run --publish 0.0.0.0:5600 < /dev/null > bridge.log 2>&1 &
   echo $! > bridge.pid
   tail -f bridge.log          # watch it; Ctrl+C stops the tail, not the bridge
   kill $(cat bridge.pid)      # stop the bridge
   ```

   To stop it from the laptop in one line, use single quotes so PowerShell does not expand the `$(...)` locally:

   ```powershell
   ssh watarrow@watarrow-pi 'kill $(cat ~/precland/bridge.pid)'
   ```

4. Unplugging. The Pi is powered by the USB cable, so pulling it while the OS is running can corrupt the SD card. Shut down first, wait about 15 seconds after the connection closes until the green activity LED stops blinking, then unplug:

   ```powershell
   ssh -t watarrow@watarrow-pi "sudo poweroff"
   ```

   On the next boot, allow two to three minutes before connecting. The Wi-Fi connection and the SSH key both persist.

Two settings in `pi/pi_landing_config.json` are still the SITL values and must be updated for a real test: `marker_size_m` and `outer_marker_size_m` have to match the printed marker, and `camera_hfov_deg` is the nominal Pi Camera V2 value until a calibration file is made.

### Not yet done

Wiring the Pi to a flight controller (UART enable, serial MAVLink endpoint, autostart on boot, the real-hardware parameter check) is deferred until the drone is built. Until then the Pi bridge always runs with `--dry-run`. The plan for that work, and the bench checks to pass before any flight, are in `docs/next-steps.md`. What has been done so far, including what went wrong, is in `docs/process-log.md`.
