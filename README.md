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
