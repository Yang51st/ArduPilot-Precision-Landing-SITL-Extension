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

## Configuration

Edit `aruco_landing_config.json` to change the marker, camera, ground position, or Python MAVLink/frame endpoints. Stop and restart the Python bridge and Simulink model after changing it.
