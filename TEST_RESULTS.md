# Verification — 23 September 2026

The source package targets Ubuntu 24.04 / ROS 2 Jazzy on Jetson Orin Nano.
This verification ran on the connected **Ubuntu 22.04 x86-64 / ROS 2 Humble PC**.
The actual OAK-D Pro was connected over USB 3. The Jetson and robot motors were
not connected, so this is not a moving-robot or ARM64/Jazzy acceptance test.

## Results

| Check | Result |
|---|---|
| Isolated colcon package build | Passed |
| Python, YAML and shell syntax | Passed |
| Pytest suite | **19 passed**, including real pseudo-terminal serial I/O |
| Actual OAK RGB and aligned depth | 1280×720, approximately 15 Hz; about 94% valid depth in final scene |
| Actual OAK IMU and Madgwick filter | Approximately 200 Hz |
| Robot wheel/IMU EKF | Approximately 30 Hz; stationary wheel RPM was simulated |
| Mapping + TF + depth cloud | Passed automated freshness/content/TF checks |
| Map export | YAML + PGM saved; RTAB-Map database flushed on clean shutdown |
| Saved-map reload | Localization mode loaded a separate database copy, published localization poses and map-to-base TF |
| Nav2 lifecycle | Controller, smoother, planner, behaviors, BT navigator and velocity smoother all active |
| Nav2 plan-only action | ComputePathToPose succeeded, returning 20 poses between two known-free test locations |
| Dashboard | Live RGB, map, telemetry and stop-state display inspected in browser |
| Dashboard endpoints | `/api/state`, `/camera.jpg`, `/map.png` returned HTTP 200 |
| Existing-dashboard RGB topic | `/camera/image_raw/compressed` received live JPEG messages |
| Preservation | SHA-256 verified all 198 preexisting source files unchanged and present |

The final stationary robot-stack map was 161×122 cells at 0.05 m resolution,
with 1,204 occupied and 2,337 free cells. Mapping and reload tests used
`enable_motors:=false`. Wheel RPM was zero and battery readings were simulated;
camera, depth and IMU data were real. The camera mount height was the documented
example, not a measured robot installation. The test map is therefore a bench-test
artifact, not a map to navigate with on the real robot.

The plan-only action did not execute a navigation goal or command motion.
No Arduino firmware was flashed and no real motor serial device was opened.
Full robot tests used isolated ROS domain 73 with Fast DDS; unit tests used
domain 217. This did not change the system's ROS middleware configuration.

## Software tests

The suite checks map preservation, command selection, speed limits, stale command
and sensor stops, emergency-stop behavior, PWM conversion, wheel odometry,
serial telemetry parsing and serial watchdog output. The pseudo-terminal test
uses the real serial reader/writer against a simulated serial endpoint.

Run after building and sourcing the package:

```bash
cd ~/ros2_ws/src/oak_depth_navigation
source scripts/env.sh
python3 -m pytest -q test
```

PC-only integration evidence is retained under `.local/robot_test/`:
`mapping_check.json`, `navigation_check.json`, `navigation_states.json`,
`planning_check.json`, launch logs, database and 2D export. Earlier camera-only
maps remain under `maps/`. These files and private dependency downloads are
excluded from the portable archive.

## Checks still requiring the actual robot

- Run the dependency installer and build on the Jetson's actual Ubuntu/Jazzy image.
- Set the Arduino device path and measured camera mount in `config/robot.yaml`.
- Verify forward/turn directions, encoder signs, physical stop and command timeout
  with wheels raised; then calibrate open-loop PWM speed and odometry on the floor.
- Map with the camera rigidly mounted and check repeated loop closures, saved-map
  recognition, obstacle coverage and low-speed goal execution.
- Measure Jetson CPU load, USB stability and thermal performance while driving.

The forward camera cannot observe obstacles behind or beside the robot at all
times. LiDAR is optional for this package but can provide wider coverage.

## USB naming script follow-up

Added `scripts/setup_usb_udev.py` and **18 passing tests** for serial/port matching,
composite-camera indices, duplicate serials, invalid names and rule injection,
existing-file/symlink preservation, alias conflicts, OAK boot permissions,
targeted reload/verification and dry-run behavior. Run them without ROS or root:

```bash
python3 -m pytest -q test/test_usb_udev.py
```

Actual USB discovery and interactive dry-run previews were checked against this
PC's connected OAK and Brio webcam. No system rules, group memberships or device
permissions were changed. Privileged installation/reload was exercised with
mocked commands in tests; actual activation is performed when you run the script
and accept its displayed rule. The updated source archive is
`dist/oak_depth_navigation_jetson_usb_setup.tar.gz`; the earlier archive is retained.


## DWM dashboard integration — 26 September 2026

- Local Humble package build: passed.
- Python and both dashboard JavaScript syntax checks: passed.
- Dashboard-only launch in localhost-only ROS domain 217: HTTP server, rosbridge
  and rosapi started successfully; no camera or motor nodes launched.
- Browser: DWM default page rendered, connected to the bridge and listed topics;
  OAK Navigation opened in-page and fetched state; returning restored DWM.
- Tab-switch checks: held DWM input stops and keyboard state clears; leaving OAK
  calls its release handler and unloads the iframe; switching while idle sends
  no DWM velocity command. Host and IPv6 bridge URL defaults checked.
- No live camera, mapping, navigation, or motor movement tested in this change.
  Jetson deployment still needs the updated package and rosbridge-suite dependency.


## Developer dashboard depth tab — 26 September 2026

- Added Depth Navigation alongside User and Developer in the source developer
  dashboard and its independent OAK package copy.
- Browser verified the themed RGB/map/control panel, state connection, and return
  to the existing Developer tools. No robot hardware was started.
- Original classic dashboard and standalone navigation endpoints retained.
- Fixed closing the uninitialized Files & Terminal panel during tab changes.


## Native dashboard and launch modes — 26 September 2026

- Native Depth Navigation markup and scoped JavaScript in both developer files;
  no iframe remains. Classic dashboard pages retained unchanged.
- 16 targeted tests passed: mode construction, single odometry TF authority,
  manual navigation rejection, command watchdogs, map preservation, terminal
  backend, and authenticated route-limited OAK proxy.
- Local Humble build passed. Manual launch started in domain 217 with motor
  hardware explicitly disabled, localhost dashboard and no camera/SLAM/Nav2/RViz.
- Browser confirmed native telemetry and disabled navigation controls in manual
  mode; an emergency-stop click reached the isolated command mux.
- Mapping and navigation launch construction was checked without starting robot
  perception. No camera/motor/Jetson deployment or real navigation test performed.
