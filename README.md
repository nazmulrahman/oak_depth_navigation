# OAK-D Pro + DWM robot: Jetson setup and operation

A standalone ROS 2 package for your **Jetson Orin Nano / Ubuntu 24.04 / ROS 2 Jazzy**,
OAK-D Pro, and existing DWM Arduino Nano motor/encoder firmware. It also supports
Humble on Ubuntu 22.04 for testing on this PC. No existing robot package is needed.

Included: DWM serial/PWM motor control, wheel odometry, OAK RGB/depth/IMU driver,
wheel + OAK gyro EKF, RTAB-Map mapping/localization, Nav2, map export, and a local
web dashboard with live color video, map, hold-to-drive controls and navigation goals.
**LiDAR and a separate IMU are not required.**

The mapping/navigation workflow uses wheel + IMU odometry, so loss of visual odometry does not
interrupt local motion estimation. RTAB-Map still needs recognizable visual features
for loop closures and saved-map localization. A forward camera has side/rear blind
spots; LiDAR is useful if you later want wider obstacle coverage.

## One-terminal operation

After building, source once in the terminal on the robot:

```bash
source ~/ros2_ws/src/oak_depth_navigation/scripts/env.sh
```

Choose **one** mode below. Each launch starts its own dashboard HTTP server,
rosbridge, motor bridge, command watchdog, and wheel odometry. Do not run an
additional DWM bringup, dashboard launch, or rosbridge alongside these defaults.

```bash
# Lightweight manual control and telemetry; no camera, EKF, SLAM, Nav2 or RViz.
ros2 launch oak_depth_navigation robot_manual.launch.py

# Build a new map while driving manually; camera + wheel/OAK EKF + RTAB-Map.
ros2 launch oak_depth_navigation robot_mapping.launch.py map_name:=room_01

# Localize and navigate using an existing saved map; adds Nav2.
ros2 launch oak_depth_navigation robot_navigation.launch.py map_name:=room_01
```

Stop the running launch with **Ctrl+C** before switching modes. Mapping flushes
its RTAB-Map database on clean shutdown; use a new map name for each new mapping
session. Navigation requires that saved `.db`. All modes are headless by default:
no RViz or browser window is launched on the robot. Mapping/navigation optionally
accept `rviz:=true`; manual mode intentionally has no visualization option.

On the PC, open `http://JETSON_IP:8080` in a browser. Click **CONNECT** for the
User/Developer telemetry and manual controls, or use the native **Depth Navigation**
tab. No extra terminal is needed for the web server or ROS bridge. Manual mode
publishes wheel-only `/odom` and its TF, requires no OAK device, and rejects
navigation-mode requests. Camera and map displays are unavailable in that mode.
Optional Arduino IMU telemetry is passed through when present, not fused.

Override the serial device if necessary with `serial_port:=/dev/dwm_nano`.
`enable_motors:=false` is a hardware-free bench option and does not produce real
wheel feedback. The firmware/command watchdogs remain active in every mode.

## 1. Transfer the package

The latest portable archive is `dist/oak_depth_navigation_native_modes_ros2_ws.tar.gz`.
Earlier archives are retained; use the latest archive named above. The latest archive contains source,
configuration, firmware reference, dashboard, scripts and tests. It excludes the PC's
x86 dependency downloads, build files, private camera snapshots and test databases.
Copy the archive to the Jetson using SCP, USB storage or your usual transfer method.
On the Jetson, use **`~/ros2_ws`** as your workspace. The package will be at
`~/ros2_ws/src/oak_depth_navigation`. Existing packages in `src/` can stay in place:

```bash
mkdir -p ~/ros2_ws/src
# Run only if ~/ros2_ws/src/oak_depth_navigation does not already exist.
tar --keep-old-files -xzf ~/Downloads/oak_depth_navigation_native_modes_ros2_ws.tar.gz -C ~/ros2_ws/src
cd ~/ros2_ws/src/oak_depth_navigation
```

## 2. Confirm OS and install dependencies

```bash
bash scripts/check_platform.sh
bash scripts/install_dependencies.sh
```

The installer detects the actual Ubuntu release: **24.04 installs Jazzy**, and
**22.04 installs Humble**. It installs ROS base, DepthAI **v2** (`depthai_ros_driver`),
RTAB-Map, Nav2, robot_localization, Madgwick IMU filter, image libraries, RViz,
keyboard teleop and colcon. Do not substitute the differently configured `_v3` driver.
The script uses the official ROS apt repository and native ARM64 packages on Jetson.
It does not upgrade JetPack, remove packages, edit your existing workspace, or modify
`.bashrc`. An existing OAK udev rules file is preserved.

Log out and log in after installation so `dialout` and `plugdev` membership takes
effect, then reconnect the camera and Arduino. ROS is sourced by `scripts/env.sh`.
No CUDA build, Isaac ROS container, ROS 1 bridge or paid SLAM component is required.
The OAK computes stereo depth on-device; RTAB-Map/Nav2 run on the Jetson CPU.

## 3. Connect and configure the actual robot

Connect:

- OAK-D Pro to Jetson using a USB 3 data cable; mount it rigidly, facing forward.
- Existing Arduino Nano motor controller to Jetson USB.
- The existing motor battery/drivers/encoder wiring and a suitable Jetson supply.
- OAK supported external power if USB power is insufficient. Keep its lenses clear.

The included `firmware/dwm_firmware.ino` is copied from your existing DWM package.
**No firmware flashing was performed.** If that firmware is already loaded, retain it.
It uses 115200 baud: `M,left_pwm,right_pwm`, `R,0/1`, and `W,left_rpm,right_rpm` feedback.
Battery and BNO055 telemetry are accepted, but the BNO055 is not used by the new EKF.
Negative PWM is forward for your existing wiring. The firmware has its own one-second
command timeout; the ROS bridge and command selector add shorter watchdogs.

Create custom USB device names automatically (no ROS environment needed):

```bash
python3 scripts/setup_usb_udev.py
```

The script requests sudo, lists connected USB devices with manufacturer/product,
VID:PID, serial number, USB port and device nodes, then asks which device/node to
name. Enter `dwm_nano` for the Arduino serial endpoint to create `/dev/dwm_nano`.
It previews the rule, saves a new `/etc/udev/rules.d/99-oak-usb-NAME.rules`, adds
the invoking user to the appropriate device group if needed, reloads udev,
triggers the selected device and checks the resulting symlink. Repeat for other
devices in the same session. Log out/in if it adds group membership.

Existing files and conflicting aliases are preserved; choose another name if an
alias is already owned by an existing rule. The script does not change robot.yaml.
Serial-number matching follows a device between USB ports. If a serial number is
missing or duplicated among connected devices, it uses a unique USB-port identity;
keep that device on the same port/hub. Avoid connecting future devices that reuse
the same serial number, or select port matching for those devices.

For an OAK, `oak_camera` names the **raw USB device**, not a video stream. The
script also grants the `plugdev` group access to all vendor `03e7` boot/runtime
devices, since OAK USB identity changes during firmware loading. The custom alias
can disappear when that identity changes; DepthAI does not require the alias.
Use `/camera/image_raw/compressed` for the dashboard video.

Read-only discovery and interactive preview are available:

```bash
python3 scripts/setup_usb_udev.py --list
python3 scripts/setup_usb_udev.py --dry-run
```

The script supports USB serial nodes, V4L camera nodes and raw USB nodes; it does
not rename network interfaces or disks. Close applications using a selected
device before applying rules. If reload or verification fails, the saved rule is
retained and the error is shown; reconnect the device and inspect `/dev/NAME`.
Rule behavior follows the [udev manual](https://man7.org/linux/man-pages/man7/udev.7.html)
and [udevadm manual](https://man7.org/linux/man-pages/man8/udevadm.8.html).

Alternatively, find the Arduino's existing stable name, then configure the robot:

```bash
ls -l /dev/serial/by-id/
nano config/robot.yaml
```

Set `serial_port` to its `/dev/serial/by-id/...` path, or use `/dev/dwm_nano` if your
existing udev alias is present. Auto-selecting a USB port is deliberately not used.

**Measure and enter `camera_mount` in `config/robot.yaml`.** Its supplied height of
0.50 m is an example, not a measurement. The reference frame is `base_footprint`:
the ground point below the sprocket axle midpoint, X forward, Y left, Z up. Supply
XYZ in metres and roll/pitch/yaw in radians; a level, forward-facing camera uses
zero angles. OAK optical/IMU extrinsics are supplied by the camera driver.

Existing DWM geometry is supplied: wheel radius 0.095 m, separation 0.430 m,
footprint from X=-0.50 to +0.45 m and Y=±0.25 m. Change these if the chassis changed.
The Nav2 footprint is in both `config/nav2.yaml` (Humble) and `config/nav2_jazzy.yaml`.
Motor direction, RPM sign and open-loop PWM/kick settings must match the actual
wiring; do a brief wheels-raised direction/stop check before floor driving.

## 4. Build

```bash
cd ~/ros2_ws/src/oak_depth_navigation
bash scripts/build.sh
source scripts/env.sh
```

Build/install/log output stays in this package's `.local/` directory. Existing
workspace build/install files are untouched. Source `scripts/env.sh` in every new
terminal. On Jetson this uses Jazzy; on the current PC it uses Humble and private
local dependencies. Never copy the PC's `.local` folder to the Jetson.

## 5. Map while driving manually

```bash
source ~/ros2_ws/src/oak_depth_navigation/scripts/env.sh
ros2 launch oak_depth_navigation robot_mapping.launch.py map_name:=room_01
```

This starts motors, camera/IMU, EKF, mapping and dashboard. Keep the robot still
for about 10 seconds while the camera and IMU initialize. It creates
`~/oak_maps/room_01/rtabmap.db`. Existing mapping databases are refused, never erased.
Choose a new map name for another session. `database:=/absolute/path/new.db` is also
supported. Do not run the old DWM bringup or another serial/odometry stack alongside it.

Find the Jetson's address with `hostname -I`. On your PC/phone on the same LAN open:

```text
http://JETSON_IP:8080
```

Choose **OAK Navigation**. Hold the arrow buttons to drive; release to stop. The center square sends zero.
Emergency Stop latches the stop; Release Stop clears it and leaves manual mode.
Drive slowly around the room with overlapping views of stationary, textured objects.
Avoid sudden turns, covering the lenses, or close moving people filling the image.
The map updates automatically. Map the actual robot with its final camera mounting.
A handheld map has a different base-frame relationship and is not the robot's map.

Keyboard alternative, from a second terminal on the Jetson:

```bash
source ~/ros2_ws/src/oak_depth_navigation/scripts/env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.12 -p turn:=0.3
```

Maximum accepted manual speed starts at 0.15 m/s and 0.40 rad/s; Nav2 starts at
0.12 m/s and 0.35 rad/s. PWM speed is open-loop: calibrate it on the real robot.
Losing command updates or wheel telemetry stops commanded motion. Navigation also
requires fresh depth and odometry. A software stop does not replace a physical stop.

## 6. Save the map, then stop cleanly

While mapping is still running, in another sourced terminal:

```bash
ros2 run oak_depth_navigation save_map ~/oak_maps/room_01/export
```

This creates `export/map.yaml` and `export/map.pgm`. The export directory must be new;
use `export_02` for another snapshot. Then press **Ctrl+C in the mapping terminal**
and wait for `Saving database/long-term memory...done!`. This flushes the visual map
and features to `~/oak_maps/room_01/rtabmap.db`. Do not cut Jetson power during saving.

Keep the complete room directory. The `.db` is required for camera localization;
a 2D YAML/PGM alone is not sufficient. The 2D export is useful for inspection and
other navigation stacks; this package's Nav2 gets `/map` from RTAB-Map.

## 7. Navigate with the saved map

```bash
source ~/ros2_ws/src/oak_depth_navigation/scripts/env.sh
ros2 launch oak_depth_navigation robot_navigation.launch.py map_name:=room_01
```

Start in a previously mapped area, with the same mounting configuration. Wait for
recognition/localization and `Managed nodes are active`. The source database is
copied to a fresh localization session under `~/.ros/oak_depth_navigation/sessions`,
so the original mapping database stays unchanged.

Open the dashboard’s **OAK Navigation** tab, enable **click-to-set navigation goal**, then click a known-free
map area. The dashboard enables navigation mode and sends a Nav2 goal. The final
heading is map +X; RViz's Nav2 Goal tool lets you choose the arrival heading instead.
Use `rviz:=true` on a machine with a display, or run RViz on another Jazzy PC on the
same ROS domain/network. Jazzy and Humble should not be mixed in one deployment.

Manual input takes control and keeps navigation output blocked until Navigation
mode is explicitly selected again. Emergency Stop cancels a dashboard goal and
blocks motors/relay; Release Stop does not automatically resume motion. If a goal
was issued in RViz, cancel it there before resuming when appropriate.

If the robot cannot recognize its location, face a previously mapped view and move
slowly under manual control. RViz's 2D Pose Estimate publishes `/initialpose` if an
approximate starting pose is needed. Do not drive an unlocalized robot autonomously.

## 8. Camera feed and dashboard integration

The default page at `http://JETSON_IP:8080` is now a bundled copy of
`dwm_dashboard_developer.html`, with **User**, **Developer**, and **Depth Navigation**
tabs. The Depth Navigation tab matches the developer theme and includes RGB,
occupancy map, mode/speed/battery status, held-button driving, emergency stop,
and click-to-set Nav2 goals. Its connection uses the OAK HTTP server independently
of the ROS websocket connection used by the other tabs.

The controls, map canvas and scripts are native to `dwm_dashboard_developer.html`;
there is no iframe or other dashboard page loaded by this tab. The backend supplies
only telemetry, image data, and ROS command handling. The same native integration
is in the source developer dashboard in `dwm_bringup/dashboard`.

Files & Terminal remains optional and separate from robot operation. If you want
that PC-side tool as well, run its existing server with
`--oak-url http://JETSON_IP:8080`. Its limited data proxy connects the native tab
to OAK without embedding another page. Without Files & Terminal, simply open the
robot's dashboard URL; no PC-side server is needed.

The previous dashboard remains at `/classic`, its standalone OAK view remains
at `/navigation`, and the matching developer-style OAK view is at
`/depth-navigation`. Switching tabs releases held drive input, disables the
hidden view's goal checkbox, and pauses its image/state polling. Existing
navigation goals continue until stopped; switching tabs while idle does not
send a velocity command.

The developer dashboard retains its original CDN dependencies for styling,
fonts, icons, ROSLIB and terminal rendering. The previous standalone OAK views
remain available without CDN dependencies. Camera and map data require the running OAK/robot stack.

All three robot launches start the dashboard and rosbridge automatically. Use
`dashboard:=false` to disable both, or `rosbridge:=false` if a bridge already
runs on port 9090. Install the new dependency on the Jetson before rebuilding:

```bash
sudo apt install ros-jazzy-rosbridge-suite
bash scripts/build.sh
```

To run just the dashboard and bridge with an existing OAK stack:

```bash
ros2 launch oak_depth_navigation dashboard.launch.py
```

The dashboard node publishes the RGB view as `/camera/image_raw/compressed`
(`sensor_msgs/msg/CompressedImage`, JPEG, up to 8 Hz, 800-pixel width), OAK IMU
as `/imu/data`, and reads `/odom` from the wheel/OAK EKF. Keep this node running
for the DWM camera and IMU compatibility topics. OAK provides no LiDAR scan;
that original tab needs an actual scan publisher.

Direct data URLs are `/camera.jpg`, `/map.png`, and `/api/state`. Ports 8080 and
9090 are intended for your trusted robot LAN; they have no user login.

## Frames and topics

```text
map --RTAB-Map--> odom --EKF--> base_footprint
                                |--robot_state_publisher--> base_link / tracks
                                |--DepthAI driver----------> oak / optical / IMU
```

In mapping/navigation, the wheel node publishes `/wheel/odom` with TF disabled. The EKF fuses wheel forward
velocity/yaw rate and OAK gyro Z, publishing `/odom` and the single odom-to-base TF.
RTAB-Map consumes RGB/depth and EKF odometry; Nav2 uses `/oak/points` in voxel costmaps.
Absolute IMU yaw and acceleration are not integrated as position measurements.
Covariances are provisional and should be tuned from robot logs.

| Topic | Purpose |
|---|---|
| `/cmd_vel` | Manual input (Twist), also compatible with existing dashboard |
| `/oak/nav_cmd_vel` | Smoothed Nav2 input to command selector |
| `/motor/cmd_vel` | Selected/clamped command to PWM converter |
| `/wheel_cmd_pwm`, `/wheel_rpm` | Existing DWM serial protocol bridge |
| `/emergency_stop` | Latched software stop command, std_msgs/Bool |
| `/oak/control_status` | JSON control status |
| `/oak/rgb/image_rect`, `/oak/stereo/image_raw` | RGB and aligned metric depth |
| `/oak/imu/filtered` | OAK gyro/accel with filtered orientation |
| `/camera/image_raw/compressed` | Dashboard color feed |
| `/map`, `/oak/cloud_map` | 2D occupancy / assembled depth cloud |

## Diagnostics and optional PC camera-only use

```bash
ros2 run oak_depth_navigation check_mapping --base-frame base_footprint --odom-topic /odom --seconds 30
ros2 topic hz /wheel_rpm
ros2 topic hz /camera/image_raw/compressed
ros2 lifecycle get /controller_server
```

No wheel messages: check serial path, permissions, firmware and Nano USB connection.
No camera: check USB 3 cable/power and `lsusb`; do not run another camera viewer.
No map: check RGB/depth, wheel odometry, OAK IMU and TF. No navigation: wait for
localization and active lifecycle states; check that the goal lies in free space.
If CPU load is high, keep RViz off on Jetson and view the dashboard remotely first.
Do not reduce image sizes independently: RGB and aligned depth must match.

For PC handheld mapping without any motor controller:

```bash
ros2 launch oak_depth_navigation mapping.launch.py
# Reopen a completed handheld map without motors:
ros2 launch oak_depth_navigation mapping.launch.py localization:=true database:=/path/to/saved.db
```

These use visual odometry and `oak_mount` as the base frame. Use the `robot_*` launch
files on your robot. `enable_motors:=false` on a robot launch omits the serial bridge;
without wheel telemetry it is a bench test, not a working moving-robot configuration.

RViz now defaults to **Accumulated 3D color map** on `/oak/cloud_map`, with fixed
frame `map` and an Orbit view. Drag the left mouse button to orbit, use the middle
button to pan, and scroll to zoom. Move the camera slowly to add overlapping views.
The optional **Live depth** display shows only the current camera cloud; enable
**Depth occupancy map** when you want the 2D navigation projection. To update an
already open RViz window without restarting mapping, use File → Open Config and
select this package's `rviz/mapping.rviz`. Keep the RTAB-Map `.db` for the 3D map;
the YAML/PGM export contains only the 2D occupancy map.

Camera-only launches default to `responsive_view:=true`: a colored live cloud on
`/oak/live_color_cloud`, shorter synchronization queues, a 4 Hz SLAM processing
target, and no constant-motion extrapolation in visual odometry. Actual accumulated
cloud publication depends on mapping activity and computation time. Robot launches
default to `responsive_view:=false` to retain their original processing budget.
The live preview is the current depth surface, not additional saved map geometry.

On this PC, RViz rendering uses the RTX 4060 Ti, while the installed RTAB-Map/OpenCV
C++ binaries perform visual odometry and SLAM on CPU; they do not link CUDA modules.
The camera computes stereo depth onboard. Enabling a GPU feature flag alone cannot
convert these binaries into a CUDA build; see the
[RTAB-Map GPU FAQ](https://github.com/introlab/rtabmap/wiki/FAQ).
The 24 September live test measured approximately 14.5 Hz live preview with 105 ms
median message age, 147 ms odometry age, and 292 ms accumulated-map age. These are
ROS message ages at receipt, not measured screen-to-photon latency.
If tracking is lost, stop moving and return to a previously viewed textured scene.
GPU rendering cannot recover missing visual correspondences. The display is a
colored point cloud rather than a filled, textured mesh.

## Verification and sources

See `TEST_RESULTS.md` for exactly what was tested. Real OAK streams and mapping were
tested on this Humble PC; motors were tested against a pseudo-terminal, never moved.
The Jetson ARM64/Jazzy build and moving-robot calibration still require your hardware.

- [NVIDIA JetPack and supported platforms](https://developer.nvidia.com/embedded/jetpack)
- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [DepthAI ROS Jazzy v2 source](https://github.com/luxonis/depthai-ros/tree/jazzy)
- [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros/tree/ros2)
- [Nav2 Jazzy configuration](https://docs.nav2.org/jazzy/configuration_and_development/)
