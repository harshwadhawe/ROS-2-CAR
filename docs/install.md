# Install on the Pi

Tested: Raspberry Pi 5, Ubuntu 24.04 (6.8 raspi kernel), ROS 2 Jazzy. See the
[prerequisites](../README.md#prerequisites) first.

## 0. Boot config (camera and I2C)
Ubuntu's Pi image enables both by default. Check `/boot/firmware/config.txt` has
```
camera_auto_detect=1
dtparam=i2c_arm=on
```
then reboot if you changed it. The camera needs **nothing else**: no libcamera
build, no `camera_ros` (see [libcamera: do not install](#libcamera-do-not-install)).

## 1. ROS 2 and system packages
Install ROS 2 Jazzy from the official Ubuntu (deb) guide; `ros-jazzy-ros-base` is enough. Then:
```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-pytest python3-yaml \
    python3-opencv python3-smbus2 python3-pip v4l-utils i2c-tools \
    ros-jazzy-ackermann-msgs ros-jazzy-joy ros-jazzy-cv-bridge \
    ros-jazzy-web-video-server ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap
sudo usermod -aG i2c,video,input $USER     # log out and back in afterwards
```
A 404 on a ROS package means a stale index: `sudo apt update`, retry.

TFLite runtime for the learned policy. The Pi's Python is system-managed:
install for your user, and pin numpy below 2 so ROS's compiled numpy is never
replaced (pip refuses rather than breaking it):
```bash
python3 -m pip install --user --break-system-packages ai-edge-litert==2.2.0 "numpy<2"
python3 -c "import numpy, ai_edge_litert; print(numpy.__version__)"   # must print 1.26.x
```

### What each package is for
| Package | Used by |
|---|---|
| `python3-smbus2`, `i2c-tools` | `pca9685.py` (PWM board), the launch's all-outputs-off, `i2cdetect` check |
| `v4l-utils` (`media-ctl`, `v4l2-ctl`) | `ov5647.py`: camera routing, format, sensor controls, frame stream |
| `python3-opencv` | debayer, track mask, resizing (`ov5647`, `lane_geometry`, nodes) |
| `ros-jazzy-cv-bridge` | Image messages <-> numpy in every camera node |
| `ros-jazzy-ackermann-msgs` | `/drive`, `/ackermann_cmd` |
| `ros-jazzy-joy` | `joy_node`, the PS4 pad |
| `ros-jazzy-web-video-server` | live feed in a browser (`use_camera:=true`) |
| `ros-jazzy-rosbag2`, `-storage-mcap` | `record:=true` and `training/bag_to_npz.py` |
| `python3-yaml` | `bag_to_npz.py` reads `config/car.yaml` |
| `python3-pytest`, `python3-colcon-common-extensions` | build and `colcon test` |
| `ai-edge-litert` (pip) | `policy_node` only; skip it if you never use `auto:=policy` |

### libcamera: do not install
Nothing here uses libcamera, and it cannot help on this OS: the apt
`ros-jazzy-libcamera` (pulled in by `ros-jazzy-camera-ros`) expects
hyphenated rp1-cfe names that Ubuntu's 6.8 raspi kernel does not have, and
fails with `Unable to acquire a CFE instance` / `no cameras available`.
`csi_camera` reads the sensor directly instead ([reference](reference.md#why-csi_camera-and-not-camera_ros)).

**Do not** install or build `camera_ros`, `ros-jazzy-libcamera`, Raspberry
Pi's libcamera fork, `rpicam-apps`, or `ros-jazzy-v4l2-camera` for this
camera. If they are already installed (from apt) they are harmless as long as
**nothing else opens the camera while `csi_camera` runs**.

### Conflicts
| Conflict | Symptom | Fix |
|---|---|---|
| Another program holds the camera (a second launch, `camera_ros`, `rpicam`/`cam`, a desktop PipeWire libcamera plugin) | `VIDIOC_S_FMT: Device or resource busy`, or no frames | Find it: `sudo fuser -v /dev/video0 /dev/media0`; stop it. `csi_camera` resets the routing on every start, so restart it after |
| A `camera_ros` built from source in this workspace | It shadows apt's, links a libcamera that may be gone; plain `colcon build` rebuilds it | Remove it, see [Cleaning up](#cleaning-up-an-earlier-libcamera-attempt) |
| `colcon build` without `--packages-select` | Builds every package in `src/`, including leftovers | Always `--packages-select pistar_car` |
| pip upgrading numpy to 2.x in `~/.local` | `cv_bridge` / ROS Python crashes with a numpy ABI error | `python3 -m pip uninstall --user numpy`; reinstall with `"numpy<2"` |
| Old `/ackermann_cmd` or `/drive` publishers from a crashed run | Car twitches or two commanders fight | `pgrep -af pistar_car`, `kill -INT` each |

### Cleaning up an earlier libcamera attempt
This Pi has leftovers from building libcamera and `camera_ros` from source.
Nothing uses them; the `camera_ros` overlay actively gets in the way:
```bash
cd ~/ros2_ws
rm -rf src/camera_ros build/camera_ros install/camera_ros   # broken source-built overlay
rm -rf ~/libcamera-rpi-src ~/.local/libcamera-rpi ~/libcamera-build.log
colcon build --symlink-install --packages-select pistar_car  # refresh install/setup.bash
source install/setup.bash && ros2 pkg prefix camera_ros      # now /opt/ros/jazzy (apt), or not found
```
Optional, frees space, nothing depends on them:
`sudo apt remove ros-jazzy-camera-ros ros-jazzy-v4l2-camera`.

## 2. Code and build
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/harshwadhawe/ROS-2-CAR.git pistar_car
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pistar_car
echo 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon test --packages-select pistar_car && colcon test-result --all    # expect 0 failures
```
**Clone came from the old `pi` remote?** Point it at GitHub:
```bash
cd ~/ros2_ws/src/pistar_car
git remote set-url origin https://github.com/harshwadhawe/ROS-2-CAR.git
git pull --ff-only
```
**Updating later:** `cd ~/ros2_ws/src/pistar_car && git pull`, then the
`colcon build` line again, then relaunch.

## 3. Check the hardware
```bash
ls /dev/i2c-1 && i2cdetect -y 1 | grep -q " 40 " && echo "PCA9685 ok"
media-ctl -d /dev/media0 -p | grep -q ov5647 && echo "camera ok"
ls /dev/input/js0 && echo "pad ok"
vcgencmd get_throttled                    # 0x0 is good; see troubleshooting
```
Pair the PS4 pad once: hold **Share + PS** until the light flashes, then
`bluetoothctl`, and inside it: `scan on`, `pair <MAC>`, `trust <MAC>`, `connect <MAC>`, `quit`.

Next: [calibrate](calibrate.md).
