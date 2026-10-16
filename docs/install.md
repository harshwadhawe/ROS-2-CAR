# Install on the Pi

Tested: Raspberry Pi 5, Ubuntu 24.04 (6.8 raspi kernel), ROS 2 Jazzy. See the
[prerequisites](../README.md#prerequisites) first.

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
