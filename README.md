# pistar_car

ROS 2 Jazzy stack for the Pi Star Lab RC car: PCA9685 actuation, PS4 pad
teleop, the OV5647 ribbon camera, rosbag data collection, a geometric lane
follower and a learned (TFLite) steering policy.

Pipeline: **install on the Pi → calibrate → collect laps → train on a Mac/GPU
box → copy the model to the Pi → drive with square on the pad.**

---

## 1. Install on the Pi

Tested on a Raspberry Pi 5, Ubuntu 24.04 (kernel 6.8 raspi), ROS 2 Jazzy.

### 1.1 ROS 2 and system packages
ROS 2 Jazzy itself: follow the official Ubuntu (deb) install, `ros-jazzy-ros-base` is enough.
Then:
```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-pytest python3-yaml \
    python3-opencv python3-smbus2 python3-pip v4l-utils i2c-tools \
    ros-jazzy-ackermann-msgs ros-jazzy-joy ros-jazzy-cv-bridge \
    ros-jazzy-web-video-server ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap
sudo usermod -aG i2c,video,input $USER     # log out and back in afterwards
```
If `apt install` 404s on a ROS package, the index is stale: `sudo apt update` and retry.

The learned policy needs the TFLite runtime. The Pi's Python is
system-managed, so install it for your user, and pin numpy below 2 so ROS's
compiled numpy is never replaced (pip will refuse rather than break it):
```bash
python3 -m pip install --user --break-system-packages ai-edge-litert==2.2.0 "numpy<2"
python3 -c "import numpy, ai_edge_litert; print(numpy.__version__)"   # must print 1.26.x
```

### 1.2 Get the code and build
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
**Already have a clone from the old `pi` remote?** Point it at GitHub instead:
```bash
cd ~/ros2_ws/src/pistar_car
git remote set-url origin https://github.com/harshwadhawe/ROS-2-CAR.git
git pull --ff-only
```
**Updating later:** `cd ~/ros2_ws/src/pistar_car && git pull`, then rerun the
`colcon build` line. Edits to existing `.py` files apply without a rebuild
(symlink install); new nodes, launch or config files need it.

### 1.3 Check the hardware
```bash
ls /dev/i2c-1 && i2cdetect -y 1 | grep -q " 40 " && echo "PCA9685 ok"
media-ctl -d /dev/media0 -p | grep -q ov5647 && echo "camera ok"
ls /dev/input/js0 && echo "pad ok"
vcgencmd get_throttled                    # 0x0 is good, see Troubleshooting
```
Pair the PS4 pad once: hold **Share + PS** until the light flashes, then
```bash
bluetoothctl    # then: scan on, pair <MAC>, trust <MAC>, connect <MAC>, quit
```

**Power:** give the Pi its own 5 V / 5 A supply. Sharing the ESC's supply
browns it out (`throttled=0x50005`) and reboots it mid-drive.

---

## 2. Calibrate (once per car, wheels off the ground)

All in `config/car.yaml`. After editing, rerun the `colcon build` line from
1.2 and relaunch (launch reads the installed copy of the yaml).

| What | Keys | How |
|---|---|---|
| Servo/ESC endpoints | `actuator_node: steering_*_pwm, throttle_*_pwm` | Copy from DonkeyCar's `myconfig.py`; same 12-bit ticks |
| Speed cap | `actuator_node: throttle_limit` (now 0.25) | The car starts moving near 0.17 |
| Stick directions | `teleop_joy: steer_scale, throttle_scale` | Stick left must turn the wheels left; flip the sign if not |
| Full stick = cap | `teleop_joy: max_speed` | `throttle_limit x actuator max_speed` (0.25 x 2.0 = 0.5) |
| Mode button | `teleop_joy: mode_button` | 3 = square on hid-playstation; `ros2 topic echo /joy` to check |
| Camera geometry | `lane_follower: cam_height, cam_pitch_deg, cam_hfov_deg` | Mark the floor 0.45 m ahead; it should sit on the green arc in `/lane_follower/debug` |
| Track threshold | `lane_follower: road_v_max` | Track blue, carpet not, in `/lane_follower/debug` |

---

## 3. Collect data

```bash
ros2 launch pistar_car car.launch.py use_camera:=true record:=true
```
Watch the camera at `http://<pi-ip>:8080/stream?topic=/camera/image_raw`.
Drive laps with the pad (left stick steers, right stick throttle, centred =
stop). Press **Ctrl-C once** and wait ~2 s to stop; the bag closes cleanly.

Each run writes `~/bags/drive_<timestamp>/` with `/camera/image_raw` (160x120
at 20 Hz) and `/cmd` (the steering and speed actually sent to the car):
```bash
ros2 bag info ~/bags/drive_<timestamp>
```
**Good data** is what limits the policy. Aim for several minutes of smooth laps
in **both directions**, steering progressively rather than full lock, a few
deliberate drift-and-recover moments, and the lighting the car will drive in.
Off-track, stopped and reversing frames are filtered out at training, so
they cost nothing but are not learnt from.

Back up the bags; they exist only on the Pi:
```bash
scp -r pistar@<pi-ip>:bags ./training/out/      # from the training machine
```

---

## 4. Train (on a Mac or GPU machine, not the Pi)

**On the Pi**, turn bags into training arrays (frame, label, off-track flag,
and the geometric follower's steering as a baseline):
```bash
cd ~/ros2_ws/src/pistar_car
python3 training/bag_to_npz.py ~/bags/drive_* --out ~/datasets
```
**On the training machine**, from a clone of this repo:
```bash
git clone https://github.com/harshwadhawe/ROS-2-CAR.git pistar_car && cd pistar_car
scp -r pistar@<pi-ip>:datasets ./training/out/
python3 -m venv training/out/.venv
training/out/.venv/bin/pip install -r training/requirements.txt
training/out/.venv/bin/python training/train.py training/out/datasets/*.npz --out training/out/run1
```
(Already have torch in your base Python? `python3 -m venv --system-site-packages`
reuses it.) 20 epochs take ~4 min on an M1 GPU.

`training/out/run1/` then holds:

| File | What |
|---|---|
| `policy.tflite` | The model the Pi runs: `img_in` 1x120x160x3 raw RGB -> steering (rad) |
| `policy.pt` | PyTorch weights |
| `metrics.json` | Held-out scores vs baselines, and the tflite-vs-torch check |

Read `metrics.json` before deploying. `policy` should clearly beat
`baseline_geometric_follower` on `mae_rad` (lower) and `r` (higher). Validation
is the last 20% of each bag, so it is from the same session as training and
flatters the model; the track is the real test. The first model (2,702 usable
frames) scored MAE 0.076 rad, r +0.877, against the follower's 0.194 / +0.517.

What training keeps: a `/cmd` within 0.25 s, a human command (not AUTO),
moving forward, and track filling >= 15% of the frame. It fine-tunes ImageNet
MobileNetV2 with flip (steering negated) and brightness augmentation, and the
export fails loudly if the tflite disagrees with torch.

**Copy the model to the Pi:**
```bash
ssh pistar@<pi-ip> mkdir -p models
scp training/out/run1/policy.tflite pistar@<pi-ip>:models/policy.tflite
```

---

## 5. Run

| Command | AUTO is driven by |
|---|---|
| `ros2 launch pistar_car car.launch.py use_camera:=true` | nothing (manual only) |
| `... use_camera:=true auto:=follower` | geometric lane follower |
| `... use_camera:=true auto:=policy` | learned TFLite policy (`~/models/policy.tflite`) |

Add `record:=true` to any of them to keep collecting data.

- Starts in **MANUAL**. **Square** toggles **AUTO**; square again returns to manual.
- AUTO steers only; speed is fixed (`speed` in the `lane_follower` / `policy` blocks).
- The pad is the kill switch in both modes: if it drops, the car stops within
  ~0.75 s and comes back in manual. Both auto nodes also stop when they lose the track.
- First runs on a stand: hold the car off-centre, press square, check the
  wheels turn back towards the middle of the track.
- Watch what the follower sees at `http://<pi-ip>:8080/stream?topic=/lane_follower/debug`.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Device or resource busy` from the camera | An old run still holds it: `pgrep -af pistar_car`, then `kill -INT <pid>` |
| A launch crashed or hung | Same check; then zero the PWM outputs: `python3 -c "from smbus2 import SMBus; b=SMBus(1); b.write_byte_data(0x40, 0xFD, 0x10); b.close()"` |
| Pi reboots or drops off Wi-Fi | `vcgencmd get_throttled`: bit 0 (`...5`) = under-voltage now. Separate Pi supply |
| `package 'web_video_server' not found` | `sudo apt update && sudo apt install ros-jazzy-web-video-server` |
| `No module named ai_edge_litert` | Step 1.1, the pip line |
| Feed too dark/bright | `camera: auto_exposure: true` uses the sensor's own AE; set false and tune `analogue_gain` |

---

## Reference

### Nodes and topics
| Topic | Type | From -> to |
|---|---|---|
| /joy | sensor_msgs/Joy | joy_node -> teleop_joy |
| /cmd | ackermann_msgs/AckermannDrive | teleop_joy -> actuator_node (the only /cmd publisher) |
| /auto_cmd | ackermann_msgs/AckermannDrive | lane_follower or policy -> teleop_joy, forwarded in AUTO |
| /camera/image_raw | sensor_msgs/Image | csi_camera, 160x120 bgr8 at 20 Hz |
| /lane_follower/debug | sensor_msgs/Image | lane_follower: track (blue), lookahead arc (green), target |

Steering is radians, positive = left (REP-103); speed is m/s nominal,
open-loop (it drops as the battery drains).

### Why csi_camera and not camera_ros
The apt `ros-jazzy-libcamera` expects hyphenated rp1-cfe entity names; Ubuntu's
6.8 raspi kernel has underscores (`Unable to acquire a CFE instance`). So
`csi_camera` reads raw Bayer off the CFE with `v4l2-ctl` and debayers in
OpenCV, with exposure, gain and white balance done on the sensor. Frame rate is
set on the sensor (`fps`, via vertical blanking).

### Adding a sensor
1. Add `pistar_car/<sensor>_node.py`.
2. Register it in `setup.py` under `console_scripts`.
3. Add its parameter block to `config/car.yaml`.
4. Add a `use_<sensor>` argument and Node in `launch/car.launch.py`.
