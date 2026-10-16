# pistar_car

ROS 2 Jazzy stack for the Pi Star Lab RC car: PCA9685 actuation, PS4 pad
teleop, the OV5647 ribbon camera, rosbag data collection, a geometric lane
follower and a learned (TFLite) steering policy, switched by **square** on the pad.

**Pipeline:** [install](docs/install.md) → [calibrate](docs/calibrate.md) →
[collect laps](docs/collect.md) → [train](docs/train.md) → [run](docs/run.md).
Stuck? [troubleshooting](docs/troubleshooting.md). Topics, design notes:
[reference](docs/reference.md).

## Prerequisites

**Car hardware**
| Part | Notes |
|---|---|
| Raspberry Pi 5, 4 GB+ | 32 GB+ microSD |
| PCA9685 PWM board | I2C bus 1, address 0x40; steering on ch 14, ESC on ch 1 |
| Steering servo + ESC + drive motor | endpoints copied from DonkeyCar's `myconfig.py` |
| OV5647 camera (Pi Camera v1.3 or clone) | on the CSI port, Pi 5 22-pin ribbon |
| PS4 DualShock 4 controller | Bluetooth, `hid-playstation` driver |
| **Separate 5 V / 5 A supply for the Pi** | sharing the ESC's battery browns it out and reboots it |
| A stand that lifts the wheels | for every first run of a change |

**Car software** (exact commands in [install](docs/install.md))
- Ubuntu 24.04 (Raspberry Pi image, kernel 6.8 raspi) with camera and I2C enabled in `config.txt` (the default).
- ROS 2 Jazzy (`ros-jazzy-ros-base`) plus: `ackermann-msgs`, `joy`, `cv-bridge`,
  `web-video-server`, `rosbag2` + `rosbag2-storage-mcap`.
- System: `python3-opencv`, `python3-smbus2`, `python3-yaml`, `v4l-utils`, `i2c-tools`, `git`, colcon, pytest.
- pip (user): `ai-edge-litert==2.2.0` with `numpy<2`, only for `auto:=policy`.
- Internet on the Pi for `apt`, `pip` and `git clone` (repo is public, no login).
- **Not** libcamera, `camera_ros` or `rpicam-apps`: they cannot see this camera on
  this kernel, and anything else holding the camera blocks `csi_camera`
  ([why, and cleanup](docs/install.md#libcamera-do-not-install)).

**Training machine** (Mac or Linux, not the Pi)
- Python 3.12+, ~3 GB free for the venv and data.
- GPU optional: Apple MPS or CUDA. 20 epochs ≈ 4 min on an M1 Pro.
- `ssh`/`scp` access to the Pi on the same network.

**Track**: a dark track on a lighter floor. The current one is black on beige
carpet, ringed by grey carpet; the follower relies on the lighter floor
separating the track from anything else dark.

## Quick start (car already installed and calibrated)
```bash
ros2 launch pistar_car car.launch.py use_camera:=true record:=true            # drive laps, recording
python3 training/bag_to_npz.py ~/bags/drive_* --out ~/datasets                # on the Pi
training/out/.venv/bin/python training/train.py training/out/datasets/*.npz --out training/out/run1   # training machine
scp training/out/run1/policy.tflite pistar@<pi-ip>:models/policy.tflite
ros2 launch pistar_car car.launch.py use_camera:=true auto:=policy            # square = AUTO
```

## Code map
| Path | What | ROS? |
|---|---|---|
| `pistar_car/actuator_node.py` | /cmd -> PCA9685 PWM, watchdog to neutral | node |
| `pistar_car/pca9685.py` | PCA9685 register driver | no |
| `pistar_car/teleop_joy.py` | pad -> /cmd; MANUAL/AUTO switch, forwards /auto_cmd | node |
| `pistar_car/pad_mapping.py` | stick scaling, square toggle | no |
| `pistar_car/csi_camera.py` | camera -> /camera/image_raw | node |
| `pistar_car/ov5647.py` | sensor setup, frame rate, Bayer unpack | no |
| `pistar_car/lane_follower.py` | geometric follower -> /auto_cmd, debug image | node |
| `pistar_car/lane_geometry.py` | floor projection, track region, pure pursuit | no |
| `pistar_car/policy_node.py` | TFLite policy -> /auto_cmd | node |
| `pistar_car/teleop_keyboard.py` | keyboard teleop (fallback) | node |
| `launch/car.launch.py` | one launch, all modes | - |
| `config/car.yaml` | every tunable, per node | - |
| `training/bag_to_npz.py` | bags -> npz (runs on the Pi) | rosbag2 |
| `training/data.py` · `model.py` · `export.py` · `train.py` | frame filter/split · network · tflite export · training loop | no |
| `test/` | pytest; runs on any machine: `PYTHONPATH=. python3 -m pytest test/` | no |
