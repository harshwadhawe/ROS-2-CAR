# Reference

## Topics
| Topic | Type | From -> to |
|---|---|---|
| /joy | sensor_msgs/Joy | joy_node -> teleop_joy |
| /drive | ackermann_msgs/AckermannDriveStamped | lane_follower or policy -> teleop_joy, forwarded in AUTO |
| /ackermann_cmd | ackermann_msgs/AckermannDriveStamped | teleop_joy -> actuator_node (the only publisher) |
| /camera/image_raw | sensor_msgs/Image | csi_camera, 160x120 bgr8 at 20 Hz |
| /lane_follower/debug | sensor_msgs/Image | lane_follower: track (blue), lookahead arc (green), target |

These are F1TENTH's names and type: any controller that publishes
`AckermannDriveStamped` on `/drive` (F1TENTH pure pursuit, gap follow, ...)
drives this car in AUTO unchanged. `teleop_joy` stands where F1TENTH's
`ackermann_mux` does, and `actuator_node` where its VESC driver does.
`/drive` messages carry the stamp of the camera frame they were computed from.
Bags from before the switch (2026-10-15 22:39-22:49) have `/cmd`
(`AckermannDrive`); `bag_to_npz.py` reads both.

Steering is radians, positive = left (REP-103). Speed is m/s nominal and
open-loop: it drops as the battery drains.

## Why csi_camera and not camera_ros
The apt `ros-jazzy-libcamera` expects hyphenated rp1-cfe entity names; Ubuntu's
6.8 raspi kernel has underscores (`Unable to acquire a CFE instance`).
`csi_camera` / `ov5647.py` read raw Bayer off the CFE with `v4l2-ctl` and
debayer in OpenCV, with exposure, gain and white balance done on the sensor.
Frame rate is set on the sensor (`fps`, via vertical blanking).

## Why the road centroid
On the old taped track the tape was ~26 px and 85% of frames showed one edge,
so edge-pair detectors had nothing to pair (sliding windows r=+0.15 vs road
centroid r=+0.52 against human steering, camera_setup notebooks). On the
current track, connectivity to the floor ahead separates the track from the
equally dark grey carpet (`lane_geometry.track_region`).

## Adding a sensor
1. Add `pistar_car/<sensor>_node.py` (ROS wiring) and, if it has real logic, a ROS-free module next to it.
2. Register the node in `setup.py` under `console_scripts`.
3. Add its parameter block to `config/car.yaml`.
4. Add a `use_<sensor>` argument and Node in `launch/car.launch.py`.
