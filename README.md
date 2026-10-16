# pistar_car

ROS 2 Jazzy base stack for the Pi Star Lab RC car.

## Topics
| Topic | Type | Source |
|---|---|---|
| /cmd | ackermann_msgs/AckermannDrive | teleop or lane_follower |
| /camera/image_raw | sensor_msgs/Image | csi_camera, OV5647 ribbon cam (optional) |
| /imu/data_raw | sensor_msgs/Imu | imu_node (planned) |
| /wheel/speed | std_msgs/Float32 | encoder_node (planned) |
| /lane_follower/debug | sensor_msgs/Image | lane_follower (mask, path, target) |

## Drive with live feed
`ros2 launch pistar_car car.launch.py use_camera:=true`, then open
`http://<pi>:8080/stream?topic=/camera/image_raw` (or `/lane_follower/debug`).

## Record training data
Add `record:=true` (with `use_camera:=true`): `/camera/image_raw` and `/cmd`
go to `~/bags/drive_<timestamp>/`. Inspect with `ros2 bag info`, replay with
`ros2 bag play`, read in Python with `rosbag2_py`.

## Autonomous (camera pure pursuit)
`ros2 launch pistar_car car.launch.py use_camera:=true auto:=follower` (or `auto:=policy`)
Starts in MANUAL (sticks drive). Square on the pad toggles AUTO, where
the `auto` node drives; square again returns to manual. The pad must stay
connected in AUTO: if it drops, the car stops and returns in manual. The
follower steers to the centroid of the dark track and stops when it loses it.
Measure the camera block of `lane_follower` in `config/car.yaml` first, on a
stand, watching `/lane_follower/debug`.

## Learned policy
On the Pi: `python3 training/bag_to_npz.py ~/bags/drive_* --out ~/datasets`.
On a machine with torch: `python3 training/train.py datasets/*.npz --out training/out/run`
(needs `litert-torch` + `ai-edge-litert`, camera_setup's pins). Copy `policy.tflite` to the Pi's `~/models/` and
launch with `auto:=policy`. `metrics.json` scores it against the geometric
follower on held-out frames. The Pi needs `ai-edge-litert` to run it.

## Adding a sensor
1. Add `pistar_car/<sensor>_node.py`.
2. Register it in `setup.py` under `console_scripts`.
3. Add its parameter block to `config/car.yaml`.
4. Add a `use_<sensor>` argument and Node in `launch/car.launch.py`.
