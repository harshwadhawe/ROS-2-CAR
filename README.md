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

## Autonomous (camera pure pursuit)
`ros2 launch pistar_car car.launch.py use_camera:=true use_follower:=true`
Stops when no yellow tape is seen. Measure the camera block of
`lane_follower` in `config/car.yaml` first, on a stand, watching
`/lane_follower/debug` in `rqt_image_view`.

## Adding a sensor
1. Add `pistar_car/<sensor>_node.py`.
2. Register it in `setup.py` under `console_scripts`.
3. Add its parameter block to `config/car.yaml`.
4. Add a `use_<sensor>` argument and Node in `launch/car.launch.py`.
