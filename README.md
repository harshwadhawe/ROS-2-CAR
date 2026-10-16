# pistar_car

ROS 2 Jazzy base stack for the Pi Star Lab RC car.

## Topics
| Topic | Type | Source |
|---|---|---|
| /cmd | ackermann_msgs/AckermannDrive | teleop or policy |
| /camera/image_raw | sensor_msgs/Image | camera_ros (optional) |
| /imu/data_raw | sensor_msgs/Imu | imu_node (planned) |
| /wheel/speed | std_msgs/Float32 | encoder_node (planned) |

## Adding a sensor
1. Add `pistar_car/<sensor>_node.py`.
2. Register it in `setup.py` under `console_scripts`.
3. Add its parameter block to `config/car.yaml`.
4. Add a `use_<sensor>` argument and Node in `launch/car.launch.py`.
