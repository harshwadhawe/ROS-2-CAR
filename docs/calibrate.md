# Calibrate (once per car, wheels off the ground)

Everything is in `config/car.yaml`. After editing, rerun the `colcon build`
line from [install](install.md#2-code-and-build) and relaunch (launch reads the
installed copy of the yaml).

| What | Keys | How |
|---|---|---|
| Servo/ESC endpoints | `actuator_node: steering_*_pwm, throttle_*_pwm` | Copy from DonkeyCar's `myconfig.py`; same 12-bit ticks |
| Speed cap | `actuator_node: throttle_limit` (0.25) | The car starts moving near 0.17 |
| Stick directions | `teleop_joy: steer_scale, throttle_scale` | Stick left must turn the wheels left; flip the sign if not |
| Full stick = cap | `teleop_joy: max_speed` | `throttle_limit x actuator max_speed` (0.25 x 2.0 = 0.5) |
| Mode button | `teleop_joy: mode_button` | 3 = square on hid-playstation; check with `ros2 topic echo /joy` |
| Camera geometry | `lane_follower: cam_height, cam_pitch_deg, cam_hfov_deg` | Mark the floor 0.45 m ahead; it should sit on the green arc in `/lane_follower/debug` |
| Track threshold | `lane_follower: road_v_max` | Track blue, carpet not, in `/lane_follower/debug` |
| Camera exposure | `camera: auto_exposure` | true = sensor AE; false uses `analogue_gain` / `exposure` |

View the debug image: launch with `use_camera:=true auto:=follower`, open
`http://<pi-ip>:8080/stream?topic=/lane_follower/debug`.

Next: [collect](collect.md).
