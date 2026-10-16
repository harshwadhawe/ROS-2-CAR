# Troubleshooting

| Symptom | Fix |
|---|---|
| `Device or resource busy` from the camera | An old run still holds it: `pgrep -af pistar_car`, then `kill -INT <pid>` |
| A launch crashed or hung | Same check, then zero the PWM outputs: `python3 -c "from smbus2 import SMBus; b=SMBus(1); b.write_byte_data(0x40, 0xFD, 0x10); b.close()"` |
| Pi reboots or drops off Wi-Fi | `vcgencmd get_throttled`: bit 0 (`...5`) = under-voltage now. Separate Pi supply |
| `package 'web_video_server' not found` | `sudo apt update && sudo apt install ros-jazzy-web-video-server` |
| `No module named ai_edge_litert` | [install](install.md) step 1, the pip line |
| Feed too dark/bright | `camera: auto_exposure: true` (sensor AE); or false and tune `analogue_gain` |
| Steering backwards in MANUAL | flip `teleop_joy: steer_scale` |
| Steering backwards in AUTO | the actuator endpoints are swapped: `steering_left_pwm` must turn the wheels left |
| Square does nothing | `ros2 topic echo /joy` while pressing; set `teleop_joy: mode_button` |
