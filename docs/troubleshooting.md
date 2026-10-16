# Troubleshooting

| Symptom | Fix |
|---|---|
| `Device or resource busy` from the camera | Something else holds it: `sudo fuser -v /dev/video0 /dev/media0`; often an old run (`pgrep -af pistar_car`, `kill -INT <pid>`). See [conflicts](install.md#conflicts) |
| `Unable to acquire a CFE instance` / `no cameras available` | That is libcamera / `camera_ros`, which cannot work here; use `use_camera:=true` (csi_camera). [Why](install.md#libcamera-do-not-install) |
| `ImportError`/ABI crash mentioning numpy | pip put numpy 2 in `~/.local`: `python3 -m pip uninstall --user numpy`, reinstall `ai-edge-litert` with `"numpy<2"` |
| A launch crashed or hung | Same check, then zero the PWM outputs: `python3 -c "from smbus2 import SMBus; b=SMBus(1); b.write_byte_data(0x40, 0xFD, 0x10); b.close()"` |
| Pi reboots or drops off Wi-Fi | `vcgencmd get_throttled`: bit 0 (`...5`) = under-voltage now. Separate Pi supply |
| `package 'web_video_server' not found` | `sudo apt update && sudo apt install ros-jazzy-web-video-server` |
| `No module named ai_edge_litert` | [install](install.md) step 1, the pip line |
| Feed too dark/bright | `camera: auto_exposure: true` (sensor AE); or false and tune `analogue_gain` |
| Steering backwards in MANUAL | flip `teleop_joy: steer_scale` |
| Steering backwards in AUTO | the actuator endpoints are swapped: `steering_left_pwm` must turn the wheels left |
| Square does nothing | `ros2 topic echo /joy` while pressing; set `teleop_joy: mode_button` |
