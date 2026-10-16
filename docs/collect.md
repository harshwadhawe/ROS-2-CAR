# Collect data

```bash
ros2 launch pistar_car car.launch.py use_camera:=true record:=true
```
Live view: `http://<pi-ip>:8080/stream?topic=/camera/image_raw`. Left stick
steers, right stick throttle, centred = stop. **Ctrl-C once** and wait ~2 s;
the bag closes cleanly.

Each run writes `~/bags/drive_<timestamp>/`: `/camera/image_raw` (160x120 at
20 Hz) and `/cmd` (what was actually sent to the car, so labels stay correct
even if a stick was inverted). Check one with `ros2 bag info <dir>`.

## What makes good data
Data quality limits the policy more than anything else.
- Several minutes of smooth laps, in **both directions**.
- Progressive steering, not full lock: lower `teleop_joy: max_steering_angle` if needed.
- A few deliberate drift-and-recover moments, so it learns to come back.
- The lighting it will drive in.

Training drops frames that are stopped, reversing, off the track, or from
AUTO, so those cost nothing but teach nothing.

## Back up the bags
They only exist on the Pi. From the training machine:
```bash
scp -r pistar@<pi-ip>:bags ./training/out/
```

Next: [train](train.md).
