# Run

| Command | AUTO is driven by |
|---|---|
| `ros2 launch pistar_car car.launch.py use_camera:=true` | nothing (manual only) |
| `... use_camera:=true auto:=follower` | geometric lane follower |
| `... use_camera:=true auto:=policy` | learned policy, `~/models/policy.tflite` |

Add `record:=true` to keep collecting data.

- Starts in **MANUAL**. **Square** toggles **AUTO**; square again returns to manual.
- AUTO only steers; speed is fixed (`speed` in the `lane_follower` / `policy` blocks).
- The pad is the kill switch in both modes: if it drops, the car stops within
  ~0.75 s and comes back in manual. Both auto nodes also stop when they lose the track.
- First run of any change on a stand: hold the car off-centre, press square,
  check the wheels turn back towards the middle of the track.
- What the follower sees: `http://<pi-ip>:8080/stream?topic=/lane_follower/debug`.
- Stop with **one** Ctrl-C; repeated presses can kill the actuator before it sets neutral.
