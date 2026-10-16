# pistar_car: notes for editing

- Each node is split: `*_node.py` / node files hold only ROS wiring; the logic
  is in a ROS-free module (`lane_geometry`, `ov5647`, `pad_mapping`, `pca9685`).
  Change behaviour in the module; tests import only modules.
- Tests run anywhere: `PYTHONPATH=. python3 -m pytest test/`. On the Pi also
  `colcon test --packages-select pistar_car`.
- Every tunable is in `config/car.yaml`; keep node defaults in step with it.
- Steering radians, + = left (REP-103). F1TENTH topics, all
  `AckermannDriveStamped`: auto sources publish `/drive`; teleop_joy (the mux)
  is the only `/ackermann_cmd` publisher; actuator_node listens there only.
- Never launch anything that drives the motors remotely; check `pgrep -af pistar_car` before testing on the Pi.
- Training: `training/{data,model,export,train}.py`; never the donkeycar package. Export is TFLite (litert_torch), not ONNX.
- Docs: one file per stage in `docs/`; README holds prerequisites and the code map.
