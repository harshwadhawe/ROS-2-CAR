# Train (Mac or GPU machine, not the Pi)

## 1. Bags -> arrays, on the Pi
```bash
cd ~/ros2_ws/src/pistar_car
python3 training/bag_to_npz.py ~/bags/drive_* --out ~/datasets
```
Per frame: image, the `/cmd` in force, its age, `road_frac` (share of the frame
that is track, ~0 off-track) and `geom_steer` (what the geometric follower
would steer, the baseline to beat).

## 2. Environment, once, on the training machine
```bash
git clone https://github.com/harshwadhawe/ROS-2-CAR.git pistar_car && cd pistar_car
python3 -m venv training/out/.venv          # --system-site-packages to reuse an installed torch
training/out/.venv/bin/pip install -r training/requirements.txt
```
The litert pins match camera_setup's lock file, the export chain known to work.

## 3. Train
```bash
scp -r pistar@<pi-ip>:datasets ./training/out/
training/out/.venv/bin/python training/train.py training/out/datasets/*.npz --out training/out/run1
```
| File in `run1/` | What |
|---|---|
| `policy.tflite` | What the Pi runs: `img_in` 1x120x160x3 raw RGB -> steering (rad) |
| `policy.pt` | PyTorch weights |
| `metrics.json` | Held-out scores vs baselines; tflite-vs-torch check |

Options: `--epochs` (20), `--lr` (3e-4), `--min-road` (0.15), `--seed` (0).

## 4. Read metrics.json before deploying
`policy` should clearly beat `baseline_geometric_follower`: lower `mae_rad`,
higher `r`. Validation is the last 20% of each bag, so it is the same session
as training and flatters the model; the track is the real test.
First model (2,702 usable frames): MAE 0.076 rad, r +0.877, vs the follower's 0.194 / +0.517.

What it does: keeps frames with a fresh human `/cmd`, moving forward, track
>= 15% of the frame (`data.py`); fine-tunes ImageNet MobileNetV2 with flip and
brightness augmentation (`model.py`, `train.py`); exports and checks the tflite
against torch on every val frame, failing loudly on a mismatch (`export.py`).

## 5. Copy to the Pi
```bash
ssh pistar@<pi-ip> mkdir -p models
scp training/out/run1/policy.tflite pistar@<pi-ip>:models/policy.tflite
```

Next: [run](run.md).
