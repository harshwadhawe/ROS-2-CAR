"""Behaviour cloning: fine-tune ImageNet MobileNetV2 to steer from one frame.

    python3 training/train.py datasets/*.npz --out training/out

Needs torch, torchvision, and litert-torch + ai-edge-litert for the export
(the same pins as camera_setup's requirements.lock.txt).

Input is what bag_to_npz writes; data.py picks the frames and the split,
model.py is the network, export.py the tflite. This file is the loop:
train-time flip (steer negated) and brightness jitter, best epoch by val MAE.

Writes policy.tflite (img_in: 1x120x160x3 raw RGB 0-255 -> steer rad, layout
and normalisation baked in so the Pi only feeds the camera frame), policy.pt
and metrics.json. The tflite is checked against torch on every val frame.
"""
import argparse
import json
import os

import numpy as np
import torch
import torch.nn as nn

from data import load
from export import export_tflite
from model import MAX_STEER, Policy

def to_tensor(x, device):
    return torch.from_numpy(x).to(device).permute(0, 3, 1, 2).float()


def augment(x, y):
    flip = torch.rand(len(x), device=x.device) < 0.5
    x = torch.where(flip.view(-1, 1, 1, 1), x.flip(3), x)
    y = torch.where(flip, -y, y)
    gain = torch.empty(len(x), 1, 1, 1, device=x.device).uniform_(0.7, 1.3)
    bias = torch.empty(len(x), 1, 1, 1, device=x.device).uniform_(-20, 20)
    return (x * gain + bias).clamp(0, 255), y


def metrics(pred, y):
    turn = np.abs(y) > 0.1
    return {'mae_rad': float(np.abs(pred - y).mean()),
            'r': float(np.corrcoef(pred, y)[0, 1]) if pred.std() > 1e-9 else 0.0,
            'sign_agree_on_turns': float((np.sign(pred[turn]) == np.sign(y[turn])).mean()) if turn.any() else None}


@torch.no_grad()
def predict(model, x, device, bs=256):
    model.eval()
    return np.concatenate([model(to_tensor(x[i:i + bs], device)).cpu().numpy() for i in range(0, len(x), bs)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', nargs='+')
    ap.add_argument('--out', default='training/out')
    ap.add_argument('--epochs', type=int, default=20)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--min-road', type=float, default=0.15)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    torch.manual_seed(a.seed)
    np.random.seed(a.seed)
    device = 'mps' if torch.backends.mps.is_available() else 'cpu'
    tr, va = load(a.npz, a.min_road)
    print(f'train {len(tr["y"])} frames, val {len(va["y"])} frames, device {device}')

    model = Policy().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * int(np.ceil(len(tr['y']) / 64))
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=steps)
    loss_fn = nn.SmoothL1Loss(beta=0.1)
    ytr = torch.from_numpy(tr['y']).float().to(device)
    best, best_state = None, None
    for ep in range(a.epochs):
        model.train()
        perm = np.random.permutation(len(ytr))
        total = 0.0
        for i in range(0, len(perm), 64):
            b = perm[i:i + 64]
            x, y = augment(to_tensor(tr['x'][b], device), ytr[b])
            loss = loss_fn(model(x) / MAX_STEER, y / MAX_STEER)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            total += loss.item() * len(b)
        m = metrics(predict(model, va['x'], device), va['y'])
        print(f'epoch {ep + 1:2d}  train loss {total / len(perm):.4f}  val mae {m["mae_rad"]:.4f} rad  r {m["r"]:+.3f}')
        if best is None or m['mae_rad'] < best['mae_rad']:
            best, best_state = dict(m, epoch=ep + 1), {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

    model.load_state_dict(best_state)
    pred = predict(model, va['x'], device)
    y = va['y']
    report = {'train_frames': int(len(tr['y'])), 'val_frames': int(len(y)), 'best_epoch': best['epoch'],
              'policy': metrics(pred, y),
              'baseline_straight': metrics(np.zeros_like(y), y),
              'baseline_mean': metrics(np.full_like(y, tr['y'].mean()), y),
              'baseline_geometric_follower': metrics(va['geom'], y),
              'policy_per_bag': {b: metrics(pred[va['bag'] == b], y[va['bag'] == b]) for b in np.unique(va['bag'])}}
    os.makedirs(a.out, exist_ok=True)
    json.dump(report, open(os.path.join(a.out, 'metrics.json'), 'w'), indent=1)
    np.savez(os.path.join(a.out, 'val_pred.npz'), pred=pred, y=y, geom=va['geom'], bag=va['bag'])

    model = model.cpu().eval()
    torch.save(model.state_dict(), os.path.join(a.out, 'policy.pt'))
    report['tflite_max_abs_diff_rad'] = export_tflite(model, os.path.join(a.out, 'policy.tflite'), va['x'])
    json.dump(report, open(os.path.join(a.out, 'metrics.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in report.items() if k != 'policy_per_bag'}, indent=1))


if __name__ == '__main__':
    main()
