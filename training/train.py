"""Behaviour cloning: fine-tune ImageNet MobileNetV2 to steer from one frame.

    python3 training/train.py datasets/*.npz --out training/out

Needs torch, torchvision, and litert-torch + ai-edge-litert for the export
(the same pins as camera_setup's requirements.lock.txt).

Input is what bag_to_npz writes. Kept frames: labelled, pad live, a human
command (not the follower's), moving forward, on the track. Each bag's last
20% is validation, after a 1 s gap, so near-identical neighbouring frames
never straddle the split. Train-time flip (steer negated) and brightness
jitter; nothing else.

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
import torchvision

MAX_STEER = 0.4                 # rad, matches actuator/teleop max_steering_angle
FOLLOWER_SPEED = 0.36           # frames at exactly this speed came from AUTO


def keep(d, min_road):
    s, v = d['steer'], d['speed']
    return (np.isfinite(s) & (d['cmd_age'] < 0.25) & ~np.isclose(v, FOLLOWER_SPEED, atol=1e-3)
            & (v > 0.02) & (d['road_frac'] >= min_road))


def split(n, val_frac=0.2, gap=20):
    """Contiguous: first part train, last val_frac val, `gap` frames dropped between."""
    cut = int(n * (1 - val_frac))
    return np.arange(0, max(cut - gap, 0)), np.arange(cut, n)


class Policy(nn.Module):
    def __init__(self, pretrained=True):
        super().__init__()
        w = torchvision.models.MobileNet_V2_Weights.IMAGENET1K_V1 if pretrained else None
        self.features = torchvision.models.mobilenet_v2(weights=w).features
        self.head = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Dropout(0.3), nn.Linear(1280, 1))
        self.register_buffer('mean', torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1) * 255)
        self.register_buffer('std', torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1) * 255)

    def forward(self, x):       # x: N x 3 x H x W, RGB 0-255
        return torch.tanh(self.head(self.features((x - self.mean) / self.std))).reshape(-1) * MAX_STEER


class Deployed(nn.Module):
    """What the Pi runs: the camera frame as-is (N x H x W x 3, RGB 0-255)."""
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, img_in):
        return self.policy(img_in.permute(0, 3, 1, 2))


def export_tflite(model, path, check_frames, tol=1e-4):
    """torch -> tflite via litert_torch, then prove it with the interpreter."""
    import litert_torch
    from ai_edge_litert.interpreter import Interpreter
    deployed = Deployed(model).eval()
    # Keyword trace names the tflite input tensor img_in (positional = args_0).
    litert_torch.convert(deployed, sample_kwargs={'img_in': torch.zeros(1, 120, 160, 3)}).export(path)
    it = Interpreter(model_path=path)
    it.allocate_tensors()
    inp, out = it.get_input_details()[0], it.get_output_details()[0]
    with torch.no_grad():
        ref = deployed(torch.from_numpy(check_frames).float()).numpy()
    got = []
    for f in check_frames:
        it.set_tensor(inp['index'], f[None].astype(np.float32))
        it.invoke()
        got.append(float(it.get_tensor(out['index']).ravel()[0]))
    worst = float(np.abs(np.array(got) - ref).max())
    if worst > tol:
        raise SystemExit(f'tflite disagrees with torch by {worst:.2e} rad')
    return worst


def load(paths, min_road):
    tr, va = {'x': [], 'y': []}, {'x': [], 'y': [], 'geom': [], 'bag': []}
    for p in paths:
        d = np.load(p)
        idx = np.flatnonzero(keep(d, min_road))
        itr, iva = split(len(idx))
        for part, sel in ((tr, idx[itr]), (va, idx[iva])):
            part['x'].append(d['image'][sel])
            part['y'].append(d['steer'][sel])
        va['geom'].append(np.nan_to_num(d['geom_steer'][idx[iva]]) if 'geom_steer' in d else np.full(len(iva), np.nan))
        va['bag'] += [os.path.basename(p)[6:21]] * len(iva)
    cat = lambda part: {k: np.concatenate(v) if k != 'bag' else np.array(v) for k, v in part.items()}
    return cat(tr), cat(va)


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
              # None when the npz predates geom_steer (re-run bag_to_npz).
              'baseline_geometric_follower': None if np.isnan(va['geom']).any() else metrics(va['geom'], y),
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
