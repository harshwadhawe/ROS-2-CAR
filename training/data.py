"""Which frames train, the train/val split, and loading bag_to_npz output.
numpy only, so it runs anywhere (no torch, no ROS).

Kept frames: labelled, pad live, a human command (not AUTO), moving forward,
on the track. Each bag's last 20% is validation, after a 1 s gap, so
near-identical neighbouring frames never straddle the split.
"""
import os
import sys

import numpy as np
import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, ROOT)                    # pistar_car.lane_geometry, without installing
from pistar_car.lane_geometry import grid_for, steer_from_frame  # noqa: E402

FOLLOWER_SPEED = 0.36           # frames at exactly this speed came from AUTO
CONFIG = os.path.join(ROOT, 'config', 'car.yaml')


def keep(d, min_road):
    s, v = d['steer'], d['speed']
    return (np.isfinite(s) & (d['cmd_age'] < 0.25) & ~np.isclose(v, FOLLOWER_SPEED, atol=1e-3)
            & (v > 0.02) & (d['road_frac'] >= min_road))


def split(n, val_frac=0.2, gap=20):
    """Contiguous: first part train, last val_frac val, `gap` frames dropped between."""
    cut = int(n * (1 - val_frac))
    return np.arange(0, max(cut - gap, 0)), np.arange(cut, n)


def geometric_steer(frames, config=CONFIG):
    """What lane_follower would steer on each RGB frame (0 where it sees no track)."""
    p = yaml.safe_load(open(config))['lane_follower']['ros__parameters']
    X, Y = grid_for(p, frames.shape[2], frames.shape[1])
    return np.array([steer_from_frame(f, X, Y, p)[2] or 0.0 for f in frames])


def load(paths, min_road):
    """-> (train, val) dicts: x frames (N,120,160,3 RGB uint8), y steer rad;
    val also has geom (the follower's steer) and bag (source name per frame)."""
    tr, va = {'x': [], 'y': []}, {'x': [], 'y': [], 'geom': [], 'bag': []}
    for p in paths:
        d = np.load(p)
        idx = np.flatnonzero(keep(d, min_road))
        itr, iva = split(len(idx))
        for part, sel in ((tr, idx[itr]), (va, idx[iva])):
            part['x'].append(d['image'][sel])
            part['y'].append(d['steer'][sel])
        va['geom'].append(np.nan_to_num(d['geom_steer'][idx[iva]]) if 'geom_steer' in d
                          else geometric_steer(d['image'][idx[iva]]))
        va['bag'] += [os.path.basename(p)[6:21]] * len(iva)
    cat = lambda part: {k: np.concatenate(v) if k != 'bag' else np.array(v) for k, v in part.items()}
    return cat(tr), cat(va)
