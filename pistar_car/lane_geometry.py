"""Lane following maths, no ROS: floor projection, the track region, the
lookahead target and pure pursuit. Shared by lane_follower, policy_node,
training/bag_to_npz.py and the training baseline, so they cannot drift apart.

Conventions: floor coordinates are (forward, left) metres from the camera;
steering is radians, positive = left (REP-103). `p` is the lane_follower
parameter block of config/car.yaml.
"""
import math

import numpy as np
import cv2


def ground_grid(width, height, cam_height, pitch, hfov):
    """Per-pixel floor position -> (forward, left) arrays in metres, NaN above
    the horizon. Flat floor, camera pitch radians down, cam_height up."""
    f = (width / 2.0) / math.tan(hfov / 2.0)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    a = (u - width / 2.0) / f      # right, in focal units
    b = (v - height / 2.0) / f     # down, in focal units
    down = b * math.cos(pitch) + math.sin(pitch)
    t = np.where(down > 1e-6, cam_height / np.where(down > 1e-6, down, 1.0), np.nan)
    return t * (math.cos(pitch) - b * math.sin(pitch)), -t * a


def track_region(dark, seed_rows=20):
    """The dark region under the car: the connected component owning most of
    the bottom-centre patch. Dark areas cut off by lighter floor are dropped."""
    h, w = dark.shape
    _, labels = cv2.connectedComponents(dark.astype(np.uint8), connectivity=4)
    seed = labels[h - seed_rows:, w // 4:3 * w // 4]
    counts = np.bincount(seed.ravel(), minlength=2)
    counts[0] = 0                      # label 0 is "not dark"
    return labels == counts.argmax() if counts.max() else np.zeros_like(dark)


def road_target(road, X, Y, lookahead, band, min_pixels):
    """Centroid (forward, left) of road pixels on the lookahead arc, or None."""
    with np.errstate(invalid='ignore'):
        on_arc = road & (np.abs(np.hypot(X, Y) - lookahead) < band)
    if np.count_nonzero(on_arc) < min_pixels:
        return None
    return float(X[on_arc].mean()), float(Y[on_arc].mean())


def pure_pursuit(x, y, wheelbase):
    """Steering angle (rad, + = left) that arcs through ground point (x, y)."""
    d2 = x * x + y * y
    if d2 < 1e-9:
        return 0.0
    return math.atan(wheelbase * 2.0 * y / d2)


def grid_for(p, width, height):
    """ground_grid from a lane_follower parameter block."""
    return ground_grid(width, height, p['cam_height'],
                       math.radians(p['cam_pitch_deg']), math.radians(p['cam_hfov_deg']))


def steer_from_frame(rgb, X, Y, p):
    """The whole follower on one RGB frame -> (track mask, target, steer).
    target and steer are None when too little track is on the lookahead arc."""
    region = track_region(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[:, :, 2] <= p['road_v_max'])
    target = road_target(region, X, Y, p['lookahead_distance'], p['band'], p['min_pixels'])
    if target is None:
        return region, None, None
    limit = p['max_steering_angle']
    return region, target, max(-limit, min(limit, pure_pursuit(*target, p['wheelbase'])))
