"""Self-check for lane geometry and pure pursuit. Run on the Pi with ROS sourced:
PYTHONPATH=.:$PYTHONPATH python3 test/test_lane_follower.py"""
import math

import numpy as np

from pistar_car.lane_follower import pixel_to_ground, pure_pursuit, lane_path

W, H, CAM_H, PITCH, FOV = 160, 120, 0.15, math.radians(20), math.radians(60)


def to_ground(u, v):
    return pixel_to_ground(u, v, W, H, CAM_H, PITCH, FOV)


# Image centre looks down the optical axis: straight ahead, on the floor at
# cam_height / tan(pitch).
x, y = to_ground(W / 2, H / 2)
assert abs(x - CAM_H / math.tan(PITCH)) < 1e-9 and abs(y) < 1e-9
# Left of centre is +left (REP-103); above the horizon never hits the floor.
assert to_ground(0, H - 1)[1] > 0
assert to_ground(W / 2, 0) is None or to_ground(W / 2, 0)[0] > 2.0

# Pure pursuit: dead ahead is straight, target to the left steers left.
assert pure_pursuit(1.0, 0.0, 0.26) == 0.0
assert pure_pursuit(1.0, 0.2, 0.26) > 0
assert pure_pursuit(1.0, -0.2, 0.26) < 0


def lane_mask(offset, lane_width=0.5):
    """Render tape edges for a car sitting `offset` m left of lane centre."""
    m = np.zeros((H, W), np.uint8)
    for v in range(H):
        for u in range(W):
            g = to_ground(u, v)
            if g and g[0] < 3.0:
                for edge in (lane_width / 2, -lane_width / 2):
                    if abs(g[1] - (edge - offset)) < 0.02:
                        m[v, u] = 255
    return m


# Centred in the lane: path runs straight down the middle.
path = lane_path(lane_mask(0.0), to_ground, 0.5)
assert len(path) > 5
assert all(abs(py) < 0.03 for _, py in path), path

# Car drifted left: lane centre is to the right, so steer right (negative).
path = lane_path(lane_mask(0.1), to_ground, 0.5)
tx, ty = min(path, key=lambda q: abs(math.hypot(*q) - 0.5))
assert abs(ty + 0.1) < 0.03 and pure_pursuit(tx, ty, 0.26) < 0

# Only the right tape visible: fall back to half a lane width left of it.
m = lane_mask(0.0)
m[:, :W // 2] = 0
path = lane_path(m, to_ground, 0.5)
assert path and all(abs(py) < 0.03 for _, py in path), path

# Nothing yellow: no path, so the node publishes stop.
assert lane_path(np.zeros((H, W), np.uint8), to_ground, 0.5) == []

print('lane_follower geometry: all checks passed')
