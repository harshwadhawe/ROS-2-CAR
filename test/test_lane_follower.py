"""Self-check for road-centroid geometry and pure pursuit. Run: colcon test, or python3 test/test_lane_follower.py"""
import math

import numpy as np

from pistar_car.lane_follower import ground_grid, road_target, pure_pursuit

W, H, CAM_H, PITCH, FOV = 160, 120, 0.15, math.radians(20), math.radians(60)
X, Y = ground_grid(W, H, CAM_H, PITCH, FOV)
ROAD_W, L, BAND, MIN_PX = 0.392, 0.45, 0.1, 30


def road_mask(offset=0.0, curve=0.0):
    """Road strip for a car `offset` m left of centre; `curve` 1/m bends it left."""
    with np.errstate(invalid='ignore'):
        centre = -offset + 0.5 * curve * X ** 2
        return (np.abs(Y - centre) < ROAD_W / 2) & (X < 3.0)


def test_geometry():
    # Image centre looks down the optical axis, landing cam_height/tan(pitch) ahead.
    assert abs(X[H // 2, W // 2] - CAM_H / math.tan(PITCH)) < 1e-9
    assert abs(Y[H // 2, W // 2]) < 1e-9
    # Left of the image is +left (REP-103); the top rows are sky, not floor.
    assert Y[H - 1, 0] > 0 > Y[H - 1, W - 1]
    assert np.isnan(X[0, W // 2])

    # Pure pursuit: dead ahead is straight, a target to the left steers left.
    assert pure_pursuit(1.0, 0.0, 0.264) == 0.0
    assert pure_pursuit(1.0, 0.2, 0.264) > 0 > pure_pursuit(1.0, -0.2, 0.264)

    # Centred on a straight: target dead ahead at the lookahead distance.
    tx, ty = road_target(road_mask(), X, Y, L, BAND, MIN_PX)
    assert abs(ty) < 0.01 and abs(math.hypot(tx, ty) - L) < BAND, (tx, ty)

    # Drifted 0.1 m left: road centre is to the right, so steer right. The far
    # edge falls outside a 60 deg view, so the centroid under-reads the offset
    # (-0.08 here) -- a gentler correction, never the wrong way.
    tx, ty = road_target(road_mask(offset=0.1), X, Y, L, BAND, MIN_PX)
    assert -0.1 - 0.01 < ty < -0.05 and pure_pursuit(tx, ty, 0.264) < 0, (tx, ty)

    # Road bending left: steer left, before any lateral error has built up.
    tx, ty = road_target(road_mask(curve=1.0), X, Y, L, BAND, MIN_PX)
    assert pure_pursuit(tx, ty, 0.264) > 0, (tx, ty)

    # Road only near the bumper, nothing on the arc: lost, so the node stops.
    near = road_mask() & (X < L - 2 * BAND)
    assert road_target(near, X, Y, L, BAND, MIN_PX) is None
    assert road_target(np.zeros((H, W), bool), X, Y, L, BAND, MIN_PX) is None

    print('lane_follower geometry: all checks passed')


if __name__ == '__main__':
    test_geometry()
