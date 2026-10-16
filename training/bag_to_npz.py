"""Recorded drive bags -> one .npz per bag of (frame, label, flags). Runs on
the Pi (needs rosbag2_py). Filtering is left to training, so nothing is lost.

    python3 training/bag_to_npz.py ~/bags/drive_* --out ~/datasets

Per frame: image (120x160x3 RGB uint8), the /cmd in force when it was
captured (steer rad, speed), how old that /cmd was, and road_frac -- the share
of the frame that is track connected to the floor ahead (lane_follower's
track_region), which is ~0 when the car is off the track, and geom_steer --
what the geometric lane_follower would have steered, as a baseline to beat.
"""
import argparse
import bisect
import math
import os

import cv2
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive

from pistar_car.lane_follower import ground_grid, pure_pursuit, road_target, track_region


def read(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    imgs, cmds = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/camera/image_raw':
            m = deserialize_message(data, Image)
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            imgs.append((t, a if m.encoding == 'rgb8' else cv2.cvtColor(a, cv2.COLOR_BGR2RGB)))
        elif topic == '/cmd':
            m = deserialize_message(data, AckermannDrive)
            cmds.append((t, m.steering_angle, m.speed))
    return imgs, cmds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default=os.path.expanduser('~/datasets'))
    ap.add_argument('--config', default=os.path.join(os.path.dirname(__file__), '..', 'config', 'car.yaml'))
    a = ap.parse_args()
    p = yaml.safe_load(open(a.config))['lane_follower']['ros__parameters']
    lim = p.get('max_steering_angle', 0.4)
    os.makedirs(a.out, exist_ok=True)
    for uri in a.bags:
        imgs, cmds = read(uri.rstrip('/'))
        ct = [c[0] for c in cmds]
        n = len(imgs)
        steer, speed, age = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.inf)
        for i, (t, _) in enumerate(imgs):
            j = bisect.bisect_right(ct, t) - 1
            if j >= 0:
                _, steer[i], speed[i] = cmds[j]
                age[i] = (t - ct[j]) / 1e9
        h, w = imgs[0][1].shape[:2]
        X, Y = ground_grid(w, h, p['cam_height'], math.radians(p['cam_pitch_deg']), math.radians(p['cam_hfov_deg']))
        road, geom = np.zeros(n), np.full(n, np.nan)
        for i, (_, im) in enumerate(imgs):
            region = track_region(cv2.cvtColor(im, cv2.COLOR_RGB2HSV)[:, :, 2] <= p['road_v_max'])
            road[i] = region.mean()
            tgt = road_target(region, X, Y, p['lookahead_distance'], p['band'], p['min_pixels'])
            if tgt:
                geom[i] = max(-lim, min(lim, pure_pursuit(*tgt, p['wheelbase'])))
        name = os.path.basename(uri.rstrip('/'))
        np.savez_compressed(os.path.join(a.out, name + '.npz'),
                            image=np.stack([im for _, im in imgs]), t=np.array([t for t, _ in imgs]),
                            steer=steer, speed=speed, cmd_age=age, road_frac=road, geom_steer=geom)
        print(f'{name}: {n} frames, {np.isfinite(steer).sum()} labelled')


if __name__ == '__main__':
    main()
