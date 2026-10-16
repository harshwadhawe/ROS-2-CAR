"""Recorded drive bags -> one .npz per bag of (frame, label, flags). Runs on
the Pi (needs rosbag2_py). Filtering is left to training, so nothing is lost.

    python3 training/bag_to_npz.py ~/bags/drive_* --out ~/datasets

Per frame: image (120x160x3 RGB uint8), the command in force when it was
captured (steer rad, speed; /ackermann_cmd, or /cmd in bags from before the
F1TENTH topic names), how old that command was, and road_frac -- the share
of the frame that is track connected to the floor ahead (lane_follower's
track_region), which is ~0 when the car is off the track, and geom_steer --
what the geometric lane_follower would have steered, as a baseline to beat.
"""
import argparse
import bisect
import os

import cv2
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

from pistar_car.lane_geometry import grid_for, steer_from_frame


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
        elif topic in ('/ackermann_cmd', '/cmd'):
            m = deserialize_message(data, AckermannDriveStamped).drive if topic == '/ackermann_cmd' \
                else deserialize_message(data, AckermannDrive)
            cmds.append((t, m.steering_angle, m.speed))
    return imgs, cmds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default=os.path.expanduser('~/datasets'))
    ap.add_argument('--config', default=os.path.join(os.path.dirname(__file__), '..', 'config', 'car.yaml'))
    a = ap.parse_args()
    p = yaml.safe_load(open(a.config))['lane_follower']['ros__parameters']
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
        X, Y = grid_for(p, imgs[0][1].shape[1], imgs[0][1].shape[0])
        road, geom = np.zeros(n), np.full(n, np.nan)
        for i, (_, im) in enumerate(imgs):
            region, _, steer = steer_from_frame(im, X, Y, p)
            road[i] = region.mean()
            if steer is not None:
                geom[i] = steer
        name = os.path.basename(uri.rstrip('/'))
        np.savez_compressed(os.path.join(a.out, name + '.npz'),
                            image=np.stack([im for _, im in imgs]), t=np.array([t for t, _ in imgs]),
                            steer=steer, speed=speed, cmd_age=age, road_frac=road, geom_steer=geom)
        print(f'{name}: {n} frames, {np.isfinite(steer).sum()} labelled')


if __name__ == '__main__':
    main()
