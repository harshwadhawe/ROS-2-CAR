"""Camera-only pure pursuit on the centroid of the dark road surface.

Subscribes: camera/image_raw (sensor_msgs/Image)
Publishes:  cmd (ackermann_msgs/AckermannDrive)
            lane_follower/debug (sensor_msgs/Image), only while someone watches

Each frame: threshold the dark track, keep only the dark region connected to
the floor just ahead of the car, take its pixels whose floor distance is
within `band` of lookahead_distance (an arc across the road), average their
floor positions, and steer pure pursuit through that point. Too few road
pixels on the arc means speed 0 is published, so a lost track stops the car
rather than coasting.

The track is black on beige carpet, ringed by grey carpet that is just as dark.
Brightness cannot tell grey carpet from track; connectivity can, because beige
always lies between them. Road centroid also beat edge methods on the old
taped track (r=+0.52 vs +0.15, camera_setup/lane_detection_track.ipynb).

Camera geometry (height, pitch, hfov) must be measured on the car; tune it in
config/car.yaml while watching the debug image with the car on a stand.
"""
import math

import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge


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


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        defaults = {
            'speed': 0.3,                  # m/s; actuator throttle_limit still caps it
            'lookahead_distance': 0.45,    # m
            'band': 0.1,                   # m either side of the lookahead arc
            'wheelbase': 0.264,            # m, from measured curvature, see car.yaml
            'max_steering_angle': 0.4,
            'cam_height': 0.15,            # m above floor -- measure
            'cam_pitch_deg': 20.0,         # down from horizontal -- measure
            'cam_hfov_deg': 60.0,          # lens field of view -- measure
            'road_v_max': 100,             # HSV value at or below = dark (0-255)
            'min_pixels': 30,              # fewer road pixels on the arc = lost
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.grid = None                   # built on the first frame, size known then
        self.bridge = CvBridge()
        self.lost = True
        self.pub = self.create_publisher(AckermannDrive, 'cmd', 10)
        self.debug_pub = self.create_publisher(Image, 'lane_follower/debug', 1)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 1)
        self.get_logger().info('Lane follower up, waiting for camera.')

    def on_image(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        h, w = rgb.shape[:2]
        if self.grid is None or self.grid[0].shape != (h, w):
            self.grid = ground_grid(w, h, self.p['cam_height'],
                                    math.radians(self.p['cam_pitch_deg']),
                                    math.radians(self.p['cam_hfov_deg']))
        X, Y = self.grid
        road = track_region(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[:, :, 2] <= self.p['road_v_max'])
        target = road_target(road, X, Y, self.p['lookahead_distance'],
                             self.p['band'], self.p['min_pixels'])

        cmd = AckermannDrive()
        if target:
            limit = self.p['max_steering_angle']
            cmd.steering_angle = max(-limit, min(limit, pure_pursuit(*target, self.p['wheelbase'])))
            cmd.speed = self.p['speed']
            if self.lost:
                self.get_logger().info('Road found, driving.')
                self.lost = False
        elif not self.lost:
            self.get_logger().warn('Road lost, stopping.')
            self.lost = True
        self.pub.publish(cmd)

        if self.debug_pub.get_subscription_count():
            self.publish_debug(rgb, road, target, msg.header)

    def publish_debug(self, rgb, road, target, header):
        X, Y = self.grid
        with np.errstate(invalid='ignore'):
            arc = np.abs(np.hypot(X, Y) - self.p['lookahead_distance']) < self.p['band']
        img = rgb.copy()
        img[road] = img[road] // 2 + np.array([0, 0, 120], np.uint8)        # road: blue tint
        img[road & arc] = (0, 255, 0)                                       # road on the arc
        if target:
            v, u = np.unravel_index(np.nanargmin((X - target[0]) ** 2 + (Y - target[1]) ** 2), X.shape)
            cv2.circle(img, (int(u), int(v)), 4, (255, 0, 0), 2)
        out = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        out.header = header
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
