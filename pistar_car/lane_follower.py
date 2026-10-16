"""Camera-only pure pursuit between two yellow tape edges on a black track.

Subscribes: camera/image_raw (sensor_msgs/Image, rgb8)
Publishes:  cmd (ackermann_msgs/AckermannDrive)
            lane_follower/debug (sensor_msgs/Image), only while someone watches

Each frame: threshold yellow, walk the rows bottom-up finding the tape edge
either side of the running lane centre, project both edges onto the ground
with a flat-floor pinhole model, take their midpoint as the path, then steer
pure pursuit at the path point nearest lookahead_distance. No lane seen means
speed 0 is published, so a lost track stops the car rather than coasting.

Camera geometry (height, pitch, hfov) and wheelbase are measured, not derived:
tune them in config/car.yaml. The debug image shows the mask, the path and
the chosen target -- watch it with rqt_image_view while the car is on a stand.
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


def pixel_to_ground(u, v, width, height, cam_height, pitch, hfov):
    """Pixel -> (forward, left) metres on a flat floor, or None above horizon.

    Camera is pitch radians below horizontal, cam_height metres above floor.
    """
    f = (width / 2.0) / math.tan(hfov / 2.0)
    a = (u - width / 2.0) / f      # right, in focal units
    b = (v - height / 2.0) / f     # down, in focal units
    down = b * math.cos(pitch) + math.sin(pitch)
    if down <= 1e-6:
        return None
    t = cam_height / down
    return t * (math.cos(pitch) - b * math.sin(pitch)), -t * a


def pure_pursuit(x, y, wheelbase):
    """Steering angle (rad, + = left) that arcs through ground point (x, y)."""
    d2 = x * x + y * y
    if d2 < 1e-9:
        return 0.0
    return math.atan(wheelbase * 2.0 * y / d2)


def lane_path(mask, to_ground, lane_width, row_step=4):
    """Bottom-up scan of a binary mask -> list of (forward, left) centre points.

    One edge missing: offset the seen edge by half the lane width. Neither
    seen: skip the row. The search centre follows the lane so curves work.
    """
    h, w = mask.shape
    centre = w // 2
    path = []
    for v in range(h - 1, -1, -row_step):
        cols = np.flatnonzero(mask[v])
        left = cols[cols < centre]
        right = cols[cols >= centre]
        pl = to_ground(left.max(), v) if left.size else None
        pr = to_ground(right.min(), v) if right.size else None
        if pl and pr:
            p = ((pl[0] + pr[0]) / 2, (pl[1] + pr[1]) / 2)
            centre = (left.max() + right.min()) // 2
        elif pl:
            p = (pl[0], pl[1] - lane_width / 2)
        elif pr:
            p = (pr[0], pr[1] + lane_width / 2)
        else:
            continue
        path.append(p)
    return path


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        defaults = {
            'speed': 0.3,                  # m/s; actuator throttle_limit still caps it
            'lookahead_distance': 0.5,     # m
            'wheelbase': 0.26,             # m, axle to axle -- measure yours
            'max_steering_angle': 0.4,
            'cam_height': 0.15,            # m above floor -- measure
            'cam_pitch_deg': 20.0,         # down from horizontal -- measure
            'cam_hfov_deg': 60.0,          # from the webcam datasheet
            'lane_width': 0.5,             # m between tape inner edges
            'min_path_points': 3,          # fewer = lane lost, stop
            # Yellow in OpenCV HSV (H 0-179). Widen if the tape washes out.
            'hsv_low': [20, 80, 80],
            'hsv_high': [40, 255, 255],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.bridge = CvBridge()
        self.lost = True
        self.pub = self.create_publisher(AckermannDrive, 'cmd', 10)
        self.debug_pub = self.create_publisher(Image, 'lane_follower/debug', 1)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 1)
        self.get_logger().info('Lane follower up, waiting for camera.')

    def on_image(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        h, w = rgb.shape[:2]
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, np.array(self.p['hsv_low'], np.uint8),
                           np.array(self.p['hsv_high'], np.uint8))

        pitch = math.radians(self.p['cam_pitch_deg'])
        hfov = math.radians(self.p['cam_hfov_deg'])

        def to_ground(u, v):
            return pixel_to_ground(u, v, w, h, self.p['cam_height'], pitch, hfov)

        path = lane_path(mask, to_ground, self.p['lane_width'])

        cmd = AckermannDrive()
        target = None
        if len(path) >= self.p['min_path_points']:
            L = self.p['lookahead_distance']
            target = min(path, key=lambda q: abs(math.hypot(*q) - L))
            limit = self.p['max_steering_angle']
            cmd.steering_angle = max(-limit, min(limit, pure_pursuit(*target, self.p['wheelbase'])))
            cmd.speed = self.p['speed']
            if self.lost:
                self.get_logger().info('Lane found, driving.')
                self.lost = False
        elif not self.lost:
            self.get_logger().warn('Lane lost, stopping.')
            self.lost = True
        self.pub.publish(cmd)

        if self.debug_pub.get_subscription_count():
            self.publish_debug(rgb, mask, path, target, to_ground, msg.header)

    def publish_debug(self, rgb, mask, path, target, to_ground, header):
        # ponytail: brute-force ground->pixel by searching the projected grid;
        # 160x120 makes it cheap, invert pixel_to_ground if resolution grows.
        img = rgb.copy()
        img[mask > 0] = (255, 0, 255)
        h, w = mask.shape
        grid = [(u, v, to_ground(u, v)) for v in range(0, h, 2) for u in range(0, w, 2)]
        grid = [(u, v, g) for u, v, g in grid if g]
        for q in path:
            u, v, _ = min(grid, key=lambda c: (c[2][0] - q[0]) ** 2 + (c[2][1] - q[1]) ** 2)
            cv2.circle(img, (u, v), 1, (0, 255, 0), -1)
        if target:
            u, v, _ = min(grid, key=lambda c: (c[2][0] - target[0]) ** 2 + (c[2][1] - target[1]) ** 2)
            cv2.circle(img, (u, v), 4, (255, 0, 0), 2)
        out = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        out.header = header
        self.debug_pub.publish(out)

    def stop(self):
        self.pub.publish(AckermannDrive())


def main():
    rclpy.init()
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
