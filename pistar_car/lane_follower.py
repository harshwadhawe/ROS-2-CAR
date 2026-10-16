"""Camera-only pure pursuit on the centroid of the dark road surface.

Subscribes: camera/image_raw (sensor_msgs/Image)
Publishes:  drive (ackermann_msgs/AckermannDriveStamped), F1TENTH's autonomy topic
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
config/car.yaml while watching the debug image with the car on a stand. The
maths lives in lane_geometry.py; this file is the ROS wiring and the overlay.
"""
import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge

from pistar_car.lane_geometry import grid_for, steer_from_frame


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        defaults = {
            'speed': 0.3,                  # m/s; actuator throttle_limit still caps it
            'lookahead_distance': 0.45,    # m
            'band': 0.1,                   # m either side of the lookahead arc
            'wheelbase': 0.264,            # m, from measured curvature, see car.yaml
            'max_steering_angle': 0.4,
            'cam_height': 0.17,            # m above floor
            'cam_pitch_deg': 12.5,         # down from horizontal
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
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.debug_pub = self.create_publisher(Image, 'lane_follower/debug', 1)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 1)
        self.get_logger().info('Lane follower up, waiting for camera.')

    def on_image(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        h, w = rgb.shape[:2]
        if self.grid is None or self.grid[0].shape != (h, w):
            self.grid = grid_for(self.p, w, h)
        road, target, steer = steer_from_frame(rgb, *self.grid, self.p)

        out = AckermannDriveStamped()
        out.header.stamp = msg.header.stamp      # the frame this steering came from
        out.header.frame_id = 'base_link'
        cmd = out.drive
        if target:
            cmd.steering_angle = steer
            cmd.speed = self.p['speed']
            if self.lost:
                self.get_logger().info('Road found, driving.')
                self.lost = False
        elif not self.lost:
            self.get_logger().warn('Road lost, stopping.')
            self.lost = True
        self.pub.publish(out)

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
