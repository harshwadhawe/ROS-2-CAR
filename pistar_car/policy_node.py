"""Learned steering: training/train.py's policy.onnx on each camera frame.

Subscribes: camera/image_raw (sensor_msgs/Image)
Publishes:  cmd (ackermann_msgs/AckermannDrive); launch remaps it to auto_cmd

The model only steers; speed is fixed. It runs on OpenCV's DNN module, so the
Pi needs nothing beyond what the camera stack already uses. When the visible
track region falls below min_road -- the same test that dropped off-track
frames from training -- it publishes a stop, so the model is never trusted
somewhere it was not trained.
"""
import os

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge

from pistar_car.lane_follower import track_region

MODEL_SIZE = (160, 120)         # train.py input, w x h


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy')
        defaults = {
            'model_path': '~/models/policy.onnx',
            'speed': 0.36,             # m/s nominal, as lane_follower
            'max_steering_angle': 0.4,
            'min_road': 0.15,          # matches train.py --min-road
            'road_v_max': 100,         # matches lane_follower
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        path = os.path.expanduser(self.p['model_path'])
        self.net = cv2.dnn.readNetFromONNX(path)
        self.bridge = CvBridge()
        self.lost = True
        self.pub = self.create_publisher(AckermannDrive, 'cmd', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 1)
        self.get_logger().info(f'Policy {path} loaded, waiting for camera.')

    def on_image(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        on_track = track_region(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[:, :, 2]
                                <= self.p['road_v_max']).mean() >= self.p['min_road']
        cmd = AckermannDrive()
        if on_track:
            # Model takes raw RGB 0-255 and normalises internally.
            self.net.setInput(cv2.dnn.blobFromImage(rgb, 1.0, MODEL_SIZE, swapRB=False, crop=False))
            limit = self.p['max_steering_angle']
            cmd.steering_angle = max(-limit, min(limit, float(self.net.forward().ravel()[0])))
            cmd.speed = self.p['speed']
            if self.lost:
                self.get_logger().info('On track, policy driving.')
                self.lost = False
        elif not self.lost:
            self.get_logger().warn('Off track, stopping.')
            self.lost = True
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
