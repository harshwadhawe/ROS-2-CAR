"""Learned steering: training/train.py's policy.tflite on each camera frame.

Subscribes: camera/image_raw (sensor_msgs/Image)
Publishes:  drive (ackermann_msgs/AckermannDriveStamped), F1TENTH's autonomy topic

The model only steers; speed is fixed. It runs on the ai_edge_litert
interpreter (pip install ai-edge-litert), the runtime donkeycar used. When
the visible track region falls below min_road -- the same test that dropped
off-track frames from training -- it publishes a stop, so the model is never
trusted somewhere it was not trained.
"""
import os

import cv2
import numpy as np
import rclpy
from ai_edge_litert.interpreter import Interpreter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge

from pistar_car.lane_geometry import track_region

MODEL_SIZE = (160, 120)         # train.py input, w x h


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy')
        defaults = {
            'model_path': '~/models/policy.tflite',
            'speed': 0.36,             # m/s nominal, as lane_follower
            'max_steering_angle': 0.4,
            'min_road': 0.15,          # matches train.py --min-road
            'road_v_max': 100,         # matches lane_follower
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        path = os.path.expanduser(self.p['model_path'])
        self.net = Interpreter(model_path=path)
        self.net.allocate_tensors()
        self.inp = self.net.get_input_details()[0]['index']
        self.out = self.net.get_output_details()[0]['index']
        self.bridge = CvBridge()
        self.lost = True
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 1)
        self.get_logger().info(f'Policy {path} loaded, waiting for camera.')

    def on_image(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        on_track = track_region(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[:, :, 2]
                                <= self.p['road_v_max']).mean() >= self.p['min_road']
        out = AckermannDriveStamped()
        out.header.stamp = msg.header.stamp      # the frame this steering came from
        out.header.frame_id = 'base_link'
        cmd = out.drive
        if on_track:
            # img_in is the frame as-is: 1 x 120 x 160 x 3, RGB 0-255.
            if rgb.shape[1::-1] != MODEL_SIZE:
                rgb = cv2.resize(rgb, MODEL_SIZE, interpolation=cv2.INTER_AREA)
            self.net.set_tensor(self.inp, rgb[None].astype(np.float32))
            self.net.invoke()
            limit = self.p['max_steering_angle']
            cmd.steering_angle = max(-limit, min(limit, float(self.net.get_tensor(self.out).ravel()[0])))
            cmd.speed = self.p['speed']
            if self.lost:
                self.get_logger().info('On track, policy driving.')
                self.lost = False
        elif not self.lost:
            self.get_logger().warn('Off track, stopping.')
            self.lost = True
        self.pub.publish(out)


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
