"""OV5647 ribbon camera -> camera/image_raw (bgr8), without libcamera.

The apt libcamera cannot see the Pi 5 CSI port on Ubuntu's 6.8 kernel (see
README), so this reads raw Bayer straight off the CFE with v4l2-ctl and
debayers in OpenCV. Exposure, gain and white balance run on the sensor
itself (OV5647 AEC/AGC/AWB); there is no lens-shading correction.
"""
import subprocess

import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

W, H, BYTES_PER_LINE = 640, 480, 800   # 10-bit packed: 4 px in 5 bytes
FRAME_BYTES = BYTES_PER_LINE * H


def debayer(raw, out_size):
    """Packed SGBRG10 frame bytes -> BGR at out_size (w, h)."""
    bayer = np.frombuffer(raw, np.uint8).reshape(H, W // 4, 5)[:, :, :4].reshape(H, W)
    return cv2.resize(cv2.cvtColor(bayer, cv2.COLOR_BayerGR2BGR), out_size,
                      interpolation=cv2.INTER_AREA)


def configure(auto, gain, exposure):
    # ponytail: media0/subdev2 are the indices this Pi enumerates; look the
    # entities up by name if a second camera or kernel update reorders them.
    for cmd in (
        ['media-ctl', '-d', '/dev/media0', '-l', '"csi2":4 -> "rp1-cfe-csi2_ch0":0 [1]'],
        ['media-ctl', '-d', '/dev/media0', '-V', f'"csi2":0 [fmt:SGBRG10_1X10/{W}x{H} field:none]'],
        ['media-ctl', '-d', '/dev/media0', '-V', f'"csi2":4 [fmt:SGBRG10_1X10/{W}x{H} field:none]'],
        ['v4l2-ctl', '-d', '/dev/video0', f'--set-fmt-video=width={W},height={H},pixelformat=pGAA'],
        ['v4l2-ctl', '-d', '/dev/v4l-subdev2', '-c', f'exposure={exposure},analogue_gain={gain}'],
        # auto_exposure is a menu: 0 = auto, 1 = manual.
        ['v4l2-ctl', '-d', '/dev/v4l-subdev2', '-c',
         f'auto_exposure={0 if auto else 1},gain_automatic={int(auto)},'
         f'white_balance_automatic={int(auto)}'],
    ):
        subprocess.run(cmd, check=True)


class CsiCamera(Node):
    def __init__(self):
        super().__init__('camera')
        self.declare_parameter('width', 160)
        self.declare_parameter('height', 120)
        # Sensor-side auto exposure/gain/white balance. Manual values below
        # only apply with this off.
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('analogue_gain', 128)   # 16-1023
        self.declare_parameter('exposure', 500)        # lines, 4-500 at 640x480
        self.declare_parameter('frame_id', 'camera_link')
        p = lambda n: self.get_parameter(n).value
        self.size, self.frame_id = (p('width'), p('height')), p('frame_id')
        configure(p('auto_exposure'), p('analogue_gain'), p('exposure'))
        self.pub = self.create_publisher(Image, '~/image_raw', 1)
        self.bridge = CvBridge()

    def run(self):
        proc = subprocess.Popen(
            ['v4l2-ctl', '-d', '/dev/video0', '--stream-mmap', '--stream-to=-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=FRAME_BYTES)
        self.get_logger().info(f'Streaming {self.size[0]}x{self.size[1]}.')
        try:
            while rclpy.ok():
                raw = proc.stdout.read(FRAME_BYTES)
                if len(raw) < FRAME_BYTES:
                    self.get_logger().error('Camera stream ended.')
                    break
                msg = self.bridge.cv2_to_imgmsg(debayer(raw, self.size), 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.pub.publish(msg)
        finally:
            proc.terminate()


def main():
    rclpy.init()
    node = CsiCamera()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
