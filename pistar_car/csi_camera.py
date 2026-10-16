"""OV5647 ribbon camera -> camera/image_raw (bgr8), without libcamera.

The apt libcamera cannot see the Pi 5 CSI port on Ubuntu's 6.8 kernel (see
README), so this reads raw Bayer straight off the CFE with v4l2-ctl and
debayers in OpenCV. Exposure, gain and white balance run on the sensor
itself (OV5647 AEC/AGC/AWB); there is no lens-shading correction. The sensor
specifics live in ov5647.py; this file is the ROS wiring.
"""
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pistar_car.ov5647 import FRAME_BYTES, configure, debayer

class CsiCamera(Node):
    def __init__(self):
        super().__init__('camera')
        self.declare_parameter('width', 160)
        self.declare_parameter('height', 120)
        self.declare_parameter('fps', 20.0)            # sensor-side, ~1-62
        # Sensor-side auto exposure/gain/white balance. Manual values below
        # only apply with this off.
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('analogue_gain', 128)   # 16-1023
        self.declare_parameter('exposure', 500)        # lines, 4-500 at 640x480
        self.declare_parameter('frame_id', 'camera_link')
        p = lambda n: self.get_parameter(n).value
        self.size, self.frame_id = (p('width'), p('height')), p('frame_id')
        configure(p('fps'), p('auto_exposure'), p('analogue_gain'), p('exposure'))
        self.pub = self.create_publisher(Image, '~/image_raw', 1)
        self.bridge = CvBridge()

        self.proc = subprocess.Popen(
            ['v4l2-ctl', '-d', '/dev/video0', '--stream-mmap', '--stream-to=-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=FRAME_BYTES)
        # Blocking reads live on their own thread so spin() can still serve
        # parameter and introspection requests.
        threading.Thread(target=self.stream, daemon=True).start()
        self.get_logger().info(f"Streaming {self.size[0]}x{self.size[1]} at {p('fps')} Hz.")

    def stream(self):
        while rclpy.ok():
            raw = self.proc.stdout.read(FRAME_BYTES)
            if len(raw) < FRAME_BYTES:
                self.get_logger().error('Camera stream ended.')
                rclpy.try_shutdown()
                return
            msg = self.bridge.cv2_to_imgmsg(debayer(raw, self.size), 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = CsiCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.proc.terminate()   # else v4l2-ctl outlives us and holds /dev/video0
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
