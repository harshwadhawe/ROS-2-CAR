"""Keyboard teleop -> /ackermann_cmd (straight to the actuator; launch with
use_joy:=false so teleop_joy is not also publishing). Run with `ros2 run` in a
real terminal (not from a launch file)."""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

HELP = """
Pi Star car teleop
  w / s : speed up / slow down (s goes into reverse past zero)
  a / d : steer left / right
  c     : center steering
  space : stop
  q     : quit
"""


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('steer_step', 0.05)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.4)
        self.speed_step = self.get_parameter('speed_step').value
        self.steer_step = self.get_parameter('steer_step').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.speed = 0.0
        self.steer = 0.0

    def on_key(self, key):
        if key == 'w':
            self.speed += self.speed_step
        elif key == 's':
            self.speed -= self.speed_step
        elif key == 'a':
            self.steer += self.steer_step
        elif key == 'd':
            self.steer -= self.steer_step
        elif key == 'c':
            self.steer = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.steer = 0.0
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed))
        self.steer = max(-self.max_steer, min(self.max_steer, self.steer))
        sys.stdout.write(f'\rspeed {self.speed:+.2f} m/s   steer {self.steer:+.2f} rad   ')
        sys.stdout.flush()

    def publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steer)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Teleop()
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(HELP)
    try:
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = sys.stdin.read(1)
                if key == 'q':
                    break
                node.on_key(key)
            node.publish()  # steady 20 Hz keeps the actuator watchdog happy
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.speed = 0.0
        node.steer = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
