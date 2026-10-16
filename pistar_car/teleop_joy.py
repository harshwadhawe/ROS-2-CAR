"""Gamepad teleop. Sticks drive the car directly, like an RC transmitter.

Subscribes: joy (sensor_msgs/Joy)
Publishes:  cmd (ackermann_msgs/AckermannDrive) at 20 Hz.

There is no deadman button: centred sticks mean stopped. Two things stop a
runaway instead. If the pad drops off Bluetooth, joy_node goes quiet and the
joy_timeout below stops us republishing the last held throttle. If this node
dies outright, the actuator's own watchdog returns the car to neutral.

Axis indices and stick directions differ between pad drivers (hid-sony vs
hid-playstation vs SDL). Check yours with `ros2 topic echo /joy` and override
the parameters in config/car.yaml rather than editing this file.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive


def scaled(axes, index, scale, limit):
    """One axis -> a clamped command. Pure, so it is checkable without a graph."""
    value = axes[index] if 0 <= index < len(axes) else 0.0
    return max(-limit, min(limit, value * scale * limit))


class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        defaults = {
            'steer_axis': 0,           # left stick X
            'throttle_axis': 4,        # right stick Y
            # Signs are measured on the real pad, not derived: steering needed
            # inverting, throttle did not. Flip either if a pad disagrees.
            'steer_scale': -1.0,
            'throttle_scale': 1.0,
            'max_speed': 1.0,
            'max_steering_angle': 0.4,
            'publish_rate': 20.0,      # must stay above the actuator watchdog
            # A dropped pad stops joy_node publishing but leaves our last
            # message cached. Without this the car keeps the held throttle.
            # Needs joy_node's autorepeat_rate non-zero to be meaningful.
            'joy_timeout': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.joy = None
        self.last_joy = 0.0
        self.live = False
        self.pub = self.create_publisher(AckermannDrive, 'cmd', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(1.0 / self.p['publish_rate'], self.tick)
        self.get_logger().info(
            f"Steering axis {self.p['steer_axis']}, throttle axis "
            f"{self.p['throttle_axis']}. Centre the sticks to stop.")

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy = time.monotonic()

    def tick(self):
        if self.joy is None:
            return
        if time.monotonic() - self.last_joy > self.p['joy_timeout']:
            if self.live:
                self.get_logger().warn('No joy messages, pad dropped? Stopping.')
                self.live = False
            return
        if not self.live:
            self.get_logger().info('Pad live, driving.')
            self.live = True

        msg = AckermannDrive()
        msg.steering_angle = scaled(self.joy.axes, self.p['steer_axis'],
                                    self.p['steer_scale'], self.p['max_steering_angle'])
        msg.speed = scaled(self.joy.axes, self.p['throttle_axis'],
                           self.p['throttle_scale'], self.p['max_speed'])
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopJoy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
