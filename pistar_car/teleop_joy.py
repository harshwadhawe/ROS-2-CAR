"""Gamepad teleop and the manual/auto switch. The pad owns the car.

Subscribes: joy (sensor_msgs/Joy)
            auto_cmd (ackermann_msgs/AckermannDrive) from lane_follower
Publishes:  cmd (ackermann_msgs/AckermannDrive) at 20 Hz.

Starts in MANUAL: sticks drive, like an RC transmitter. Square toggles AUTO,
where auto_cmd is forwarded instead; square again returns to manual. Auto
still needs a live pad: if the pad drops, the car stops and comes back in
manual. A stale auto_cmd (follower dead or lost) is sent as a stop.

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


def toggled(auto, was_down, down):
    """Flip the mode on the press edge only, so a held button flips once."""
    return not auto if down and not was_down else auto


class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        defaults = {
            'steer_axis': 0,           # left stick X
            'throttle_axis': 4,        # right stick Y
            # Signs measured on the real pad: with joy_node, stick-left is +1,
            # which is already REP-103 left. Flip either if a pad disagrees.
            'steer_scale': 1.0,
            'throttle_scale': 1.0,
            'max_speed': 1.0,
            'max_steering_angle': 0.4,
            'publish_rate': 20.0,      # must stay above the actuator watchdog
            # A dropped pad stops joy_node publishing but leaves our last
            # message cached. Without this the car keeps the held throttle.
            # Needs joy_node's autorepeat_rate non-zero to be meaningful.
            'joy_timeout': 0.5,
            'mode_button': 3,          # square on hid-playstation (cross 0, circle 1, triangle 2)
            'auto_timeout': 0.25,      # older auto_cmd than this is sent as a stop
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.joy = None
        self.last_joy = 0.0
        self.live = False
        self.auto = False
        self.button_down = False
        self.auto_cmd = None
        self.last_auto = 0.0
        self.pub = self.create_publisher(AckermannDrive, 'cmd', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_subscription(AckermannDrive, 'auto_cmd', self.on_auto, 10)
        self.create_timer(1.0 / self.p['publish_rate'], self.tick)
        self.get_logger().info(
            f"Steering axis {self.p['steer_axis']}, throttle axis "
            f"{self.p['throttle_axis']}. MANUAL; square toggles AUTO.")

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy = time.monotonic()
        i = self.p['mode_button']
        down = 0 <= i < len(msg.buttons) and msg.buttons[i] == 1
        auto = toggled(self.auto, self.button_down, down)
        self.button_down = down
        if auto != self.auto:
            self.auto = auto
            self.get_logger().info('AUTO: lane follower driving.' if auto else 'MANUAL: sticks driving.')

    def on_auto(self, msg):
        self.auto_cmd = msg
        self.last_auto = time.monotonic()

    def tick(self):
        if self.joy is None:
            return
        if time.monotonic() - self.last_joy > self.p['joy_timeout']:
            if self.live:
                self.get_logger().warn('No joy messages, pad dropped? Stopping.')
                self.live = False
                self.auto = False      # a reconnected pad comes back in manual
            return
        if not self.live:
            self.get_logger().info('Pad live, driving.')
            self.live = True

        if self.auto:
            fresh = time.monotonic() - self.last_auto <= self.p['auto_timeout']
            self.pub.publish(self.auto_cmd if fresh else AckermannDrive())
            return

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
