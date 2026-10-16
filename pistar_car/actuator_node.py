"""Drives steering servo and ESC through the PCA9685.

Subscribes: ackermann_cmd (ackermann_msgs/AckermannDriveStamped), as F1TENTH's VESC driver
  steering_angle: radians, positive = left (REP-103)
  speed:          m/s, negative = reverse
A watchdog returns the car to neutral if commands stop arriving.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from pistar_car.pca9685 import PCA9685


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_node')
        defaults = {
            'i2c_bus': 1,
            'i2c_address': 0x40,
            'pwm_frequency': 60,
            'steering_channel': 14,
            'throttle_channel': 1,
            # Zero means no pulse, so a car that never got its params sits
            # inert instead of driving stock values at the wrong hardware.
            # Real values live in config/car.yaml.
            'steering_left_pwm': 0,
            'steering_right_pwm': 0,
            'throttle_forward_pwm': 0,
            'throttle_stopped_pwm': 0,
            'throttle_reverse_pwm': 0,
            'max_steering_angle': 0.4,
            'max_speed': 2.0,
            'throttle_limit': 0.3,
            'watchdog_timeout': 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        if not any(self.p[k] for k in ('steering_left_pwm', 'steering_right_pwm',
                                       'throttle_forward_pwm', 'throttle_reverse_pwm')):
            self.get_logger().error(
                'All PWM endpoints are 0 -- car.yaml did not load. Car will not move.')

        self.pca = PCA9685(self.p['i2c_bus'], self.p['i2c_address'], self.p['pwm_frequency'])

        # Arm the ESC at neutral before accepting commands.
        self.neutral()
        time.sleep(1.0)

        self.last_cmd_time = 0.0
        self.stopped = True
        self.create_subscription(AckermannDriveStamped, 'ackermann_cmd', self.on_cmd, 10)
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info('Actuator ready. Waiting for commands on /ackermann_cmd.')

    def steering_pwm(self, angle):
        norm = clamp(angle / self.p['max_steering_angle'], -1.0, 1.0)
        right, left = self.p['steering_right_pwm'], self.p['steering_left_pwm']
        return right + (norm + 1.0) / 2.0 * (left - right)

    def throttle_pwm(self, speed):
        limit = self.p['throttle_limit']
        frac = clamp(speed / self.p['max_speed'], -limit, limit)
        stopped = self.p['throttle_stopped_pwm']
        if frac >= 0:
            return stopped + frac * (self.p['throttle_forward_pwm'] - stopped)
        return stopped + frac * (stopped - self.p['throttle_reverse_pwm'])

    def neutral(self):
        self.pca.set_pwm(self.p['steering_channel'], self.steering_pwm(0.0))
        self.pca.set_pwm(self.p['throttle_channel'], self.p['throttle_stopped_pwm'])

    def on_cmd(self, msg):
        self.last_cmd_time = time.monotonic()
        if self.stopped:
            self.get_logger().info('Commands received, driving.')
            self.stopped = False
        self.pca.set_pwm(self.p['steering_channel'], self.steering_pwm(msg.drive.steering_angle))
        self.pca.set_pwm(self.p['throttle_channel'], self.throttle_pwm(msg.drive.speed))

    def watchdog(self):
        if self.stopped:
            return
        if time.monotonic() - self.last_cmd_time > self.p['watchdog_timeout']:
            self.neutral()
            self.stopped = True
            self.get_logger().warn('No commands. Watchdog set car to neutral.')

    def shutdown(self):
        try:
            self.neutral()
            time.sleep(0.1)
        finally:
            self.pca.close()


def main():
    rclpy.init()
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C raises the first; ros2 launch tearing us down raises the second.
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
