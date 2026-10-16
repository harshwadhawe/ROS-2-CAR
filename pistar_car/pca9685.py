"""Minimal PCA9685 driver over smbus2. No Adafruit/Blinka dependency."""
import time
from smbus2 import SMBus

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD

SLEEP = 0x10
AUTO_INCREMENT = 0x20
RESTART = 0x80


class PCA9685:
    def __init__(self, bus=1, address=0x40, frequency=60):
        self.address = address
        self.bus = SMBus(bus)
        # The chip latches its last pulse across process restarts, and
        # set_frequency's RESTART resumes it. Drop it before that happens.
        self.all_off()
        self.set_frequency(frequency)

    def set_frequency(self, frequency):
        prescale = int(round(25_000_000 / (4096 * frequency))) - 1
        old_mode = self.bus.read_byte_data(self.address, MODE1)
        self.bus.write_byte_data(self.address, MODE1, (old_mode & 0x7F) | SLEEP)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, old_mode & ~SLEEP & 0xFF)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, (old_mode & ~SLEEP & 0xFF) | RESTART | AUTO_INCREMENT)

    def set_pwm(self, channel, off_ticks, on_ticks=0):
        """Set a channel's pulse in 12-bit ticks (0-4095), same units DonkeyCar uses."""
        off_ticks = max(0, min(4095, int(off_ticks)))
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(
            self.address, reg,
            [on_ticks & 0xFF, on_ticks >> 8, off_ticks & 0xFF, off_ticks >> 8],
        )

    def all_off(self):
        """Stop every output. No pulse beats a stale one: an ESC reads it as
        signal loss and cuts the motor, a servo just goes limp."""
        self.bus.write_byte_data(self.address, ALL_LED_OFF_H, 0x10)

    def close(self):
        self.all_off()
        self.bus.close()
