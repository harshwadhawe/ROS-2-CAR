"""OV5647 on the Pi 5 CSI port, no ROS: raw-mode constants, frame-rate maths,
the media-ctl/v4l2-ctl setup and the packed-Bayer unpack. csi_camera.py is the
ROS wrapper around this.

The sensor runs 640x480 SGBRG10 (packed, pGAA) straight off the CFE; the ISP
is bypassed because the apt libcamera cannot see this port on Ubuntu's 6.8
kernel (docs/reference.md).
"""
import subprocess

import numpy as np
import cv2

W, H, BYTES_PER_LINE = 640, 480, 800   # 10-bit packed: 4 px in 5 bytes
FRAME_BYTES = BYTES_PER_LINE * H
# Sensor line timing at 640x480 (read-only pixel_rate and horizontal_blanking
# controls). Frame rate is set by stretching vertical blanking.
PIXEL_RATE, LINE_PIXELS = 58_333_000, W + 1212


def vblank_for(fps):
    """vertical_blanking lines that make the sensor run at fps."""
    return max(24, min(32287, round(PIXEL_RATE / (LINE_PIXELS * fps)) - H))


def debayer(raw, out_size):
    """Packed SGBRG10 frame bytes -> BGR at out_size (w, h)."""
    bayer = np.frombuffer(raw, np.uint8).reshape(H, W // 4, 5)[:, :, :4].reshape(H, W)
    return cv2.resize(cv2.cvtColor(bayer, cv2.COLOR_BayerGR2BGR), out_size,
                      interpolation=cv2.INTER_AREA)


def configure(fps, auto, gain, exposure):
    # ponytail: media0/subdev2 are the indices this Pi enumerates; look the
    # entities up by name if a second camera or kernel update reorders them.
    for cmd in (
        ['media-ctl', '-d', '/dev/media0', '-l', '"csi2":4 -> "rp1-cfe-csi2_ch0":0 [1]'],
        ['media-ctl', '-d', '/dev/media0', '-V', f'"csi2":0 [fmt:SGBRG10_1X10/{W}x{H} field:none]'],
        ['media-ctl', '-d', '/dev/media0', '-V', f'"csi2":4 [fmt:SGBRG10_1X10/{W}x{H} field:none]'],
        ['v4l2-ctl', '-d', '/dev/video0', f'--set-fmt-video=width={W},height={H},pixelformat=pGAA'],
        ['v4l2-ctl', '-d', '/dev/v4l-subdev2', '-c', f'vertical_blanking={vblank_for(fps)}'],
        ['v4l2-ctl', '-d', '/dev/v4l-subdev2', '-c', f'exposure={exposure},analogue_gain={gain}'],
        # auto_exposure is a menu: 0 = auto, 1 = manual.
        ['v4l2-ctl', '-d', '/dev/v4l-subdev2', '-c',
         f'auto_exposure={0 if auto else 1},gain_automatic={int(auto)},'
         f'white_balance_automatic={int(auto)}'],
    ):
        subprocess.run(cmd, check=True)
