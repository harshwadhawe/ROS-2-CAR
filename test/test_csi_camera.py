"""Self-check for the raw Bayer unpack. Run on the Pi with ROS sourced:
PYTHONPATH=.:$PYTHONPATH python3 test/test_csi_camera.py"""
import numpy as np

from pistar_car.csi_camera import debayer, FRAME_BYTES, W, H, BYTES_PER_LINE

# A flat grey frame: every 4 MSB bytes 128, the 5th (LSBs) junk. Unpacking
# must drop the LSB byte, so the output stays flat grey, not striped.
raw = np.tile(np.array([128, 128, 128, 128, 255], np.uint8), FRAME_BYTES // 5)
assert raw.size == FRAME_BYTES == BYTES_PER_LINE * H
out = debayer(raw.tobytes(), (160, 120))
assert out.shape == (120, 160, 3)
assert abs(int(out.min()) - 128) <= 1 and abs(int(out.max()) - 128) <= 1, (out.min(), out.max())

print('csi_camera unpack: all checks passed')
