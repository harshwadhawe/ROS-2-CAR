"""Self-check for the raw Bayer unpack. Run: colcon test, or python3 test/test_csi_camera.py"""
import numpy as np

from pistar_car.ov5647 import debayer, vblank_for, FRAME_BYTES, H, BYTES_PER_LINE, PIXEL_RATE, LINE_PIXELS


def test_unpack():
    # A flat grey frame: every 4 MSB bytes 128, the 5th (LSBs) junk. Unpacking
    # must drop the LSB byte, so the output stays flat grey, not striped.
    raw = np.tile(np.array([128, 128, 128, 128, 255], np.uint8), FRAME_BYTES // 5)
    assert raw.size == FRAME_BYTES == BYTES_PER_LINE * H
    out = debayer(raw.tobytes(), (160, 120))
    assert out.shape == (120, 160, 3)
    assert abs(int(out.min()) - 128) <= 1 and abs(int(out.max()) - 128) <= 1, (out.min(), out.max())

    # Minimum blanking is the sensor's native ~62.5 Hz (measured); 20 Hz must
    # land within half a percent, and silly rates clamp to the control range.
    assert vblank_for(1000) == 24
    fps = PIXEL_RATE / (LINE_PIXELS * (H + vblank_for(20.0)))
    assert abs(fps - 20.0) < 0.1, fps
    assert vblank_for(0.1) == 32287

    print('csi_camera unpack: all checks passed')


if __name__ == '__main__':
    test_unpack()
