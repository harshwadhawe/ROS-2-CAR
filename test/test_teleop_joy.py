"""Self-check for the joy axis mapping. Run: colcon test, or python3 test/test_teleop_joy.py"""
from pistar_car.teleop_joy import scaled, toggled

LIMIT = 0.4


def test_axis_mapping():
    # Stick full right reports +1 on joydev; with scale -1 that must steer right
    # (negative under REP-103), and full left must steer left.
    assert scaled([1.0], 0, -1.0, LIMIT) == -LIMIT
    assert scaled([-1.0], 0, -1.0, LIMIT) == LIMIT
    assert scaled([0.0], 0, -1.0, LIMIT) == 0.0

    # Half deflection is half command.
    assert abs(scaled([0.5], 0, -1.0, LIMIT) - (-0.2)) < 1e-9

    # A pad reporting beyond full scale must not exceed the limit.
    assert scaled([-2.0], 0, -1.0, LIMIT) == LIMIT
    assert scaled([2.0], 0, -1.0, LIMIT) == -LIMIT

    # A missing axis is zero, never an IndexError: a short Joy message from an
    # unexpected pad must not crash the node mid-drive.
    assert scaled([], 4, -1.0, LIMIT) == 0.0
    assert scaled([0.0, 0.0], 4, -1.0, LIMIT) == 0.0
    assert scaled([1.0], -1, -1.0, LIMIT) == 0.0

    # Square flips the mode on the press edge only: holding it, or releasing
    # it, must not flip again.
    assert toggled(False, False, True) is True          # press: manual -> auto
    assert toggled(True, True, True) is True            # held: stays auto
    assert toggled(True, True, False) is True           # release: stays auto
    assert toggled(True, False, True) is False          # press again: back to manual
    assert toggled(False, False, False) is False

    print('teleop_joy mapping: all checks passed')


if __name__ == '__main__':
    test_axis_mapping()
