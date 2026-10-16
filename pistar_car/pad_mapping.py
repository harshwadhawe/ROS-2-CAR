"""PS4 pad -> drive values, no ROS. teleop_joy.py is the ROS wrapper."""


def scaled(axes, index, scale, limit):
    """One axis -> a clamped command. Pure, so it is checkable without a graph."""
    value = axes[index] if 0 <= index < len(axes) else 0.0
    return max(-limit, min(limit, value * scale * limit))


def toggled(auto, was_down, down):
    """Flip the mode on the press edge only, so a held button flips once."""
    return not auto if down and not was_down else auto
