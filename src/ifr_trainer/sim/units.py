"""Unit conversion factors. The simulation works in SI; displays use aviation units."""

import math

G = 9.80665          # m/s^2
KT = 0.514444        # m/s per knot
FT = 0.3048          # m per foot
NM = 1852.0          # m per nautical mile
FPM = FT / 60.0      # m/s per foot per minute
DEG = math.pi / 180.0


def wrap_deg(angle: float) -> float:
    """Wrap an angle in degrees to [0, 360)."""
    return angle % 360.0


def wrap_180(angle: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0
