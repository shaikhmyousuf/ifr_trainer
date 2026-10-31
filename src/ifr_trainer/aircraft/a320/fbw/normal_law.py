"""Simplified Airbus normal law for the point-mass model.

Pitch: the stick commands load factor. Stick neutral holds the flight path,
with automatic bank compensation up to 33 degrees.
Roll: the stick commands roll rate. Stick neutral holds the bank angle up to
33 degrees; beyond that the aircraft rolls back to 33. Bank is limited to 67 degrees.
"""

import math
from dataclasses import dataclass

DEG = math.pi / 180.0


@dataclass(frozen=True)
class NormalLawLimits:
    nz_max_clean: float = 2.5
    nz_min_clean: float = -1.0
    nz_max_flaps: float = 2.0
    nz_min_flaps: float = 0.0
    max_roll_rate: float = 15.0 * DEG
    neutral_bank: float = 33.0 * DEG
    max_bank: float = 67.0 * DEG
    bank_return_rate: float = 5.0 * DEG
    pitch_up_limit: float = 30.0 * DEG
    pitch_down_limit: float = -15.0 * DEG
    pitch_limit_band: float = 5.0 * DEG
    pitch_limit_gain: float = 3.0  # g per rad beyond the limit


def pitch_law(
    stick: float, gamma: float, phi: float, theta: float, flaps_extended: bool, lim: NormalLawLimits
) -> float:
    """Load factor command from pitch stick (+1 = full back)."""
    nz_neutral = math.cos(gamma) / math.cos(min(abs(phi), lim.neutral_bank))
    nz_max = lim.nz_max_flaps if flaps_extended else lim.nz_max_clean
    nz_min = lim.nz_min_flaps if flaps_extended else lim.nz_min_clean
    if stick >= 0.0:
        cmd = nz_neutral + stick * (nz_max - nz_neutral)
    else:
        cmd = nz_neutral + stick * (nz_neutral - nz_min)

    # Pitch attitude protection: fade out authority near the limit, push back beyond it.
    up_margin = lim.pitch_up_limit - theta
    if cmd > nz_neutral and up_margin < lim.pitch_limit_band:
        cmd = nz_neutral + (cmd - nz_neutral) * max(up_margin / lim.pitch_limit_band, 0.0)
    if up_margin < 0.0:
        cmd += up_margin * lim.pitch_limit_gain
    down_margin = theta - lim.pitch_down_limit
    if cmd < nz_neutral and down_margin < lim.pitch_limit_band:
        cmd = nz_neutral + (cmd - nz_neutral) * max(down_margin / lim.pitch_limit_band, 0.0)
    if down_margin < 0.0:
        cmd -= down_margin * lim.pitch_limit_gain
    return min(max(cmd, nz_min), nz_max)


def roll_law(stick: float, phi: float, lim: NormalLawLimits) -> float:
    """Roll rate command from roll stick (+1 = full right)."""
    cmd = stick * lim.max_roll_rate
    excess = abs(phi) - lim.neutral_bank
    if excess > 0.0:
        # Roll back towards 33 degrees unless the stick holds the bank.
        held = max(stick if phi > 0 else -stick, 0.0)
        cmd -= math.copysign(min(lim.bank_return_rate, 2.0 * excess) * (1.0 - held), phi)
    # Soft bank limit: the roll rate fades to zero at max bank.
    cmd = min(cmd, (lim.max_bank - phi) * 2.0)
    cmd = max(cmd, (-lim.max_bank - phi) * 2.0)
    return cmd
