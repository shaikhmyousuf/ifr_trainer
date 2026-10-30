"""International Standard Atmosphere (to 20 km) and airspeed conversions."""

import math
from dataclasses import dataclass

from .units import G

R_AIR = 287.05287
GAMMA = 1.4
T0 = 288.15
P0 = 101325.0
RHO0 = 1.225
A0 = math.sqrt(GAMMA * R_AIR * T0)
LAPSE = 0.0065
H_TROPOPAUSE = 11000.0
T_TROPOPAUSE = T0 - LAPSE * H_TROPOPAUSE
P_TROPOPAUSE = P0 * (T_TROPOPAUSE / T0) ** (G / (LAPSE * R_AIR))


@dataclass(frozen=True)
class Air:
    temperature: float  # K
    pressure: float  # Pa
    density: float  # kg/m^3
    speed_of_sound: float  # m/s


def isa(h_m: float, isa_dev: float = 0.0) -> Air:
    """Air properties at geopotential altitude ``h_m`` with an ISA temperature deviation."""
    if h_m <= H_TROPOPAUSE:
        t_std = T0 - LAPSE * h_m
        p = P0 * (t_std / T0) ** (G / (LAPSE * R_AIR))
    else:
        p = P_TROPOPAUSE * math.exp(-G * (h_m - H_TROPOPAUSE) / (R_AIR * T_TROPOPAUSE))
    t = (t_std if h_m <= H_TROPOPAUSE else T_TROPOPAUSE) + isa_dev
    return Air(t, p, p / (R_AIR * t), math.sqrt(GAMMA * R_AIR * t))


def cas_from_tas(tas: float, air: Air) -> float:
    """Calibrated airspeed from true airspeed (subsonic, m/s)."""
    mach = tas / air.speed_of_sound
    qc = air.pressure * ((1.0 + 0.2 * mach * mach) ** 3.5 - 1.0)
    return A0 * math.sqrt(5.0 * ((qc / P0 + 1.0) ** (2.0 / 7.0) - 1.0))


def tas_from_cas(cas: float, air: Air) -> float:
    """True airspeed from calibrated airspeed (subsonic, m/s)."""
    qc = P0 * ((1.0 + 0.2 * (cas / A0) ** 2) ** 3.5 - 1.0)
    mach = math.sqrt(5.0 * ((qc / air.pressure + 1.0) ** (2.0 / 7.0) - 1.0))
    return mach * air.speed_of_sound
