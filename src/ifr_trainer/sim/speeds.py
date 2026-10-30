"""Characteristic speeds shown on the speed tape (all CAS, knots)."""

import math
from dataclasses import dataclass

from .aircraft import AircraftData
from .atmosphere import RHO0, cas_from_tas, isa
from .units import FT, G, KT


@dataclass(frozen=True)
class SpeedLimits:
    vs1g: float
    vls: float
    v_alpha_prot: float
    v_alpha_max: float
    vmax: float
    green_dot: float | None


def vs1g_kt(ac: AircraftData, mass: float, cl_max: float) -> float:
    return math.sqrt(2.0 * mass * G / (RHO0 * ac.wing_area_m2 * cl_max)) / KT


def speed_limits(ac: AircraftData, mass: float, flap_pos: float, gear_pos: float, h_m: float) -> SpeedLimits:
    cfg = ac.flap_config_at(flap_pos)
    vs = vs1g_kt(ac, mass, cfg.cl_max)
    air = isa(h_m)
    vmax = min(ac.vmo_kt, cas_from_tas(ac.mmo * air.speed_of_sound, air) / KT)
    if cfg.vfe_kt is not None:
        vmax = min(vmax, cfg.vfe_kt)
    if gear_pos > 0.0:
        vmax = min(vmax, ac.gear.vle_kt)
    green_dot = None
    if flap_pos < 0.01:
        green_dot = 2.0 * mass / 1000.0 + 85.0 + max(0.0, (h_m / FT - 20000.0) / 1000.0)
    return SpeedLimits(vs, cfg.vls_factor * vs, 1.10 * vs, 1.03 * vs, vmax, green_dot)
