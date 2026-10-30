"""Trim: angle of attack, THS and thrust for steady flight on a given flight path."""

import math
from dataclasses import dataclass

from .aero import AeroConfig, AeroModel, Surfaces
from .aircraft import AircraftData
from .atmosphere import Air, isa
from .engines import Engines
from .units import G


@dataclass(frozen=True)
class Trim:
    alpha: float
    ths: float
    thrust: float
    lever: float


def lever_for_thrust(engines: Engines, air: Air, mach: float, thrust: float) -> float:
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if engines.thrust_at(engines.lever_to_n1(mid), air, mach) < thrust:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def trim_flight(
    ac: AircraftData,
    aero: AeroModel,
    engines: Engines,
    cfg: AeroConfig,
    mass: float,
    h_m: float,
    tas: float,
    gamma: float = 0.0,
) -> Trim:
    """Wings-level steady flight with the elevator neutral (the THS carries the trim)."""
    air = isa(h_m)
    qs = 0.5 * air.density * tas * tas * ac.wing_area_m2
    weight = mass * G
    mach = tas / air.speed_of_sound
    alpha, ths, thrust = 0.05, 0.0, 0.0
    for _ in range(200):
        untrimmed = aero.coefficients(cfg, alpha, 0.0, 0.0, 0.0, 0.0, 0.0, tas, mach, Surfaces())
        ths = -(untrimmed.pitch + ac.engines.z_m * thrust / (qs * ac.mac_m)) / ac.aero.cm_ths
        c = aero.coefficients(cfg, alpha, 0.0, 0.0, 0.0, 0.0, 0.0, tas, mach, Surfaces(ths=ths))
        thrust = (c.cd * qs + weight * math.sin(gamma)) / math.cos(alpha)
        cl_required = (weight * math.cos(gamma) - thrust * math.sin(alpha)) / qs
        new_alpha = (cl_required - ac.aero.cl_ths * ths - cfg.cl0) / ac.cl_alpha_per_rad
        if abs(new_alpha - alpha) < 1e-10:
            break
        alpha = new_alpha
    return Trim(alpha, ths, thrust, lever_for_thrust(engines, air, mach, thrust))
