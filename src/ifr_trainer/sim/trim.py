"""Trim: angle of attack, pitch trim and thrust for steady flight on a given flight path."""

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .aero import AeroConfig, AeroModel, Surfaces
from .airframe import Airframe
from .atmosphere import Air, isa
from .units import G

if TYPE_CHECKING:
    from .aircraft import Propulsion


@dataclass(frozen=True)
class Trim:
    alpha: float
    pitch_trim: float
    thrust: float
    lever: float


def lever_for_thrust(propulsion: "Propulsion", air: Air, mach: float, thrust: float) -> float:
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if propulsion.steady_thrust(mid, air, mach) < thrust:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def trim_flight(
    ac: Airframe,
    aero: AeroModel,
    propulsion: "Propulsion",
    cfg: AeroConfig,
    mass: float,
    h_m: float,
    tas: float,
    gamma: float = 0.0,
) -> Trim:
    """Wings-level steady flight with the elevator neutral (pitch trim carries the load)."""
    air = isa(h_m)
    qs = 0.5 * air.density * tas * tas * ac.wing_area_m2
    weight = mass * G
    mach = tas / air.speed_of_sound
    z_thrust = propulsion.thrust_line_z_m
    alpha, trim, thrust = 0.05, 0.0, 0.0
    for _ in range(200):
        untrimmed = aero.coefficients(cfg, alpha, 0.0, 0.0, 0.0, 0.0, 0.0, tas, mach, Surfaces())
        trim = -(untrimmed.pitch + z_thrust * thrust / (qs * ac.mac_m)) / ac.aero.cm_pitch_trim
        c = aero.coefficients(cfg, alpha, 0.0, 0.0, 0.0, 0.0, 0.0, tas, mach, Surfaces(pitch_trim=trim))
        thrust = (c.cd * qs + weight * math.sin(gamma)) / math.cos(alpha)
        cl_required = (weight * math.cos(gamma) - thrust * math.sin(alpha)) / qs
        new_alpha = (cl_required - ac.aero.cl_pitch_trim * trim - cfg.cl0) / ac.cl_alpha_per_rad
        if abs(new_alpha - alpha) < 1e-10:
            break
        alpha = new_alpha
    return Trim(alpha, trim, thrust, lever_for_thrust(propulsion, air, mach, thrust))
