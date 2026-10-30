"""Thrust trim: find the lever position that holds a given flight path at constant speed."""

import math

from .engines import Engines
from .flight_model import AeroConfig, AircraftState, PointMassModel
from .units import G


def trim_lever(
    model: PointMassModel, engines: Engines, s: AircraftState, aero: AeroConfig, mass: float, gamma: float = 0.0
) -> float:
    f = model.forces(s, aero, mass, nz=math.cos(gamma))
    required = f.drag + mass * G * math.sin(gamma)
    mach = s.tas / f.air.speed_of_sound
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if engines.thrust_at(engines.lever_to_n1(mid), f.air, mach) < required:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)
