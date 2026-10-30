"""Point-mass flight model in wind axes (milestone 1).

The aircraft is a point with velocity along its flight path. The control law
commands a load factor and a roll rate; the airframe follows them with a short
lag, limited by the lift the wing can produce (CLmax), so the aircraft can stall.
"""

import math
from dataclasses import dataclass

from .aircraft import AircraftData
from .atmosphere import Air, isa
from .units import G

MAX_GAMMA = math.radians(80.0)


@dataclass
class AircraftState:
    x_m: float = 0.0  # east of origin
    y_m: float = 0.0  # north of origin
    h_m: float = 0.0  # MSL
    tas: float = 100.0  # m/s
    gamma: float = 0.0  # flight path angle, rad
    psi: float = 0.0  # heading, rad (no sideslip in this model)
    phi: float = 0.0  # bank, rad
    roll_rate: float = 0.0  # rad/s
    nz: float = 1.0  # load factor normal to the flight path
    # Derived each step.
    alpha: float = 0.0
    theta: float = 0.0
    mach: float = 0.0
    lift_limited: bool = False


@dataclass(frozen=True)
class AeroConfig:
    cl0: float
    cl_max: float
    cd0: float


@dataclass(frozen=True)
class Forces:
    air: Air
    dynamic_pressure: float
    cl: float
    drag: float
    nz_available: float


class PointMassModel:
    NZ_TAU_S = 0.5  # stands in for short-period response + C* law
    ROLL_TAU_S = 0.25
    MIN_TAS = 15.0

    def __init__(self, aircraft: AircraftData):
        self.ac = aircraft

    def forces(self, s: AircraftState, aero: AeroConfig, mass: float, nz: float | None = None) -> Forces:
        """Aerodynamic forces at the current state (or at load factor ``nz``)."""
        ac = self.ac
        air = isa(s.h_m)
        q = 0.5 * air.density * s.tas**2
        qs = q * ac.wing_area_m2
        weight = mass * G
        cl = (s.nz if nz is None else nz) * weight / qs
        mach = s.tas / air.speed_of_sound
        wave = 20.0 * (mach - ac.mach_crit) ** 4 if mach > ac.mach_crit else 0.0
        cd = aero.cd0 + ac.induced_drag_k * cl * cl + wave
        return Forces(air, q, cl, cd * qs, aero.cl_max * qs / weight)

    def update_derived(self, s: AircraftState, aero: AeroConfig, mass: float) -> Forces:
        f = self.forces(s, aero, mass)
        s.alpha = (f.cl - aero.cl0) / self.ac.cl_alpha_per_rad
        s.theta = s.gamma + s.alpha * math.cos(s.phi)
        s.mach = s.tas / f.air.speed_of_sound
        return f

    def step(
        self,
        s: AircraftState,
        aero: AeroConfig,
        mass: float,
        thrust: float,
        nz_cmd: float,
        roll_rate_cmd: float,
        wind_en: tuple[float, float],
        dt: float,
    ) -> None:
        f = self.forces(s, aero, mass)
        target = min(max(nz_cmd, -0.5 * f.nz_available), f.nz_available)
        s.lift_limited = nz_cmd > f.nz_available
        s.nz += (target - s.nz) * min(dt / self.NZ_TAU_S, 1.0)
        s.nz = min(s.nz, f.nz_available)
        drag = self.forces(s, aero, mass).drag

        s.roll_rate += (roll_rate_cmd - s.roll_rate) * min(dt / self.ROLL_TAU_S, 1.0)
        s.phi += s.roll_rate * dt

        cos_g = math.cos(s.gamma)
        dv = (thrust - drag) / mass - G * math.sin(s.gamma)
        dgamma = G / s.tas * (s.nz * math.cos(s.phi) - cos_g)
        dpsi = G * s.nz * math.sin(s.phi) / (s.tas * cos_g)

        horizontal = s.tas * cos_g
        s.x_m += (horizontal * math.sin(s.psi) + wind_en[0]) * dt
        s.y_m += (horizontal * math.cos(s.psi) + wind_en[1]) * dt
        s.h_m += s.tas * math.sin(s.gamma) * dt

        s.tas = max(s.tas + dv * dt, self.MIN_TAS)
        s.gamma = min(max(s.gamma + dgamma * dt, -MAX_GAMMA), MAX_GAMMA)
        s.psi = (s.psi + dpsi * dt) % math.tau
        self.update_derived(s, aero, mass)
