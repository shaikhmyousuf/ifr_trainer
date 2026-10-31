"""Conventional flight controls: cables and pushrods, no computer between stick and surfaces.

Shared by the aircraft without fly-by-wire (Cessna 172, Citation). The stick moves the
elevator and ailerons directly. Pitch trim is an offset added to the elevator (a trim tab,
as in JSBSim's c172p), run by the trim switch, so releasing the stick returns the aircraft
to its trimmed speed. The trainer has no rudder pedals: an automatic rudder
stands in for a pilot who keeps the ball centred (zero sideslip) in turns.
"""

import math
from dataclasses import replace

from .aero import AeroConfig, AeroModel, Surfaces
from .airframe import Airframe
from .flight_model import AircraftState
from .units import DEG, G

MIN_DYNAMIC_PRESSURE = 50.0  # Pa; below this the automatic rudder holds still


def _toward(value: float, target: float, max_step: float) -> float:
    return value + max(-max_step, min(max_step, target - value))


def _shape(stick: float, expo: float) -> float:
    return (1.0 - expo) * stick + expo * stick**3


class DirectControls:
    K_BETA = 1.0  # 1/s: sideslip error -> yaw rate demand (automatic rudder)
    K_R = 2.5  # 1/s: yaw rate error -> yaw acceleration demand

    def __init__(self, airframe: Airframe, aero: AeroModel):
        self.ac = airframe
        self.aero = aero

    def update(
        self,
        dt: float,
        inputs,
        s: AircraftState,
        surfaces: Surfaces,
        cfg: AeroConfig,
        mass: float,
        thrust: float,
        cas_kt: float,
        flaps_extended: bool,
    ) -> None:
        lim = self.ac.controls
        up, down = lim.elevator_up_deg * DEG, lim.elevator_down_deg * DEG

        # Trim switch: moves the trim offset at a fixed rate while held.
        trim_step = inputs.pitch_trim * lim.pitch_trim_rate_deg_s * DEG * dt
        surfaces.pitch_trim = max(-lim.pitch_trim_down_deg * DEG, min(lim.pitch_trim_up_deg * DEG, surfaces.pitch_trim + trim_step))

        # The stick moves the elevator around the trimmed position; full stick still reaches the stops.
        pitch, roll = _shape(inputs.stick_pitch, lim.stick_expo), _shape(inputs.stick_roll, lim.stick_expo)
        stick = pitch * (up if pitch > 0 else down)
        elevator = max(-down, min(up, surfaces.pitch_trim + stick)) - surfaces.pitch_trim
        surfaces.elevator = _toward(surfaces.elevator, elevator, lim.elevator_rate_deg_s * DEG * dt)
        surfaces.aileron = _toward(surfaces.aileron, roll * lim.aileron_deg * DEG, lim.aileron_rate_deg_s * DEG * dt)
        surfaces.rudder = _toward(surfaces.rudder, self._auto_rudder(s, surfaces, cfg, mass), lim.rudder_rate_deg_s * DEG * dt)

    def _auto_rudder(self, s: AircraftState, surfaces: Surfaces, cfg: AeroConfig, mass: float) -> float:
        """Rudder for the yaw rate of a coordinated turn, by inverting the yaw moment model."""
        if s.dynamic_pressure < MIN_DYNAMIC_PRESSURE:
            return surfaces.rudder
        ac, d = self.ac, self.ac.aero
        inertia = ac.inertia_for(mass)
        v = max(s.tas, 1.0)
        r_cmd = G * math.sin(s.phi) * math.cos(s.theta) / v + self.K_BETA * s.beta
        n_req = inertia.izz * self.K_R * (r_cmd - s.r) - (inertia.ixx - inertia.iyy) * s.p * s.q
        base = self.aero.coefficients(cfg, s.alpha, s.beta, s.alpha_dot, s.p, s.q, s.r, s.tas, s.mach, replace(surfaces, rudder=0.0))
        rudder = (n_req / (s.dynamic_pressure * ac.wing_area_m2 * ac.span_m) - base.yaw) / d.yaw_rudder
        limit = ac.controls.rudder_deg * DEG
        return max(-limit, min(limit, rudder))
