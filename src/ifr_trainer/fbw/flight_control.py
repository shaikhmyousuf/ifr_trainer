"""Flight control computer: Airbus-style normal law.

The normal law's commands (load factor, roll rate) become surface deflections
through dynamic inversion: the computer uses the aircraft's own aero model to find
the deflection that produces the wanted angular acceleration. The rudder is not
flown by the pilot; it coordinates turns (zero sideslip) automatically. The THS
trims automatically so the elevator returns to neutral.
"""

import math
from dataclasses import replace

from ..sim.aero import AeroConfig, AeroModel, Surfaces
from ..sim.aircraft import AircraftData
from ..sim.flight_model import AircraftState
from ..sim.units import G
from .normal_law import NormalLawLimits, pitch_law, roll_law

DEG = math.pi / 180.0
MIN_DYNAMIC_PRESSURE = 200.0  # Pa; below this the law holds the surfaces still


def _toward(value: float, target: float, max_step: float) -> float:
    return value + max(-max_step, min(max_step, target - value))


class FlightControlComputer:
    C_STAR_VCO = 120.0  # m/s: C* crossover speed, where load factor and pitch rate weigh equally
    K_Q = 3.0  # 1/s: pitch rate error -> pitch acceleration demand
    K_P = 5.0  # 1/s: roll rate error -> roll acceleration demand
    K_BETA = 1.0  # 1/s: sideslip error -> yaw rate demand
    K_R = 2.5  # 1/s: yaw rate error -> yaw acceleration demand
    TRIM_GAIN = 0.5  # THS rate per unit of elevator deflection, 1/s
    ALPHA_PROT_MARGIN = 1.5 * DEG  # basic high angle-of-attack limit below the stall (full protection: M3)

    def __init__(self, aircraft: AircraftData, aero: AeroModel, limits: NormalLawLimits):
        self.ac = aircraft
        self.aero = aero
        self.limits = limits
        self.alpha_limited = False

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
        """Move ``surfaces`` (elevator, aileron, rudder, THS) for this step."""
        if s.dynamic_pressure < MIN_DYNAMIC_PRESSURE:
            return
        lim = self.ac.controls
        elevator, aileron, rudder = self._normal_law(inputs, s, surfaces, cfg, mass, thrust, flaps_extended)
        up, down = lim.elevator_up_deg * DEG, lim.elevator_down_deg * DEG
        aileron_max = lim.aileron_deg * DEG
        rudder_max = lim.rudder_limit_deg(cas_kt) * DEG
        surfaces.elevator = _toward(surfaces.elevator, max(-down, min(up, elevator)), lim.elevator_rate_deg_s * DEG * dt)
        surfaces.aileron = _toward(
            surfaces.aileron, max(-aileron_max, min(aileron_max, aileron)), lim.aileron_rate_deg_s * DEG * dt
        )
        surfaces.rudder = _toward(
            surfaces.rudder, max(-rudder_max, min(rudder_max, rudder)), lim.rudder_rate_deg_s * DEG * dt
        )

        # Auto-trim: the THS offloads the elevator. Frozen beyond 33 degrees of bank, outside
        # 0.5 to 1.25 g, and while the high angle-of-attack limit holds the stick back.
        if abs(s.phi) < self.limits.neutral_bank and 0.5 < s.n_path < 1.25 and not self.alpha_limited:
            rate = lim.ths_rate_deg_s * DEG
            ths_rate = max(-rate, min(rate, self.TRIM_GAIN * surfaces.elevator))
            surfaces.ths = max(-lim.ths_down_deg * DEG, min(lim.ths_up_deg * DEG, surfaces.ths + ths_rate * dt))

    def _normal_law(
        self,
        inputs,
        s: AircraftState,
        surfaces: Surfaces,
        cfg: AeroConfig,
        mass: float,
        thrust: float,
        flaps_extended: bool,
    ) -> tuple[float, float, float]:
        ac, d = self.ac, self.ac.aero
        inertia = ac.inertia_for(mass)
        qs = s.dynamic_pressure * ac.wing_area_m2
        v = max(s.tas, 1.0)
        p, q, r = s.p, s.q, s.r
        sin_phi, cos_phi = math.sin(s.phi), math.cos(s.phi)

        # Pitch: C* command -> pitch rate -> pitch acceleration. C* blends load factor
        # with pitch rate, C* = nz + (VCO/g) q. The pitch rate below makes the measured C*
        # equal the stick's demand; in steady flight that gives the stick's load factor,
        # and at stick neutral it holds the flight path. Pitch rate dominates the blend
        # at low speed, so the nose stays close to where the pilot leaves it.
        nz_cmd = pitch_law(inputs.stick_pitch, s.gamma, s.phi, s.theta, flaps_extended, self.limits)
        dn_dalpha = max(ac.cl_alpha_per_rad * qs / (mass * G), 0.5)
        alpha_max = self.aero.alpha_stall(cfg) - self.ALPHA_PROT_MARGIN
        n = s.n_path
        alpha_room = n + (alpha_max - s.alpha) * dn_dalpha
        self.alpha_limited = nz_cmd > alpha_room
        nz_cmd = min(nz_cmd, alpha_room)
        q_cmd = G / v * (nz_cmd - math.cos(s.gamma) * cos_phi) + G / self.C_STAR_VCO * (nz_cmd - n)
        if abs(s.phi) < self.limits.neutral_bank:
            # Bank compensation feedforward: raise alpha as the bank grows, before the
            # load factor falls short, so rolling into a turn does not sag the flight path.
            phi_dot = p + (q * sin_phi + r * cos_phi) * math.tan(s.theta)
            q_cmd += math.cos(s.gamma) * sin_phi / cos_phi**2 * phi_dot / dn_dalpha
        m_req = inertia.iyy * self.K_Q * (q_cmd - q) - (inertia.izz - inertia.ixx) * p * r

        # Roll: roll rate command (Euler bank rate) -> body roll rate -> roll acceleration.
        phi_dot_cmd = roll_law(inputs.stick_roll, s.phi, self.limits)
        p_cmd = phi_dot_cmd - (q * sin_phi + r * cos_phi) * math.tan(s.theta)
        l_req = inertia.ixx * self.K_P * (p_cmd - p) - (inertia.iyy - inertia.izz) * q * r

        # Yaw: coordinate the turn (zero sideslip).
        r_cmd = G * sin_phi * math.cos(s.theta) / v + self.K_BETA * s.beta
        n_req = inertia.izz * self.K_R * (r_cmd - r) - (inertia.ixx - inertia.iyy) * p * q

        neutral = replace(surfaces, elevator=0.0, aileron=0.0, rudder=0.0)
        base = self.aero.coefficients(cfg, s.alpha, s.beta, s.alpha_dot, p, q, r, s.tas, s.mach, neutral)
        elevator = ((m_req - ac.engines.z_m * thrust) / (qs * ac.mac_m) - base.pitch) / d.cm_elevator
        # Aileron and rudder both affect roll and yaw: solve the 2x2 system.
        cl_req = l_req / (qs * ac.span_m) - base.roll
        cn_req = n_req / (qs * ac.span_m) - base.yaw
        det = d.roll_aileron * d.yaw_rudder - d.roll_rudder * d.yaw_aileron
        aileron = (cl_req * d.yaw_rudder - d.roll_rudder * cn_req) / det
        rudder = (d.roll_aileron * cn_req - d.yaw_aileron * cl_req) / det
        return elevator, aileron, rudder
