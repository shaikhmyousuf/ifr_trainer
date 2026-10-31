"""Six-degree-of-freedom rigid-body flight model.

World frame: x east, y north, h up (metres) for positions; NED for the attitude
quaternion. Body velocities (u, v, w) are relative to the ground; the aerodynamics
use the air-relative velocity (ground velocity minus a constant wind).
"""

import math
from dataclasses import dataclass

from .aero import AeroConfig, AeroModel, Surfaces
from .airframe import Airframe
from .atmosphere import isa
from .units import G
from .vec import Quat, euler_from_quat, mat_t_vec, mat_vec, quat_from_euler, quat_integrate, rotation_matrix

ALPHA_DOT_TAU_S = 0.05
MIN_AERO_TAS = 1.0


@dataclass
class AircraftState:
    x_m: float = 0.0  # east of origin
    y_m: float = 0.0  # north of origin
    h_m: float = 0.0  # MSL
    u: float = 0.0  # body-axis ground velocity, m/s
    v: float = 0.0
    w: float = 0.0
    quat: Quat = (1.0, 0.0, 0.0, 0.0)
    p: float = 0.0  # body rates, rad/s
    q: float = 0.0
    r: float = 0.0
    # Derived each step.
    phi: float = 0.0
    theta: float = 0.0
    psi: float = 0.0
    tas: float = 0.0
    alpha: float = 0.0
    beta: float = 0.0
    alpha_dot: float = 0.0
    gamma: float = 0.0
    mach: float = 0.0
    dynamic_pressure: float = 0.0
    nz: float = 1.0  # specific force along -z body, in g (what an accelerometer reads)
    ny: float = 0.0  # specific force along +y body, in g
    nx: float = 0.0  # specific force along +x body, in g
    v_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lift_limited: bool = False  # angle of attack beyond the stall

    @property
    def n_path(self) -> float:
        """Specific force normal to the flight path (lift plus the thrust component), in g."""
        return self.nz * math.cos(self.alpha) + self.nx * math.sin(self.alpha)

    def set_attitude(self, phi: float, theta: float, psi: float) -> None:
        self.quat = quat_from_euler(phi, theta, psi)
        self.phi, self.theta, self.psi = euler_from_quat(self.quat)


class SixDofModel:
    def __init__(self, aircraft: Airframe, thrust_line_z_m: float):
        self.ac = aircraft
        self.thrust_line_z_m = thrust_line_z_m  # thrust line below the CG
        self.aero = AeroModel(aircraft)

    def air_data(self, s: AircraftState, wind_ned: tuple[float, float, float], dt: float | None = None) -> None:
        """Update air-relative quantities and Euler angles from the primary state."""
        rot = rotation_matrix(s.quat)
        s.phi, s.theta, s.psi = euler_from_quat(s.quat)
        wb = mat_t_vec(rot, wind_ned)
        ua, va, wa = s.u - wb[0], s.v - wb[1], s.w - wb[2]
        s.tas = math.sqrt(ua * ua + va * va + wa * wa)
        if s.tas > MIN_AERO_TAS:
            alpha = math.atan2(wa, ua)
            if dt:
                rate = (alpha - s.alpha) / dt
                s.alpha_dot += (rate - s.alpha_dot) * min(dt / ALPHA_DOT_TAU_S, 1.0)
            s.alpha = alpha
            s.beta = math.asin(max(-1.0, min(1.0, va / s.tas)))
        else:
            s.alpha = s.beta = s.alpha_dot = 0.0
        air = isa(s.h_m)
        s.mach = s.tas / air.speed_of_sound
        s.dynamic_pressure = 0.5 * air.density * s.tas * s.tas
        s.v_ned = mat_vec(rot, (s.u, s.v, s.w))
        s.gamma = math.asin(max(-1.0, min(1.0, -s.v_ned[2] / s.tas))) if s.tas > MIN_AERO_TAS else 0.0

    def step(
        self,
        s: AircraftState,
        cfg: AeroConfig,
        surfaces: Surfaces,
        mass: float,
        thrust: float,
        wind_ned: tuple[float, float, float],
        dt: float,
    ) -> None:
        ac = self.ac
        rot = rotation_matrix(s.quat)
        qs = s.dynamic_pressure * ac.wing_area_m2
        c = self.aero.coefficients(cfg, s.alpha, s.beta, s.alpha_dot, s.p, s.q, s.r, s.tas, s.mach, surfaces)
        s.lift_limited = s.alpha > c.alpha_stall

        # Wind-axis lift/drag to body axes; side force is already in body axes.
        lift, drag, side = c.cl * qs, c.cd * qs, c.cy * qs
        ca, sa, cb, sb = math.cos(s.alpha), math.sin(s.alpha), math.cos(s.beta), math.sin(s.beta)
        fx = -drag * ca * cb + lift * sa + thrust
        fy = side - drag * sb
        fz = -drag * sa * cb - lift * ca
        mx = qs * ac.span_m * c.roll
        my = qs * ac.mac_m * c.pitch + self.thrust_line_z_m * thrust
        mz = qs * ac.span_m * c.yaw
        s.nz = -fz / (mass * G)
        s.ny = fy / (mass * G)
        s.nx = fx / (mass * G)

        grav = mat_t_vec(rot, (0.0, 0.0, mass * G))
        fx, fy, fz = fx + grav[0], fy + grav[1], fz + grav[2]

        inertia = ac.inertia_for(mass)
        p, q, r = s.p, s.q, s.r
        s.u += (fx / mass + r * s.v - q * s.w) * dt
        s.v += (fy / mass + p * s.w - r * s.u) * dt
        s.w += (fz / mass + q * s.u - p * s.v) * dt
        s.p += (mx + (inertia.iyy - inertia.izz) * q * r) / inertia.ixx * dt
        s.q += (my + (inertia.izz - inertia.ixx) * p * r) / inertia.iyy * dt
        s.r += (mz + (inertia.ixx - inertia.iyy) * p * q) / inertia.izz * dt

        s.quat = quat_integrate(s.quat, s.p, s.q, s.r, dt)
        vn, ve, vd = mat_vec(rotation_matrix(s.quat), (s.u, s.v, s.w))
        s.x_m += ve * dt
        s.y_m += vn * dt
        s.h_m -= vd * dt
        self.air_data(s, wind_ned, dt)
