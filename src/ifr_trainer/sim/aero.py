"""Aerodynamic coefficients from stability derivatives, with a simple stall model."""

import math
from dataclasses import dataclass

from .aircraft import AircraftData


@dataclass(frozen=True)
class AeroConfig:
    """Configuration-dependent aero data (flaps, gear, speedbrake already folded in)."""

    cl0: float
    cl_max: float
    cd0: float
    cm0: float


@dataclass
class Surfaces:
    """Control surface positions in radians (pilot sense) and spoiler extension 0..1."""

    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    ths: float = 0.0
    nosewheel: float = 0.0
    speedbrake: float = 0.0
    ground_spoiler: float = 0.0


@dataclass(frozen=True)
class Coefficients:
    cl: float
    cd: float
    cy: float
    roll: float
    pitch: float
    yaw: float
    alpha_stall: float


class AeroModel:
    def __init__(self, aircraft: AircraftData):
        self.ac = aircraft

    def alpha_stall(self, cfg: AeroConfig) -> float:
        return (cfg.cl_max - cfg.cl0) / self.ac.cl_alpha_per_rad

    def wing_lift(self, cfg: AeroConfig, alpha: float) -> float:
        a = self.ac.aero
        alpha_stall = self.alpha_stall(cfg)
        if alpha <= alpha_stall:
            return max(cfg.cl0 + self.ac.cl_alpha_per_rad * alpha, a.cl_min)
        # Past the stall lift falls off, but never below a flat-plate value.
        return max(cfg.cl_max + a.cl_post_stall_slope * (alpha - alpha_stall), 0.9 * math.sin(2 * alpha))

    def coefficients(
        self,
        cfg: AeroConfig,
        alpha: float,
        beta: float,
        alpha_dot: float,
        p: float,
        q: float,
        r: float,
        tas: float,
        mach: float,
        s: Surfaces,
        height_m: float = 1000.0,
    ) -> Coefficients:
        """``height_m`` is the wing's height above the ground, for ground effect."""
        ac, a = self.ac, self.ac.aero
        v2 = 2.0 * max(tas, 1.0)
        p_hat, r_hat = p * ac.span_m / v2, r * ac.span_m / v2
        q_hat, ad_hat = q * ac.mac_m / v2, alpha_dot * ac.mac_m / v2

        # Ground effect (Wieselsberger): less induced drag and a little more lift within a span of the ground.
        h = (16.0 * max(height_m, 0.1) / ac.span_m) ** 2
        ground_effect = h / (1.0 + h)
        cl_wing = self.wing_lift(cfg, alpha) * (1.0 + 0.1 * (1.0 - ground_effect))
        cl = (
            cl_wing
            + a.cl_q * q_hat
            + a.cl_elevator * s.elevator
            + a.cl_ths * s.ths
            + a.speedbrake_cl * s.speedbrake
            + a.ground_spoiler_cl * s.ground_spoiler
        )
        alpha_stall = self.alpha_stall(cfg)
        wave = 20.0 * (mach - ac.mach_crit) ** 4 if mach > ac.mach_crit else 0.0
        separated = 1.2 * math.sin(alpha - alpha_stall) ** 2 if alpha > alpha_stall else 0.0
        cd = (
            cfg.cd0
            + ac.induced_drag_k * ground_effect * cl_wing * cl_wing
            + wave
            + separated
            + a.cd_beta * beta * beta
            + a.ground_spoiler_cd * s.ground_spoiler
        )
        cy = a.cy_beta * beta + a.cy_rudder * s.rudder
        roll = a.roll_beta * beta + a.roll_p * p_hat + a.roll_r * r_hat + a.roll_aileron * s.aileron + a.roll_rudder * s.rudder
        pitch = (
            cfg.cm0
            + a.cm_alpha * alpha
            + a.cm_q * q_hat
            + a.cm_alphadot * ad_hat
            + a.cm_elevator * s.elevator
            + a.cm_ths * s.ths
        )
        yaw = a.yaw_beta * beta + a.yaw_p * p_hat + a.yaw_r * r_hat + a.yaw_aileron * s.aileron + a.yaw_rudder * s.rudder
        return Coefficients(cl, cd, cy, roll, pitch, yaw, alpha_stall)
