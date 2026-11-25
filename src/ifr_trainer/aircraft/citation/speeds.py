"""Business jet speeds shown on the PFD speed tape (all CAS, knots).

The PFD came from the A320, so the field names are the Airbus ones. Without protections they
mean: ``vls`` the low-speed cue (1.3 VS1G, VREF in the landing configuration), ``v_alpha_prot``
the stall warning, ``v_alpha_max`` the 1 g stall, all in the current configuration.
"""

import math
from dataclasses import dataclass

from ifr_trainer.sim.airframe import Airframe
from ifr_trainer.sim.atmosphere import RHO0, cas_from_tas, isa
from ifr_trainer.sim.units import G, KT

LOW_SPEED_CUE = 1.3  # x VS1G
STALL_WARNING = 1.07  # x VS1G, about the 2 deg angle-of-attack margin of the STALL warning


@dataclass(frozen=True)
class SpeedLimits:
    vs1g: float
    vls: float
    v_alpha_prot: float
    v_alpha_max: float
    vmax: float
    green_dot: float | None = None


def vs1g_kt(ac: Airframe, mass: float, cl_max: float) -> float:
    return math.sqrt(2.0 * mass * G / (RHO0 * ac.wing_area_m2 * cl_max)) / KT


def speed_limits(ac: Airframe, mass: float, flap_pos: float, gear_pos: float, h_m: float) -> SpeedLimits:
    cfg = ac.flap_config_at(flap_pos)
    vs = vs1g_kt(ac, mass, cfg.cl_max)
    air = isa(h_m)
    vmax = min(ac.vmo_kt, cas_from_tas(ac.mmo * air.speed_of_sound, air) / KT)
    if cfg.vfe_kt is not None:
        vmax = min(vmax, cfg.vfe_kt)
    if gear_pos > 0.0:
        vmax = min(vmax, ac.gear.vle_kt)
    return SpeedLimits(vs, LOW_SPEED_CUE * vs, STALL_WARNING * vs, vs, vmax)
