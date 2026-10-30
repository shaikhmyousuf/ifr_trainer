"""Landing gear: spring-damper struts with tyre friction, brakes and nosewheel steering."""

import math
from dataclasses import dataclass

from .aircraft import GearData
from .vec import Mat, Vec, cross, mat_t_vec, mat_vec

LONG_SLIP_MS = 0.5  # velocity over which tyre friction builds to its full value
LAT_SLIP_MS = 0.3


@dataclass(frozen=True)
class GearForces:
    force: Vec  # body axes, N
    moment: Vec  # body axes, N m
    loads: dict[str, float]  # strut normal force by leg name

    @property
    def main_on_ground(self) -> bool:
        return any(load > 0.0 for name, load in self.loads.items() if name != "nose")


NO_GEAR_FORCES = GearForces((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), {})


def point_height(r: Vec, rot: Mat, pos_ned: Vec, ground_d: float) -> float:
    """Height of body point ``r`` above the ground (negative = below)."""
    return ground_d - (pos_ned[2] + mat_vec(rot, r)[2])


def gear_forces(
    gear: GearData,
    rot: Mat,
    pos_ned: Vec,
    v_ned: Vec,
    omega: Vec,
    psi: float,
    steer: float,
    brakes: float,
    ground_d: float,
) -> GearForces:
    fx = fy = fz = mx = my = mz = 0.0
    loads: dict[str, float] = {}
    for leg in gear.legs:
        r = leg.position
        offset = mat_vec(rot, r)
        compression = pos_ned[2] + offset[2] - ground_d
        if compression <= 0.0:
            loads[leg.name] = 0.0
            continue
        rv = mat_vec(rot, cross(omega, r))
        vn, ve, vd = v_ned[0] + rv[0], v_ned[1] + rv[1], v_ned[2] + rv[2]
        normal = max(0.0, leg.stiffness * compression + leg.damping * vd)
        loads[leg.name] = normal

        heading = psi + (steer if leg.steerable else 0.0)
        ch, sh = math.cos(heading), math.sin(heading)
        v_long = vn * ch + ve * sh
        v_lat = -vn * sh + ve * ch
        mu_long = gear.rolling_friction + (gear.brake_friction * brakes if leg.braked else 0.0)
        f_long = -mu_long * normal * max(-1.0, min(1.0, v_long / LONG_SLIP_MS))
        f_lat = -gear.lateral_friction * normal * max(-1.0, min(1.0, v_lat / LAT_SLIP_MS))
        f_ned = (f_long * ch - f_lat * sh, f_long * sh + f_lat * ch, -normal)

        f = mat_t_vec(rot, f_ned)
        m = cross(r, f)
        fx, fy, fz = fx + f[0], fy + f[1], fz + f[2]
        mx, my, mz = mx + m[0], my + m[1], mz + m[2]
    return GearForces((fx, fy, fz), (mx, my, mz), loads)
