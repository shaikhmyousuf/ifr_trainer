"""Piston engine and fixed-pitch propeller.

The engine makes torque from manifold pressure; the propeller absorbs power and makes
thrust according to its advance ratio J = V / (n D), from tabulated thrust and power
coefficients (JSBSim's ``prop_75in2f``, the C172's 75-inch two-blade propeller).
RPM is a state: it rises until the propeller absorbs what the engine delivers, so on a
fixed-pitch propeller the RPM also rises with airspeed, as in the real aircraft.
"""

import math
from dataclasses import dataclass

from ifr_trainer.sim.atmosphere import P0, Air

INHG = 3386.39  # Pa per inch of mercury
HP = 745.7  # W per horsepower


@dataclass(frozen=True)
class PistonData:
    max_power_hp: float  # at rated RPM, full throttle, sea level
    rated_rpm: float
    idle_map_inhg: float  # manifold pressure with the throttle closed
    max_map_inhg: float  # full throttle at sea level (intake losses below ambient)
    friction_rpm: float  # friction torque at rated RPM, as a fraction of rated torque; grows with RPM^2
    prop_diameter_m: float
    inertia_kgm2: float  # propeller and crankshaft
    z_m: float  # thrust line below the CG
    j_table: tuple[tuple[float, float, float], ...]  # (J, C_thrust, C_power)

    @classmethod
    def from_dict(cls, d: dict) -> "PistonData":
        d = dict(d)
        d["j_table"] = tuple((float(j), float(ct), float(cp)) for j, ct, cp in d["j_table"])
        return cls(**d)


def _interp(table: tuple[tuple[float, float, float], ...], j: float, col: int) -> float:
    if j <= table[0][0]:
        return table[0][col]
    for a, b in zip(table, table[1:]):
        if j <= b[0]:
            f = (j - a[0]) / (b[0] - a[0])
            return a[col] + (b[col] - a[col]) * f
    return table[-1][col]


class PistonPropeller:
    MIN_RPS = 5.0
    MAX_RPS = 60.0  # 3600 RPM: far beyond anything reached; bounds the steady-state search

    def __init__(self, data: PistonData):
        self.data = data
        self.rps = data.rated_rpm / 60.0 * 0.4
        self.detents: dict[str, float] = {}  # a throttle knob has no detents

    @property
    def thrust_line_z_m(self) -> float:
        return self.data.z_m

    @property
    def rpm(self) -> float:
        return self.rps * 60.0

    # --- Engine ------------------------------------------------------------------------

    def manifold_pressure_inhg(self, lever: float, air: Air) -> float:
        d = self.data
        full = air.pressure / P0 * d.max_map_inhg
        return min(d.idle_map_inhg, full) + max(0.0, full - d.idle_map_inhg) * min(max(lever, 0.0), 1.0)

    def engine_torque(self, lever: float, air: Air, rps: float) -> float:
        """Shaft torque in N m: indicated torque from manifold pressure, less friction that
        grows with RPM squared. At full throttle, sea level and rated RPM it is the rated torque."""
        d = self.data
        rated_torque = d.max_power_hp * HP / (d.rated_rpm / 60.0 * math.tau)
        charge = self.manifold_pressure_inhg(lever, air) / d.max_map_inhg
        return rated_torque * (charge * (1.0 + d.friction_rpm) - d.friction_rpm * (rps * 60.0 / d.rated_rpm) ** 2)

    # --- Propeller ---------------------------------------------------------------------

    def _advance_ratio(self, tas: float, rps: float) -> float:
        return max(tas, 0.0) / (max(rps, self.MIN_RPS) * self.data.prop_diameter_m)

    def prop_torque(self, rps: float, air: Air, tas: float) -> float:
        d = self.data
        cp = _interp(d.j_table, self._advance_ratio(tas, rps), 2)
        power = cp * air.density * rps**3 * d.prop_diameter_m**5
        return power / (rps * math.tau)

    def thrust_at(self, rps: float, air: Air, tas: float) -> float:
        d = self.data
        ct = _interp(d.j_table, self._advance_ratio(tas, rps), 1)
        return ct * air.density * rps**2 * d.prop_diameter_m**4

    def steady_rps(self, lever: float, air: Air, tas: float) -> float:
        """RPM (per second) at which the propeller absorbs exactly the engine's torque."""
        def surplus(rps: float) -> float:
            return self.engine_torque(lever, air, rps) - self.prop_torque(rps, air, tas)

        lo, hi = self.MIN_RPS, self.MAX_RPS
        if surplus(hi) > 0.0:
            return hi
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if surplus(mid) > 0.0:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    # --- Propulsion interface ----------------------------------------------------------

    def step(self, dt: float, lever: float, air: Air, mach: float) -> None:
        tas = mach * air.speed_of_sound
        net = self.engine_torque(lever, air, self.rps) - self.prop_torque(self.rps, air, tas)
        self.rps = min(max(self.rps + net / (self.data.inertia_kgm2 * math.tau) * dt, self.MIN_RPS), self.MAX_RPS)

    def set_steady(self, lever: float, air: Air, mach: float) -> None:
        self.rps = self.steady_rps(lever, air, mach * air.speed_of_sound)

    def thrust(self, air: Air, mach: float) -> float:
        return self.thrust_at(self.rps, air, mach * air.speed_of_sound)

    def steady_thrust(self, lever: float, air: Air, mach: float) -> float:
        tas = mach * air.speed_of_sound
        return self.thrust_at(self.steady_rps(lever, air, tas), air, tas)

    def power_hp(self, lever: float, air: Air) -> float:
        """Brake power at the current RPM."""
        return self.engine_torque(lever, air, self.rps) * self.rps * math.tau / HP
