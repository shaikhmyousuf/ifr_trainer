"""Turbofan engines: lever -> N1 command, spool dynamics, N1 -> thrust with altitude/Mach lapse.

Shared by jet aircraft modules; each supplies its own ``[engines]`` data.
"""

from dataclasses import dataclass

from .atmosphere import RHO0, Air


@dataclass(frozen=True)
class TurbofanData:
    count: int
    static_thrust_n: float
    z_m: float
    n1_idle: float
    n1_max: float
    idle_thrust_fraction: float
    thrust_exponent: float
    density_exponent: float
    mach_lapse: float
    accel_rate_low: float
    accel_rate_high: float
    decel_rate: float
    spool_crossover_n1: float
    spool_tau_s: float
    lever_n1: tuple[tuple[float, float], ...]
    detents: dict[str, float]
    mach_lapse_sea_level: float = 0.0  # extra Mach lapse, fading with density squared: ram drag dominates low down

    @classmethod
    def from_dict(cls, d: dict) -> "TurbofanData":
        d = dict(d)
        d["lever_n1"] = tuple((float(x), float(y)) for x, y in d["lever_n1"])
        d["detents"] = dict(d.get("detents", {}))
        return cls(**d)


class Turbofan:
    """All engines lumped together (symmetric thrust)."""

    def __init__(self, data: TurbofanData):
        self.data = data
        self.n1 = data.n1_idle

    @property
    def thrust_line_z_m(self) -> float:
        return self.data.z_m

    @property
    def detents(self) -> dict[str, float]:
        return self.data.detents

    def lever_to_n1(self, lever: float) -> float:
        table = self.data.lever_n1
        lever = min(max(lever, table[0][0]), table[-1][0])
        for (x0, y0), (x1, y1) in zip(table, table[1:]):
            if lever <= x1:
                return y0 + (y1 - y0) * (lever - x0) / (x1 - x0)
        return table[-1][1]

    def step(self, dt: float, lever: float, air: Air | None = None, mach: float = 0.0) -> None:
        d = self.data
        target = self.lever_to_n1(lever)
        rate = (target - self.n1) / d.spool_tau_s
        if rate > 0:
            rate = min(rate, d.accel_rate_low if self.n1 < d.spool_crossover_n1 else d.accel_rate_high)
        else:
            rate = max(rate, -d.decel_rate)
        self.n1 += rate * dt

    def set_steady(self, lever: float, air: Air | None = None, mach: float = 0.0) -> None:
        self.n1 = self.lever_to_n1(lever)

    def thrust_at(self, n1: float, air: Air, mach: float) -> float:
        d = self.data
        frac = min(max((n1 - d.n1_idle) / (d.n1_max - d.n1_idle), 0.0), 1.0)
        f = d.idle_thrust_fraction + (1.0 - d.idle_thrust_fraction) * frac**d.thrust_exponent
        sigma = air.density / RHO0
        lapse = sigma**d.density_exponent * (1.0 - (d.mach_lapse + d.mach_lapse_sea_level * sigma * sigma) * mach)
        return d.count * d.static_thrust_n * f * lapse

    def thrust(self, air: Air, mach: float) -> float:
        return self.thrust_at(self.n1, air, mach)

    def steady_thrust(self, lever: float, air: Air, mach: float) -> float:
        """Thrust once the engines have settled at this lever position."""
        return self.thrust_at(self.lever_to_n1(lever), air, mach)
