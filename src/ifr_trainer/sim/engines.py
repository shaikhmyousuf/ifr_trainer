"""Engine model: lever -> N1 command, spool dynamics, N1 -> thrust with altitude/Mach lapse."""

from .aircraft import EngineData
from .atmosphere import RHO0, Air


class Engines:
    """All engines lumped together (symmetric thrust)."""

    def __init__(self, data: EngineData):
        self.data = data
        self.n1 = data.n1_idle

    def lever_to_n1(self, lever: float) -> float:
        table = self.data.lever_n1
        lever = min(max(lever, table[0][0]), table[-1][0])
        for (x0, y0), (x1, y1) in zip(table, table[1:]):
            if lever <= x1:
                return y0 + (y1 - y0) * (lever - x0) / (x1 - x0)
        return table[-1][1]

    def step(self, dt: float, lever: float) -> None:
        d = self.data
        target = self.lever_to_n1(lever)
        rate = (target - self.n1) / d.spool_tau_s
        if rate > 0:
            rate = min(rate, d.accel_rate_low if self.n1 < d.spool_crossover_n1 else d.accel_rate_high)
        else:
            rate = max(rate, -d.decel_rate)
        self.n1 += rate * dt

    def set_steady(self, lever: float) -> None:
        self.n1 = self.lever_to_n1(lever)

    def thrust_at(self, n1: float, air: Air, mach: float) -> float:
        d = self.data
        frac = min(max((n1 - d.n1_idle) / (d.n1_max - d.n1_idle), 0.0), 1.0)
        f = d.idle_thrust_fraction + (1.0 - d.idle_thrust_fraction) * frac**d.thrust_exponent
        lapse = (air.density / RHO0) ** d.density_exponent * (1.0 - d.mach_lapse * mach)
        return d.count * d.static_thrust_n * f * lapse

    def thrust(self, air: Air, mach: float) -> float:
        return self.thrust_at(self.n1, air, mach)
