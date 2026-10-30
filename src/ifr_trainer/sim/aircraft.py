"""Aircraft data loaded from ``aircraft.toml``."""

import math
from dataclasses import dataclass
from pathlib import Path

from ..data_files import load_toml


@dataclass(frozen=True)
class FlapConfig:
    name: str
    cl0_delta: float
    cl_max: float
    cd0_delta: float
    vfe_kt: float | None
    vls_factor: float


@dataclass(frozen=True)
class EngineData:
    count: int
    static_thrust_n: float
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


@dataclass(frozen=True)
class AircraftData:
    name: str
    mass_kg: float
    wing_area_m2: float
    span_m: float
    oswald: float
    cl_alpha_per_rad: float
    cl0: float
    cd0: float
    mach_crit: float
    vmo_kt: float
    mmo: float
    speedbrake_cd: float
    gear_cd: float
    vle_kt: float
    gear_transit_s: float
    flap_transit_s_per_step: float
    flaps: tuple[FlapConfig, ...]
    engines: EngineData

    @property
    def induced_drag_k(self) -> float:
        aspect_ratio = self.span_m**2 / self.wing_area_m2
        return 1.0 / (math.pi * self.oswald * aspect_ratio)

    def flap_config_at(self, position: float) -> FlapConfig:
        """Flap data at a continuous flap position (0 = CONF 0, len-1 = FULL), interpolated."""
        position = min(max(position, 0.0), len(self.flaps) - 1.0)
        i = int(math.floor(position))
        j = min(i + 1, len(self.flaps) - 1)
        f = position - i
        a, b = self.flaps[i], self.flaps[j]

        def lerp(u: float, v: float) -> float:
            return u + (v - u) * f

        vfe = self.flaps[math.ceil(position - 1e-6)].vfe_kt if position > 1e-6 else None
        return FlapConfig(
            name=a.name if f < 0.5 else b.name,
            cl0_delta=lerp(a.cl0_delta, b.cl0_delta),
            cl_max=lerp(a.cl_max, b.cl_max),
            cd0_delta=lerp(a.cd0_delta, b.cd0_delta),
            vfe_kt=vfe,
            vls_factor=lerp(a.vls_factor, b.vls_factor),
        )

    @classmethod
    def load(cls, path: Path | None = None) -> "AircraftData":
        d = load_toml("aircraft.toml", path)
        e = d["engines"]
        engines = EngineData(
            count=e["count"],
            static_thrust_n=e["static_thrust_n"],
            n1_idle=e["n1_idle"],
            n1_max=e["n1_max"],
            idle_thrust_fraction=e["idle_thrust_fraction"],
            thrust_exponent=e["thrust_exponent"],
            density_exponent=e["density_exponent"],
            mach_lapse=e["mach_lapse"],
            accel_rate_low=e["accel_rate_low"],
            accel_rate_high=e["accel_rate_high"],
            decel_rate=e["decel_rate"],
            spool_crossover_n1=e["spool_crossover_n1"],
            spool_tau_s=e["spool_tau_s"],
            lever_n1=tuple((float(x), float(y)) for x, y in e["lever_n1"]),
            detents=dict(e["detents"]),
        )
        flaps = tuple(
            FlapConfig(
                name=f["name"],
                cl0_delta=f["cl0_delta"],
                cl_max=f["cl_max"],
                cd0_delta=f["cd0_delta"],
                vfe_kt=f.get("vfe_kt"),
                vls_factor=f["vls_factor"],
            )
            for f in d["flaps"]["configs"]
        )
        return cls(
            name=d["name"],
            mass_kg=d["mass_kg"],
            wing_area_m2=d["wing_area_m2"],
            span_m=d["span_m"],
            oswald=d["oswald"],
            cl_alpha_per_rad=d["cl_alpha_per_rad"],
            cl0=d["cl0"],
            cd0=d["cd0"],
            mach_crit=d["mach_crit"],
            vmo_kt=d["vmo_kt"],
            mmo=d["mmo"],
            speedbrake_cd=d["speedbrake_cd"],
            gear_cd=d["gear"]["cd"],
            vle_kt=d["gear"]["vle_kt"],
            gear_transit_s=d["gear"]["transit_s"],
            flap_transit_s_per_step=d["flaps"]["transit_s_per_step"],
            flaps=flaps,
            engines=engines,
        )
