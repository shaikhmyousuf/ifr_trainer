"""Aircraft data loaded from ``aircraft.toml``."""

import math
from dataclasses import dataclass
from pathlib import Path

from ..data_files import load_toml

Vec = tuple[float, float, float]


@dataclass(frozen=True)
class FlapConfig:
    name: str
    cl0_delta: float
    cl_max: float
    cd0_delta: float
    cm0_delta: float
    vls_factor: float
    vfe_kt: float | None = None


@dataclass(frozen=True)
class EngineData:
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


@dataclass(frozen=True)
class Inertia:
    ixx: float
    iyy: float
    izz: float


@dataclass(frozen=True)
class AeroDerivatives:
    """Non-dimensional derivatives (per rad; rates non-dimensionalised by span or MAC over 2V)."""

    cl_q: float
    cl_elevator: float
    cl_ths: float
    cl_min: float
    cl_post_stall_slope: float
    cm0: float
    cm_alpha: float
    cm_q: float
    cm_alphadot: float
    cm_elevator: float
    cm_ths: float
    cy_beta: float
    cy_rudder: float
    roll_beta: float
    roll_p: float
    roll_r: float
    roll_aileron: float
    roll_rudder: float
    yaw_beta: float
    yaw_p: float
    yaw_r: float
    yaw_aileron: float
    yaw_rudder: float
    cd_beta: float
    speedbrake_cl: float
    ground_spoiler_cl: float
    ground_spoiler_cd: float


@dataclass(frozen=True)
class ControlLimits:
    elevator_up_deg: float
    elevator_down_deg: float
    aileron_deg: float
    rudder_deg: float
    rudder_limit_start_kt: float
    rudder_limit_end_kt: float
    rudder_limit_high_speed_deg: float
    ths_up_deg: float
    ths_down_deg: float
    ths_rate_deg_s: float
    elevator_rate_deg_s: float
    aileron_rate_deg_s: float
    rudder_rate_deg_s: float
    nosewheel_pedal_deg: float
    nosewheel_fade_kt: float

    def rudder_limit_deg(self, cas_kt: float) -> float:
        """Rudder travel limiter: full travel at low speed, reduced at high speed."""
        f = (cas_kt - self.rudder_limit_start_kt) / (self.rudder_limit_end_kt - self.rudder_limit_start_kt)
        f = min(max(f, 0.0), 1.0)
        return self.rudder_deg + (self.rudder_limit_high_speed_deg - self.rudder_deg) * f


@dataclass(frozen=True)
class GearLeg:
    name: str
    x: float
    y: float
    z: float
    stiffness: float
    damping: float
    steerable: bool
    braked: bool

    @property
    def position(self) -> Vec:
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class GearData:
    cd: float
    vle_kt: float
    transit_s: float
    rolling_friction: float
    brake_friction: float
    lateral_friction: float
    legs: tuple[GearLeg, ...]
    strike_points: dict[str, Vec]


@dataclass(frozen=True)
class AircraftData:
    name: str
    mass_kg: float
    wing_area_m2: float
    span_m: float
    mac_m: float
    oswald: float
    cl_alpha_per_rad: float
    cl0: float
    cd0: float
    mach_crit: float
    vmo_kt: float
    mmo: float
    speedbrake_cd: float
    inertia: Inertia
    aero: AeroDerivatives
    controls: ControlLimits
    gear: GearData
    flap_transit_s_per_step: float
    flaps: tuple[FlapConfig, ...]
    engines: EngineData

    @property
    def induced_drag_k(self) -> float:
        aspect_ratio = self.span_m**2 / self.wing_area_m2
        return 1.0 / (math.pi * self.oswald * aspect_ratio)

    def inertia_for(self, mass: float) -> Inertia:
        f = mass / self.mass_kg
        return Inertia(self.inertia.ixx * f, self.inertia.iyy * f, self.inertia.izz * f)

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
            cm0_delta=lerp(a.cm0_delta, b.cm0_delta),
            vls_factor=lerp(a.vls_factor, b.vls_factor),
            vfe_kt=vfe,
        )

    @classmethod
    def load(cls, path: Path | None = None) -> "AircraftData":
        d = load_toml("aircraft.toml", path)
        e = dict(d["engines"])
        e["lever_n1"] = tuple((float(x), float(y)) for x, y in e["lever_n1"])
        e["detents"] = dict(e["detents"])
        g = dict(d["gear"])
        g["legs"] = tuple(GearLeg(**leg) for leg in g["legs"])
        g["strike_points"] = {name: tuple(p) for name, p in g["strike_points"].items()}
        scalars = {k: v for k, v in d.items() if not isinstance(v, dict)}
        return cls(
            **scalars,
            inertia=Inertia(**d["inertia"]),
            aero=AeroDerivatives(**d["aero"]),
            controls=ControlLimits(**d["controls"]),
            gear=GearData(**g),
            flap_transit_s_per_step=d["flaps"]["transit_s_per_step"],
            flaps=tuple(FlapConfig(**f) for f in d["flaps"]["configs"]),
            engines=EngineData(**e),
        )
