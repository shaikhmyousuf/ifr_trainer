"""Airframe data shared by every aircraft type: geometry, mass, inertia, aerodynamics,
flaps, gear and control surface limits. Loaded from the aircraft module's ``aircraft.toml``;
sections the airframe does not know (engines, type-specific data) are left to the module.
"""

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
    cm0_delta: float
    vfe_kt: float | None = None


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
    cl_pitch_trim: float
    cl_min: float
    cl_post_stall_slope: float
    cm0: float
    cm_alpha: float
    cm_q: float
    cm_alphadot: float
    cm_elevator: float
    cm_pitch_trim: float
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
    speedbrake_cl: float = 0.0


@dataclass(frozen=True)
class ControlLimits:
    """Surface travel and rates. Pitch trim is the THS on an airliner, a trim tab on a light aircraft."""

    elevator_up_deg: float
    elevator_down_deg: float
    aileron_deg: float
    rudder_deg: float
    pitch_trim_up_deg: float
    pitch_trim_down_deg: float
    pitch_trim_rate_deg_s: float
    elevator_rate_deg_s: float
    aileron_rate_deg_s: float
    rudder_rate_deg_s: float
    # Optional rudder travel limiter: full travel below the start speed, reducing to
    # rudder_limit_high_speed_deg at the end speed.
    rudder_limit_start_kt: float | None = None
    rudder_limit_end_kt: float | None = None
    rudder_limit_high_speed_deg: float | None = None

    def rudder_limit_deg(self, cas_kt: float) -> float:
        if self.rudder_limit_start_kt is None or self.rudder_limit_end_kt is None:
            return self.rudder_deg
        f = (cas_kt - self.rudder_limit_start_kt) / (self.rudder_limit_end_kt - self.rudder_limit_start_kt)
        f = min(max(f, 0.0), 1.0)
        return self.rudder_deg + (self.rudder_limit_high_speed_deg - self.rudder_deg) * f


@dataclass(frozen=True)
class GearData:
    cd: float
    vle_kt: float
    transit_s: float


@dataclass(frozen=True)
class Airframe:
    name: str
    mass_kg: float
    wing_area_m2: float
    span_m: float
    mac_m: float
    oswald: float
    cl_alpha_per_rad: float
    cl0: float
    cd0: float
    vmo_kt: float
    inertia: Inertia
    aero: AeroDerivatives
    controls: ControlLimits
    gear: GearData
    flap_transit_s_per_step: float
    flaps: tuple[FlapConfig, ...]
    mmo: float | None = None
    mach_crit: float = 1.0  # wave drag rises above this Mach number
    speedbrake_cd: float = 0.0

    @property
    def induced_drag_k(self) -> float:
        aspect_ratio = self.span_m**2 / self.wing_area_m2
        return 1.0 / (math.pi * self.oswald * aspect_ratio)

    def inertia_for(self, mass: float) -> Inertia:
        f = mass / self.mass_kg
        return Inertia(self.inertia.ixx * f, self.inertia.iyy * f, self.inertia.izz * f)

    def flap_config_at(self, position: float) -> FlapConfig:
        """Flap data at a continuous flap position (0 = retracted, len-1 = fully extended), interpolated."""
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
            vfe_kt=vfe,
        )

    @classmethod
    def from_dict(cls, d: dict) -> "Airframe":
        known = {"inertia", "aero", "controls", "gear", "flaps"}
        scalars = {k: v for k, v in d.items() if not isinstance(v, dict) and not isinstance(v, list)}
        missing = known - d.keys()
        if missing:
            raise KeyError(f"aircraft data is missing sections: {sorted(missing)}")
        return cls(
            **scalars,
            inertia=Inertia(**d["inertia"]),
            aero=AeroDerivatives(**d["aero"]),
            controls=ControlLimits(**d["controls"]),
            gear=GearData(**d["gear"]),
            flap_transit_s_per_step=d["flaps"]["transit_s_per_step"],
            flaps=tuple(FlapConfig(**f) for f in d["flaps"]["configs"]),
        )

    @classmethod
    def load(cls, package: str, path: Path | None = None) -> "Airframe":
        return cls.from_dict(load_toml("aircraft.toml", path, package))
