"""The simulation: pilot inputs -> normal law -> engines + flight model -> systems."""

import math
from dataclasses import dataclass
from enum import Enum

from ..fbw.normal_law import NormalLawLimits, pitch_law, roll_law
from .aircraft import AircraftData
from .airport import Airport
from .atmosphere import cas_from_tas, isa, tas_from_cas
from .engines import Engines
from .flight_model import AeroConfig, AircraftState, PointMassModel
from .scenario import Scenario
from .speeds import SpeedLimits, speed_limits
from .trim import trim_lever
from .units import DEG, FPM, FT, KT, NM


@dataclass
class PilotInputs:
    stick_pitch: float = 0.0  # +1 = full back (nose up)
    stick_roll: float = 0.0  # +1 = full right
    lever: float = 0.0  # 0 = IDLE ... 1 = TOGA
    flap_lever: int = 0
    gear_down: bool = False
    speedbrake: float = 0.0  # 0 ... 1


class Phase(Enum):
    FLYING = "flying"
    LANDED = "landed"
    CRASHED = "crashed"


def wind_vector(from_deg: float, speed_kt: float) -> tuple[float, float]:
    """(east, north) air mass velocity in m/s for a wind blowing from ``from_deg``."""
    rad = math.radians(from_deg)
    return -speed_kt * KT * math.sin(rad), -speed_kt * KT * math.cos(rad)


class Simulation:
    TREND_TAU_S = 1.0
    MAX_TOUCHDOWN_FPM = -600.0
    MAX_TOUCHDOWN_BANK_DEG = 7.0
    MAX_TOUCHDOWN_PITCH_DEG = 11.5

    def __init__(
        self,
        aircraft: AircraftData,
        airport: Airport,
        scenario: Scenario,
        law_limits: NormalLawLimits | None = None,
    ):
        self.aircraft = aircraft
        self.airport = airport
        self.scenario = scenario
        self.law_limits = law_limits or NormalLawLimits()
        self.model = PointMassModel(aircraft)
        self.engines = Engines(aircraft.engines)
        self.reset()

    def reset(self) -> PilotInputs:
        """Put the aircraft at the scenario start, trimmed. Returns the matching pilot inputs."""
        sc = self.scenario
        rwy = self.airport.runway(sc.runway)
        hdg = math.radians(rwy.heading_deg)
        d = sc.distance_nm * NM
        h = sc.altitude_ft * FT
        self.mass = sc.mass_kg or self.aircraft.mass_kg
        self.flap_pos = float(sc.flaps)
        self.gear_pos = 1.0 if sc.gear_down else 0.0
        self.speedbrake = 0.0
        self.wind = wind_vector(sc.wind_dir_deg, sc.wind_kt)
        self.state = AircraftState(
            x_m=rwy.threshold_x_m - d * math.sin(hdg),
            y_m=rwy.threshold_y_m - d * math.cos(hdg),
            h_m=h,
            tas=tas_from_cas(sc.cas_kt * KT, isa(h)),
            psi=math.radians(sc.heading_deg if sc.heading_deg is not None else rwy.heading_deg),
        )
        self.model.update_derived(self.state, self.aero(), self.mass)
        lever = trim_lever(self.model, self.engines, self.state, self.aero(), self.mass)
        self.engines.set_steady(lever)
        self.phase = Phase.FLYING
        self.message = ""
        self.time_s = 0.0
        self.speed_trend_kt = 0.0
        self._last_cas_kt = self.cas_kt
        self.inputs = PilotInputs(lever=lever, flap_lever=sc.flaps, gear_down=sc.gear_down)
        return self.inputs

    def aero(self) -> AeroConfig:
        ac = self.aircraft
        cfg = ac.flap_config_at(self.flap_pos)
        cd0 = ac.cd0 + cfg.cd0_delta + ac.gear_cd * self.gear_pos + ac.speedbrake_cd * self.speedbrake
        return AeroConfig(cl0=ac.cl0 + cfg.cl0_delta, cl_max=cfg.cl_max, cd0=cd0)

    def step(self, dt: float, inputs: PilotInputs) -> None:
        if self.phase is not Phase.FLYING:
            return
        self.inputs = inputs
        self._move_surfaces(dt, inputs)
        s = self.state
        nz_cmd = pitch_law(inputs.stick_pitch, s.gamma, s.phi, s.theta, self.flap_pos > 0.01, self.law_limits)
        p_cmd = roll_law(inputs.stick_roll, s.phi, self.law_limits)
        self.engines.step(dt, inputs.lever)
        air = isa(s.h_m)
        thrust = self.engines.thrust(air, s.tas / air.speed_of_sound)
        self.model.step(s, self.aero(), self.mass, thrust, nz_cmd, p_cmd, self.wind, dt)
        self.time_s += dt

        cas = self.cas_kt
        rate = (cas - self._last_cas_kt) / dt
        self._last_cas_kt = cas
        self.speed_trend_kt += (rate * 10.0 - self.speed_trend_kt) * min(dt / self.TREND_TAU_S, 1.0)
        self._check_ground()

    def _move_surfaces(self, dt: float, inputs: PilotInputs) -> None:
        ac = self.aircraft
        target = float(min(max(inputs.flap_lever, 0), len(ac.flaps) - 1))
        step = dt / ac.flap_transit_s_per_step
        self.flap_pos += max(-step, min(step, target - self.flap_pos))
        gear_target = 1.0 if inputs.gear_down else 0.0
        step = dt / ac.gear_transit_s
        self.gear_pos += max(-step, min(step, gear_target - self.gear_pos))
        self.speedbrake += max(-dt, min(dt, inputs.speedbrake - self.speedbrake))

    def _check_ground(self) -> None:
        s = self.state
        if s.h_m > self.airport.elevation_m:
            return
        vs = self.vs_fpm
        s.h_m = self.airport.elevation_m
        rwy = self.airport.runway_at(s.x_m, s.y_m)
        problems = []
        if rwy is None:
            problems.append("off the runway")
        if self.gear_pos < 1.0:
            problems.append("gear not down")
        if vs < self.MAX_TOUCHDOWN_FPM:
            problems.append(f"sink rate {vs:.0f} fpm")
        if abs(s.phi) > self.MAX_TOUCHDOWN_BANK_DEG * DEG:
            problems.append(f"bank {math.degrees(s.phi):.0f} deg")
        if s.theta > self.MAX_TOUCHDOWN_PITCH_DEG * DEG:
            problems.append("tail strike")
        elif s.theta < 0.0:
            problems.append("nose gear first")
        if problems:
            self.phase = Phase.CRASHED
            self.message = "CRASH: " + ", ".join(problems)
        else:
            along, cross = rwy.to_runway_frame(s.x_m, s.y_m)
            self.phase = Phase.LANDED
            self.message = (
                f"LANDED RWY {rwy.ident}: {vs:.0f} fpm, {along:.0f} m past threshold, "
                f"{cross:+.1f} m from centreline"
            )

    # --- Derived values for displays -------------------------------------------------

    @property
    def cas_kt(self) -> float:
        return cas_from_tas(self.state.tas, isa(self.state.h_m)) / KT

    @property
    def alt_ft(self) -> float:
        return self.state.h_m / FT

    @property
    def agl_ft(self) -> float:
        return (self.state.h_m - self.airport.elevation_m) / FT

    @property
    def vs_fpm(self) -> float:
        return self.state.tas * math.sin(self.state.gamma) / FPM

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.state.psi) % 360.0

    @property
    def ground_velocity(self) -> tuple[float, float]:
        s = self.state
        horizontal = s.tas * math.cos(s.gamma)
        return horizontal * math.sin(s.psi) + self.wind[0], horizontal * math.cos(s.psi) + self.wind[1]

    @property
    def track_deg(self) -> float:
        ve, vn = self.ground_velocity
        return math.degrees(math.atan2(ve, vn)) % 360.0

    @property
    def groundspeed_kt(self) -> float:
        return math.hypot(*self.ground_velocity) / KT

    @property
    def threshold_distance_nm(self) -> float:
        rwy = self.airport.runway(self.scenario.runway)
        return math.hypot(self.state.x_m - rwy.threshold_x_m, self.state.y_m - rwy.threshold_y_m) / NM

    def speed_limits(self) -> SpeedLimits:
        return speed_limits(self.aircraft, self.mass, self.flap_pos, self.gear_pos, self.state.h_m)
