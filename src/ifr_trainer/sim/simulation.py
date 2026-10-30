"""The simulation: pilot inputs -> flight control computer -> engines + 6-DOF model -> checks."""

import math
from dataclasses import dataclass
from enum import Enum

from ..fbw.flight_control import FlightControlComputer
from ..fbw.normal_law import NormalLawLimits
from .aero import AeroConfig, Surfaces
from .aircraft import AircraftData
from .airport import Airport
from .atmosphere import cas_from_tas, isa, tas_from_cas
from .engines import Engines
from .flight_model import AircraftState, Environment, SixDofModel
from .gear import point_height
from .scenario import Scenario
from .speeds import SpeedLimits, speed_limits
from .trim import trim_flight
from .units import DEG, FPM, FT, KT, NM
from .vec import mat_t_vec, mat_vec, rotation_matrix


@dataclass
class PilotInputs:
    stick_pitch: float = 0.0  # +1 = full back (nose up)
    stick_roll: float = 0.0  # +1 = full right
    rudder: float = 0.0  # +1 = full right pedal
    brakes: float = 0.0  # 0 ... 1
    lever: float = 0.0  # 0 = IDLE ... 1 = TOGA
    flap_lever: int = 0
    gear_down: bool = False
    speedbrake: float = 0.0  # 0 ... 1


class Phase(Enum):
    AIRBORNE = "airborne"
    ON_GROUND = "on ground"
    CRASHED = "crashed"


class Severity(Enum):
    INFO = "info"
    CAUTION = "caution"
    CRASH = "crash"


def wind_vector(from_deg: float, speed_kt: float) -> tuple[float, float]:
    """(east, north) air mass velocity in m/s for a wind blowing from ``from_deg``."""
    rad = math.radians(from_deg)
    return -speed_kt * KT * math.sin(rad), -speed_kt * KT * math.cos(rad)


class Simulation:
    TREND_TAU_S = 1.0
    SETTLE_S = 3.0  # ground starts settle on the gear before the clock starts
    LIFTOFF_S = 0.5  # time with no weight on wheels before the aircraft counts as airborne
    HARD_LANDING_FPM = -600.0
    CRASH_LANDING_FPM = -900.0
    STRUT_STATIC_COMPRESSION_M = 0.25

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
        self.model = SixDofModel(aircraft)
        self.engines = Engines(aircraft.engines)
        self.fcc = FlightControlComputer(aircraft, self.model.aero, law_limits or NormalLawLimits())
        self.reset()

    # --- Setup -------------------------------------------------------------------------

    def reset(self) -> PilotInputs:
        """Put the aircraft at the scenario start, trimmed. Returns the matching pilot inputs."""
        sc = self.scenario
        rwy = self.airport.runway(sc.runway)
        hdg = math.radians(rwy.heading_deg)
        d = sc.distance_nm * NM
        self.mass = sc.mass_kg or self.aircraft.mass_kg
        self.flap_pos = float(sc.flaps)
        self.gear_pos = 1.0 if (sc.gear_down or sc.on_ground) else 0.0
        self.surfaces = Surfaces()
        wind_e, wind_n = wind_vector(sc.wind_dir_deg, sc.wind_kt)
        self.wind = (wind_e, wind_n)
        self.env = Environment(wind_ned=(wind_n, wind_e, 0.0), ground_d=-self.airport.elevation_m)
        psi = math.radians(sc.heading_deg if sc.heading_deg is not None else rwy.heading_deg)
        self.state = AircraftState(x_m=rwy.threshold_x_m - d * math.sin(hdg), y_m=rwy.threshold_y_m - d * math.cos(hdg))
        self.phase = Phase.ON_GROUND if sc.on_ground else Phase.AIRBORNE
        self.message = ""
        self.severity = Severity.INFO
        self.message_time = 0.0
        self._thrust = 0.0
        self._air_time = 0.0
        self._landed = False

        if sc.on_ground:
            lever = 0.0
            self.inputs = PilotInputs(lever=lever, flap_lever=sc.flaps, gear_down=True, brakes=1.0)
            self._place_on_ground(psi, sc.ths_deg * DEG)
        else:
            gamma = sc.flight_path_deg * DEG
            h = sc.altitude_ft * FT
            tas = tas_from_cas(sc.cas_kt * KT, isa(h))
            trim = trim_flight(self.aircraft, self.model.aero, self.engines, self.aero(), self.mass, h, tas, gamma)
            s = self.state
            s.h_m = h
            s.set_attitude(0.0, trim.alpha + gamma, psi)
            wind_body = mat_t_vec(rotation_matrix(s.quat), self.env.wind_ned)
            s.u = tas * math.cos(trim.alpha) + wind_body[0]
            s.v = wind_body[1]
            s.w = tas * math.sin(trim.alpha) + wind_body[2]
            s.nz = math.cos(gamma)
            self.model.air_data(s, self.env)
            self.surfaces.ths = trim.ths
            self.fcc.reset(airborne=True)
            lever = trim.lever
            self.inputs = PilotInputs(lever=lever, flap_lever=sc.flaps, gear_down=sc.gear_down)
        self.engines.set_steady(lever)
        self._thrust = self.engines.thrust(isa(self.state.h_m), self.state.mach)
        self.time_s = 0.0
        self.speed_trend_kt = 0.0
        self._last_cas_kt = self.cas_kt
        return self.inputs

    def _place_on_ground(self, psi: float, ths: float) -> None:
        s = self.state
        main_z = max(leg.z for leg in self.aircraft.gear.legs)
        s.h_m = self.airport.elevation_m + main_z - self.STRUT_STATIC_COMPRESSION_M
        s.set_attitude(0.0, 0.0, psi)
        self.model.air_data(s, self.env)
        self.surfaces.ths = ths
        self.fcc.reset(airborne=False)
        dt = 1.0 / 120.0
        for _ in range(round(self.SETTLE_S / dt)):
            self._advance(dt, self.inputs)

    def aero(self) -> AeroConfig:
        ac = self.aircraft
        cfg = ac.flap_config_at(self.flap_pos)
        cd0 = ac.cd0 + cfg.cd0_delta + ac.gear.cd * self.gear_pos + ac.speedbrake_cd * self.surfaces.speedbrake
        return AeroConfig(cl0=ac.cl0 + cfg.cl0_delta, cl_max=cfg.cl_max, cd0=cd0, cm0=ac.aero.cm0 + cfg.cm0_delta)

    # --- Stepping ----------------------------------------------------------------------

    def step(self, dt: float, inputs: PilotInputs) -> None:
        if self.phase is Phase.CRASHED:
            return
        self.inputs = inputs
        self._advance(dt, inputs)
        self.time_s += dt
        cas = self.cas_kt
        rate = (cas - self._last_cas_kt) / dt
        self._last_cas_kt = cas
        self.speed_trend_kt += (rate * 10.0 - self.speed_trend_kt) * min(dt / self.TREND_TAU_S, 1.0)
        self._update_phase(dt)

    def _advance(self, dt: float, inputs: PilotInputs) -> None:
        s = self.state
        self._move_surfaces(dt, inputs)
        cfg = self.aero()
        cas = self.cas_kt
        self.fcc.update(dt, inputs, s, self.surfaces, cfg, self.mass, self._thrust, cas, self.flap_pos > 0.01)
        self.engines.step(dt, inputs.lever)
        self._thrust = self.engines.thrust(isa(s.h_m), s.mach)
        self.model.step(
            s, cfg, self.surfaces, self.mass, self._thrust, inputs.brakes, self.gear_pos >= 1.0, self.env, dt
        )

    def _move_surfaces(self, dt: float, inputs: PilotInputs) -> None:
        ac = self.aircraft
        target = float(min(max(inputs.flap_lever, 0), len(ac.flaps) - 1))
        step = dt / ac.flap_transit_s_per_step
        self.flap_pos += max(-step, min(step, target - self.flap_pos))
        on_ground = self.state.on_ground
        gear_target = 1.0 if (inputs.gear_down or on_ground) else 0.0  # no retraction on the ground
        step = dt / ac.gear.transit_s
        self.gear_pos += max(-step, min(step, gear_target - self.gear_pos))
        sf = self.surfaces
        sf.speedbrake += max(-dt, min(dt, inputs.speedbrake - sf.speedbrake))
        # Ground spoilers: extend on the ground with the lever at idle above taxi speed.
        idle = inputs.lever <= ac.engines.detents["IDLE"] + 0.02
        deploy = on_ground and idle and self.groundspeed_kt > 20.0
        sf.ground_spoiler += max(-dt, min(dt, (1.0 if deploy else 0.0) - sf.ground_spoiler))

    # --- Phase, touchdown and damage ---------------------------------------------------

    def _set_message(self, text: str, severity: Severity) -> None:
        self.message = text
        self.severity = severity
        self.message_time = self.time_s
        if severity is Severity.CRASH:
            self.phase = Phase.CRASHED

    def _update_phase(self, dt: float) -> None:
        s = self.state
        rot = rotation_matrix(s.quat)
        for name, point in self.aircraft.gear.strike_points.items():
            if point_height(point, rot, s.pos_ned, self.env.ground_d) < 0.0:
                self._set_message(f"CRASH: {name}", Severity.CRASH)
                return

        if not s.on_ground:
            self._air_time += dt
            if self.phase is Phase.ON_GROUND and self._air_time > self.LIFTOFF_S:
                self.phase = Phase.AIRBORNE
            return

        if self.phase is Phase.AIRBORNE:
            self._touchdown(rot)
            if self.phase is Phase.CRASHED:
                return
        self.phase = Phase.ON_GROUND
        self._air_time = 0.0
        if self.groundspeed_kt > 1.0 and not self._mains_on_runway(rot):
            self._set_message("CRASH: runway excursion", Severity.CRASH)
        elif self._landed and self.groundspeed_kt < 1.0:
            rwy = self.airport.runway(self._landing_runway)
            along, _ = rwy.to_runway_frame(s.x_m, s.y_m)
            self._landed = False
            self._set_message(f"{self.message}. Stopped {rwy.length_m - along:.0f} m before the runway end", self.severity)

    def _mains_on_runway(self, rot) -> bool:
        s = self.state
        for leg in self.aircraft.gear.legs:
            if leg.name == "nose":
                continue
            n, e, _ = mat_vec(rot, leg.position)
            if self.airport.runway_at(s.x_m + e, s.y_m + n) is None:
                return False
        return True

    def _touchdown(self, rot) -> None:
        s = self.state
        vs = self.vs_fpm
        rwy = self.airport.runway_at(s.x_m, s.y_m)
        if rwy is None or not self._mains_on_runway(rot):
            self._set_message(f"CRASH: touched down off the runway at {vs:.0f} fpm", Severity.CRASH)
            return
        along, cross = rwy.to_runway_frame(s.x_m, s.y_m)
        details = f"{vs:.0f} fpm, {along:.0f} m past threshold {rwy.ident}, {cross:+.1f} m from centreline"
        if vs < self.CRASH_LANDING_FPM:
            self._set_message(f"CRASH: {details}", Severity.CRASH)
            return
        hard = vs < self.HARD_LANDING_FPM
        self._landed = True
        self._landing_runway = rwy.ident
        self._set_message(("HARD LANDING: " if hard else "TOUCHDOWN: ") + details, Severity.CAUTION if hard else Severity.INFO)

    # --- Derived values for displays -------------------------------------------------

    @property
    def cas_kt(self) -> float:
        return cas_from_tas(self.state.tas, isa(self.state.h_m)) / KT

    @property
    def alt_ft(self) -> float:
        return self.state.h_m / FT

    @property
    def agl_ft(self) -> float:
        """Radio altitude: height of the main wheels above the ground."""
        s = self.state
        rot = rotation_matrix(s.quat)
        mains = [leg for leg in self.aircraft.gear.legs if leg.name != "nose"]
        return min(point_height(leg.position, rot, s.pos_ned, self.env.ground_d) for leg in mains) / FT

    @property
    def vs_fpm(self) -> float:
        return -self.state.v_ned[2] / FPM

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.state.psi) % 360.0

    @property
    def ground_velocity(self) -> tuple[float, float]:
        """(east, north) in m/s."""
        vn, ve, _ = self.state.v_ned
        return ve, vn

    @property
    def track_deg(self) -> float:
        ve, vn = self.ground_velocity
        if math.hypot(ve, vn) < 1.0:
            return self.heading_deg
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
