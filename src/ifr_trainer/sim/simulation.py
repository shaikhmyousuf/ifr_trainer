"""The simulation: pilot inputs -> flight control computer -> engines + 6-DOF model -> monitoring."""

import math
from dataclasses import dataclass
from enum import Enum

from ..fbw.flight_control import FlightControlComputer
from ..fbw.normal_law import NormalLawLimits
from ..nav.receivers import NavReadings, NavReceivers
from .aero import AeroConfig, Surfaces
from .aircraft import AircraftData
from .airport import Airport
from .atmosphere import cas_from_tas, isa, tas_from_cas
from .cockpit import Cockpit
from .engines import Engines
from .flight_model import AircraftState, SixDofModel
from .scenario import Scenario
from .speeds import SpeedLimits, speed_limits
from .trim import trim_flight
from .units import DEG, FPM, FT, KT, NM
from .vec import mat_t_vec, rotation_matrix


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
    WHEEL_HEIGHT_M = 3.0  # main wheels below the CG; the radio altimeter reads wheel height
    GO_AROUND_VS_FPM = 300.0  # climbing this fast after reaching the minimum counts as a go-around
    MAX_DIP_BELOW_MINIMUM_FT = 50.0  # a normal go-around sinks a little below the minimum

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
        self.nav = NavReceivers(airport.navaids)
        self.reset()

    def reset(self) -> PilotInputs:
        """Put the aircraft at the scenario start, trimmed. Returns the matching pilot inputs."""
        sc = self.scenario
        rwy = self.airport.runway(sc.runway)
        hdg = math.radians(rwy.heading_deg)
        d, off = sc.distance_nm * NM, sc.offset_nm * NM
        self.cockpit = Cockpit(**sc.cockpit)
        self.mass = sc.mass_kg or self.aircraft.mass_kg
        self.flap_pos = float(sc.flaps)
        self.gear_pos = 1.0 if sc.gear_down else 0.0
        self.surfaces = Surfaces()
        wind_e, wind_n = wind_vector(sc.wind_dir_deg, sc.wind_kt)
        self.wind = (wind_e, wind_n)
        self.wind_ned = (wind_n, wind_e, 0.0)

        gamma = sc.flight_path_deg * DEG
        h = sc.altitude_ft * FT
        tas = tas_from_cas(sc.cas_kt * KT, isa(h))
        trim = trim_flight(self.aircraft, self.model.aero, self.engines, self.aero(), self.mass, h, tas, gamma)
        psi = math.radians(sc.heading_deg if sc.heading_deg is not None else rwy.heading_deg)
        s = self.state = AircraftState(
            x_m=rwy.threshold_x_m - d * math.sin(hdg) + off * math.cos(hdg),
            y_m=rwy.threshold_y_m - d * math.cos(hdg) - off * math.sin(hdg),
            h_m=h,
        )
        s.set_attitude(0.0, trim.alpha + gamma, psi)
        wind_body = mat_t_vec(rotation_matrix(s.quat), self.wind_ned)
        s.u = tas * math.cos(trim.alpha) + wind_body[0]
        s.v = wind_body[1]
        s.w = tas * math.sin(trim.alpha) + wind_body[2]
        s.nz = math.cos(gamma)
        self.model.air_data(s, self.wind_ned)
        self.surfaces.ths = trim.ths
        self.engines.set_steady(trim.lever)
        self._thrust = trim.thrust
        self.inputs = PilotInputs(lever=trim.lever, flap_lever=sc.flaps, gear_down=sc.gear_down)

        self.phase = Phase.FLYING
        self.message = ""
        self.severity = Severity.INFO
        self.message_time = 0.0
        self.time_s = 0.0
        self.speed_trend_kt = 0.0
        self._last_cas_kt = self.cas_kt
        self.minimum_reached_time: float | None = None
        self._lowest_after_minimum_ft = math.inf
        self._go_around_reported = False
        return self.inputs

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
        s = self.state
        self._move_surfaces(dt, inputs)
        cfg = self.aero()
        cas = self.cas_kt
        self.fcc.update(dt, inputs, s, self.surfaces, cfg, self.mass, self._thrust, cas, self.flap_pos > 0.01)
        self.engines.step(dt, inputs.lever)
        self._thrust = self.engines.thrust(isa(s.h_m), s.mach)
        self.model.step(s, cfg, self.surfaces, self.mass, self._thrust, self.wind_ned, dt)
        self.time_s += dt

        cas = self.cas_kt
        rate = (cas - self._last_cas_kt) / dt
        self._last_cas_kt = cas
        self.speed_trend_kt += (rate * 10.0 - self.speed_trend_kt) * min(dt / self.TREND_TAU_S, 1.0)
        self._monitor()

    def _move_surfaces(self, dt: float, inputs: PilotInputs) -> None:
        ac = self.aircraft
        target = float(min(max(inputs.flap_lever, 0), len(ac.flaps) - 1))
        step = dt / ac.flap_transit_s_per_step
        self.flap_pos += max(-step, min(step, target - self.flap_pos))
        step = dt / ac.gear.transit_s
        self.gear_pos += max(-step, min(step, (1.0 if inputs.gear_down else 0.0) - self.gear_pos))
        sf = self.surfaces
        sf.speedbrake += max(-dt, min(dt, inputs.speedbrake - sf.speedbrake))

    # --- Minimum, go-around and terrain ------------------------------------------------

    def _set_message(self, text: str, severity: Severity) -> None:
        self.message = text
        self.severity = severity
        self.message_time = self.time_s
        if severity is Severity.CRASH:
            self.phase = Phase.CRASHED

    def _monitor(self) -> None:
        alt, vs = self.alt_ft, self.vs_fpm
        minimum = self.cockpit.minimum_ft
        if self.agl_ft <= 0.0:
            below = minimum is not None and self.minimum_reached_time is not None
            reason = " after descending below the minimum" if below else ""
            self._set_message(f"CRASH: flew into terrain{reason}", Severity.CRASH)
            return
        if minimum is None:
            return
        if self.minimum_reached_time is None:
            if alt <= minimum and vs < 0.0:
                self.minimum_reached_time = self.time_s
                self._set_message(f"MINIMUM {minimum:.0f} FT: go around", Severity.INFO)
            return
        if self._go_around_reported:
            return
        self._lowest_after_minimum_ft = min(self._lowest_after_minimum_ft, alt)
        if vs > self.GO_AROUND_VS_FPM:
            self._go_around_reported = True
            dip = minimum - self._lowest_after_minimum_ft
            delay = self.time_s - self.minimum_reached_time
            text = f"GO-AROUND: lowest {self._lowest_after_minimum_ft:.0f} ft ({dip:.0f} ft below minimum)"
            severity = Severity.INFO
            if dip > self.MAX_DIP_BELOW_MINIMUM_FT:
                text = f"LATE GO-AROUND: {dip:.0f} ft below minimum, {delay:.1f} s to climb"
                severity = Severity.CAUTION
            self._set_message(text, severity)

    # --- Derived values for displays -------------------------------------------------

    def nav_readings(self) -> NavReadings:
        s = self.state
        return self.nav.read(self.cockpit, s.x_m, s.y_m, s.h_m)

    @property
    def cas_kt(self) -> float:
        return cas_from_tas(self.state.tas, isa(self.state.h_m)) / KT

    @property
    def tas_kt(self) -> float:
        return self.state.tas / KT

    @property
    def alt_ft(self) -> float:
        return self.state.h_m / FT

    @property
    def agl_ft(self) -> float:
        """Radio altitude: height of the main wheels above the ground (flat terrain at field elevation)."""
        return (self.state.h_m - self.airport.elevation_m - self.WHEEL_HEIGHT_M) / FT

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
        return math.degrees(math.atan2(ve, vn)) % 360.0

    @property
    def groundspeed_kt(self) -> float:
        return math.hypot(*self.ground_velocity) / KT

    @property
    def wind_from_deg(self) -> float:
        return math.degrees(math.atan2(-self.wind[0], -self.wind[1])) % 360.0

    @property
    def wind_kt(self) -> float:
        return math.hypot(*self.wind) / KT

    def speed_limits(self) -> SpeedLimits:
        return speed_limits(self.aircraft, self.mass, self.flap_pos, self.gear_pos, self.state.h_m)
