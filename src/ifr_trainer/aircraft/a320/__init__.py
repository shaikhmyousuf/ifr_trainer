"""Airbus A320 (loosely based): turbofans, fly-by-wire normal law, PFD / ND / lower panel."""

from typing import TYPE_CHECKING

from ifr_trainer.data_files import load_toml
from ifr_trainer.sim.aero import AeroModel
from ifr_trainer.sim.aircraft import Aircraft
from ifr_trainer.sim.airframe import Airframe
from ifr_trainer.sim.turbofan import Turbofan, TurbofanData

from .cockpit import A320Cockpit
from .fbw.flight_control import FlightControlComputer
from .fbw.normal_law import NormalLawLimits
from .speeds import SpeedLimits, speed_limits

if TYPE_CHECKING:
    from ifr_trainer.displays.common import Text
    from ifr_trainer.sim.simulation import Simulation

    from .displays.panel import A320Panel


class A320(Aircraft):
    id = "a320"
    title = "Airbus A320 (loosely based)"

    def __init__(self, law_limits: NormalLawLimits | None = None):
        data = load_toml("aircraft.toml", package=__name__)
        self.airframe = Airframe.from_dict(data)
        self.engines = TurbofanData.from_dict(data["engines"])
        self.vls_factors = tuple(data["speeds"]["vls_factors"])
        self.law_limits = law_limits or NormalLawLimits()

    def new_propulsion(self) -> Turbofan:
        return Turbofan(self.engines)

    def new_flight_controls(self, aero: AeroModel) -> FlightControlComputer:
        return FlightControlComputer(self.airframe, aero, self.law_limits, self.engines.z_m)

    def new_cockpit(self, settings: dict) -> A320Cockpit:
        return A320Cockpit(**settings)

    def new_panel(self, text: "Text") -> "A320Panel":
        from .displays.panel import A320Panel  # pygame is only needed with a window

        return A320Panel(text)

    def scenario_settings(self) -> dict[str, dict]:
        return load_toml("scenarios.toml", package=__name__)

    def speed_limits(self, sim: "Simulation") -> SpeedLimits:
        return speed_limits(self.airframe, self.vls_factors, sim.mass, sim.flap_pos, sim.gear_pos, sim.state.h_m)


def build() -> A320:
    return A320()
