"""Cessna 172P Skyhawk: piston engine and fixed-pitch propeller, direct controls, steam gauges."""

from typing import TYPE_CHECKING

from ifr_trainer.data_files import load_toml
from ifr_trainer.sim.aero import AeroModel
from ifr_trainer.sim.aircraft import Aircraft
from ifr_trainer.sim.airframe import Airframe
from ifr_trainer.sim.cockpit import Nav1Cockpit
from ifr_trainer.sim.direct_controls import DirectControls

from .piston import PistonData, PistonPropeller

if TYPE_CHECKING:
    from ifr_trainer.displays.common import Text

    from .displays.panel import C172Panel


class C172(Aircraft):
    id = "c172"
    title = "Cessna 172P Skyhawk"

    def __init__(self):
        data = load_toml("aircraft.toml", package=__name__)
        self.airframe = Airframe.from_dict(data)
        self.engine = PistonData.from_dict(data["engine"])
        self.speed_marks = data["speeds"]

    def new_propulsion(self) -> PistonPropeller:
        return PistonPropeller(self.engine)

    def new_flight_controls(self, aero: AeroModel) -> DirectControls:
        return DirectControls(self.airframe, aero)

    def new_cockpit(self, settings: dict) -> Nav1Cockpit:
        return Nav1Cockpit(**settings)

    def new_panel(self, text: "Text") -> "C172Panel":
        from .displays.panel import C172Panel  # pygame is only needed with a window

        return C172Panel(text, self.speed_marks)

    def scenario_settings(self) -> dict[str, dict]:
        return load_toml("scenarios.toml", package=__name__)


def build() -> C172:
    return C172()
