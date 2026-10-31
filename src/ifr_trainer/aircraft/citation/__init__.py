"""Generic light business jet on Cessna Citation II data: two turbofans, direct controls,
round-dial panel."""

from typing import TYPE_CHECKING

from ifr_trainer.data_files import load_toml
from ifr_trainer.sim.aero import AeroModel
from ifr_trainer.sim.aircraft import Aircraft
from ifr_trainer.sim.airframe import Airframe
from ifr_trainer.sim.cockpit import Nav1Cockpit
from ifr_trainer.sim.direct_controls import DirectControls
from ifr_trainer.sim.turbofan import Turbofan, TurbofanData

if TYPE_CHECKING:
    from ifr_trainer.displays.common import Text

    from .displays.panel import CitationPanel


class Citation(Aircraft):
    id = "citation"
    title = "Business jet (Cessna Citation II data)"

    def __init__(self):
        data = load_toml("aircraft.toml", package=__name__)
        self.airframe = Airframe.from_dict(data)
        self.engines = TurbofanData.from_dict(data["engines"])
        self.speed_marks = data["speeds"]

    def new_propulsion(self) -> Turbofan:
        return Turbofan(self.engines)

    def new_flight_controls(self, aero: AeroModel) -> DirectControls:
        return DirectControls(self.airframe, aero)

    def new_cockpit(self, settings: dict) -> Nav1Cockpit:
        return Nav1Cockpit(**settings)

    def new_panel(self, text: "Text") -> "CitationPanel":
        from .displays.panel import CitationPanel  # pygame is only needed with a window

        return CitationPanel(text, self.speed_marks)

    def scenario_settings(self) -> dict[str, dict]:
        return load_toml("scenarios.toml", package=__name__)


def build() -> Citation:
    return Citation()
