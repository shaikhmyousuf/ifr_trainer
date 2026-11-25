"""Generic light business jet on Cessna Citation II data: two turbofans, direct controls,
glass cockpit (PFD / ND, adapted from the A320's)."""

from typing import TYPE_CHECKING

from ifr_trainer.data_files import load_toml
from ifr_trainer.sim.aero import AeroModel
from ifr_trainer.sim.aircraft import Aircraft
from ifr_trainer.sim.airframe import Airframe
from ifr_trainer.sim.direct_controls import DirectControls
from ifr_trainer.sim.turbofan import Turbofan, TurbofanData

from .cockpit import CitationCockpit
from .speeds import SpeedLimits, speed_limits

if TYPE_CHECKING:
    from ifr_trainer.displays.common import Text
    from ifr_trainer.sim.simulation import Simulation

    from .displays.panel import CitationPanel


class Citation(Aircraft):
    id = "citation"
    title = "Business jet (Cessna Citation II data)"

    def __init__(self):
        data = load_toml("aircraft.toml", package=__name__)
        self.airframe = Airframe.from_dict(data)
        self.engines = TurbofanData.from_dict(data["engines"])

    def new_propulsion(self) -> Turbofan:
        return Turbofan(self.engines)

    def new_flight_controls(self, aero: AeroModel) -> DirectControls:
        return DirectControls(self.airframe, aero)

    def new_cockpit(self, settings: dict) -> CitationCockpit:
        return CitationCockpit(**settings)

    def new_panel(self, text: "Text") -> "CitationPanel":
        from .displays.panel import CitationPanel  # pygame is only needed with a window

        return CitationPanel(text)

    def scenario_settings(self) -> dict[str, dict]:
        return load_toml("scenarios.toml", package=__name__)

    def speed_limits(self, sim: "Simulation") -> SpeedLimits:
        return speed_limits(self.airframe, sim.mass, sim.flap_pos, sim.gear_pos, sim.state.h_m)


def build() -> Citation:
    return Citation()
