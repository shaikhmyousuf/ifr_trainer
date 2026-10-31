"""C172 cockpit state: NAV1 (VOR or localizer + glideslope), NAV2 (VOR), ADF, heading bug.

The shared cockpit has separate VOR1 and ILS receivers, as an Airbus does. In the Cessna,
NAV1 is one receiver that tunes either, so the ILS frequency always follows NAV1.
"""

from dataclasses import dataclass

from ifr_trainer.sim.cockpit import Cockpit


@dataclass
class C172Cockpit(Cockpit):
    def __post_init__(self) -> None:
        self.ils_mhz = self.vor1_mhz

    def __setattr__(self, name: str, value) -> None:
        super().__setattr__(name, value)
        if name == "vor1_mhz":
            super().__setattr__("ils_mhz", value)
