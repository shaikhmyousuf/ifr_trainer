"""Business jet cockpit state: the shared radios and references, plus EFIS (ND) settings.
The same as the A320's: separate VOR1, VOR2 and ILS receivers."""

from dataclasses import dataclass

from ifr_trainer.sim.cockpit import Cockpit

ND_MODES = ("ROSE ILS", "ROSE VOR", "ARC")
ND_RANGES = (10, 20, 40, 80, 160)
NEEDLE1 = ("VOR1", "ADF1", "OFF")
NEEDLE2 = ("VOR2", "ADF2", "OFF")


@dataclass
class CitationCockpit(Cockpit):
    nd_mode: str = "ROSE ILS"
    nd_range: int = 20
    needle1: str = "VOR1"
    needle2: str = "ADF2"
    ls: bool = True  # ILS scales on the PFD
    fpv: bool = False  # flight path vector (the bird) on the PFD
