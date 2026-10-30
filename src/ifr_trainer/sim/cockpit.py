"""Pilot-set cockpit state: nav radios, EFIS (ND) settings and FCU references."""

from dataclasses import dataclass

ND_MODES = ("ROSE ILS", "ROSE VOR", "ARC")
ND_RANGES = (10, 20, 40, 80, 160)
NEEDLE1 = ("VOR1", "ADF1", "OFF")
NEEDLE2 = ("VOR2", "ADF2", "OFF")


@dataclass
class Cockpit:
    vor1_mhz: float = 113.40
    vor1_crs: float = 360.0
    vor2_mhz: float = 113.40
    vor2_crs: float = 360.0
    ils_mhz: float = 110.30
    ils_crs: float = 360.0
    adf1_khz: float = 380.0
    adf2_khz: float = 380.0
    nd_mode: str = "ROSE ILS"
    nd_range: int = 20
    needle1: str = "VOR1"
    needle2: str = "ADF2"
    ls: bool = True  # ILS scales on the PFD
    hdg_bug: float = 360.0  # FCU selected heading (a reference only; there is no autopilot)
    alt_target_ft: float = 3000.0
    minimum_ft: float | None = None  # BARO decision altitude / minimum descent altitude
