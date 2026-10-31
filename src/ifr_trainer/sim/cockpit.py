"""Pilot-set cockpit state shared by every aircraft: nav radios, references and the minimum.

Aircraft modules subclass ``Cockpit`` to add their own settings (EFIS modes, for example).
"""

from dataclasses import dataclass


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
    hdg_bug: float = 360.0  # selected heading (a reference only; there is no autopilot)
    alt_target_ft: float = 3000.0
    minimum_ft: float | None = None  # decision altitude / minimum descent altitude
