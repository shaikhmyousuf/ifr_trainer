"""Start conditions loaded from ``scenarios.toml``."""

from dataclasses import dataclass, field
from pathlib import Path

from ..data_files import load_toml


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    runway: str  # reference runway for the start position
    distance_nm: float  # start position: before the threshold on the extended centreline...
    altitude_ft: float  # MSL
    cas_kt: float
    offset_nm: float = 0.0  # ...and this far right of it (negative = left)
    heading_deg: float | None = None  # defaults to the runway heading
    flaps: int = 0
    gear_down: bool = False
    flight_path_deg: float = 0.0
    wind_dir_deg: float = 0.0
    wind_kt: float = 0.0
    mass_kg: float | None = None
    cockpit: dict = field(default_factory=dict)  # Cockpit fields: radios, ND, references, minimum


def load_scenarios(path: Path | None = None) -> dict[str, Scenario]:
    return {name: Scenario(name=name, **fields) for name, fields in load_toml("scenarios.toml", path).items()}
