"""Start conditions loaded from ``scenarios.toml``."""

from dataclasses import dataclass
from pathlib import Path

from ..data_files import load_toml


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    runway: str
    distance_nm: float  # before the threshold on the extended centreline (negative = past it)
    altitude_ft: float = 0.0  # MSL; ignored when on_ground
    cas_kt: float = 0.0
    flaps: int = 0
    gear_down: bool = False
    on_ground: bool = False
    flight_path_deg: float = 0.0
    heading_deg: float | None = None
    ths_deg: float = 0.0  # takeoff trim for ground starts
    wind_dir_deg: float = 0.0
    wind_kt: float = 0.0
    mass_kg: float | None = None


def load_scenarios(path: Path | None = None) -> dict[str, Scenario]:
    return {name: Scenario(name=name, **fields) for name, fields in load_toml("scenarios.toml", path).items()}
