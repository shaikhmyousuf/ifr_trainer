"""Start conditions loaded from ``scenarios.toml``."""

from dataclasses import dataclass
from pathlib import Path

from ..data_files import load_toml


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    runway: str
    distance_nm: float
    altitude_ft: float
    cas_kt: float
    flaps: int = 0
    gear_down: bool = False
    heading_deg: float | None = None
    wind_dir_deg: float = 0.0
    wind_kt: float = 0.0
    mass_kg: float | None = None


def load_scenarios(path: Path | None = None) -> dict[str, Scenario]:
    return {name: Scenario(name=name, **fields) for name, fields in load_toml("scenarios.toml", path).items()}
