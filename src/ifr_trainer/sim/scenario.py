"""Start conditions: the shared ``data/scenarios.toml`` merged with each aircraft's settings.

The shared file holds what is common to every aircraft: the exercise, position, altitude,
wind, radios and minimum. An aircraft's ``scenarios.toml`` adds how it flies the exercise
(speed, flaps, gear, its own cockpit settings) and may override anything else. Only the
scenarios an aircraft lists are offered for it; it may also define scenarios of its own.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from ..data_files import load_toml

if TYPE_CHECKING:
    from .aircraft import Aircraft


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
    cockpit: dict = field(default_factory=dict)  # cockpit fields: radios, references, minimum, displays


def merge_scenarios(shared: dict[str, dict], settings: dict[str, dict]) -> dict[str, Scenario]:
    """Merge an aircraft's ``settings`` over the ``shared`` scenarios; cockpit tables merge key by key."""
    names = [n for n in shared if n in settings] + [n for n in settings if n not in shared]
    scenarios = {}
    for name in names:
        base, own = shared.get(name, {}), settings[name]
        fields = {**base, **own, "cockpit": {**base.get("cockpit", {}), **own.get("cockpit", {})}}
        scenarios[name] = Scenario(name=name, **fields)
    return scenarios


def load_scenarios(aircraft: "Aircraft", path: Path | None = None) -> dict[str, Scenario]:
    return merge_scenarios(load_toml("data/scenarios.toml", path), aircraft.scenario_settings())
