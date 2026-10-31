"""Airport, runways and navaids loaded from ``airport.toml``."""

import math
from dataclasses import dataclass
from pathlib import Path

from ..data_files import load_toml
from ..nav.navaids import Navaid, navaid_from_dict
from .units import FT


@dataclass(frozen=True)
class Runway:
    ident: str
    heading_deg: float
    threshold_x_m: float
    threshold_y_m: float
    length_m: float
    width_m: float

    def to_runway_frame(self, x: float, y: float) -> tuple[float, float]:
        """(distance past threshold along the runway, offset right of centreline) in metres."""
        h = math.radians(self.heading_deg)
        dx, dy = x - self.threshold_x_m, y - self.threshold_y_m
        return dx * math.sin(h) + dy * math.cos(h), dx * math.cos(h) - dy * math.sin(h)

    def contains(self, x: float, y: float) -> bool:
        along, cross = self.to_runway_frame(x, y)
        return 0.0 <= along <= self.length_m and abs(cross) <= self.width_m / 2.0


@dataclass(frozen=True)
class Airport:
    ident: str
    name: str
    elevation_ft: float
    magnetic_variation_deg: float
    runways: tuple[Runway, ...]
    navaids: tuple[Navaid, ...]

    @property
    def elevation_m(self) -> float:
        return self.elevation_ft * FT

    def runway(self, ident: str) -> Runway:
        for r in self.runways:
            if r.ident == ident:
                return r
        raise KeyError(f"no runway {ident!r} at {self.ident}")

    def runway_at(self, x: float, y: float) -> Runway | None:
        return next((r for r in self.runways if r.contains(x, y)), None)

    @classmethod
    def load(cls, path: Path | None = None) -> "Airport":
        d = load_toml("data/airport.toml", path)
        runways = tuple(Runway(**r) for r in d["runways"])
        thresholds = {r.ident: (r.threshold_x_m, r.threshold_y_m) for r in runways}
        elevation_m = d["elevation_ft"] * FT
        return cls(
            ident=d["ident"],
            name=d["name"],
            elevation_ft=d["elevation_ft"],
            magnetic_variation_deg=d["magnetic_variation_deg"],
            runways=runways,
            navaids=tuple(navaid_from_dict(n, elevation_m, thresholds) for n in d.get("navaids", ())),
        )
