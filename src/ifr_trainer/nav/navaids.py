"""Ground navaids: VOR/DME, ILS (localizer + glideslope + DME) and NDB."""

import math
from dataclasses import dataclass

LOC_HALF_WIDTH_AT_THRESHOLD_M = 105.0  # ICAO: full-scale deflection 105 m either side at the threshold


@dataclass(frozen=True)
class Vor:
    ident: str
    frequency: float  # MHz
    x_m: float
    y_m: float
    elevation_m: float
    has_dme: bool = True
    range_nm: float = 60.0


@dataclass(frozen=True)
class Ils:
    ident: str
    frequency: float  # MHz
    runway: str
    course_deg: float
    glideslope_deg: float
    localizer_x_m: float
    localizer_y_m: float
    glideslope_x_m: float
    glideslope_y_m: float
    elevation_m: float
    half_width_deg: float  # localizer half sector (full-scale deflection)
    has_dme: bool = True


@dataclass(frozen=True)
class Ndb:
    ident: str
    frequency: float  # kHz
    x_m: float
    y_m: float
    elevation_m: float
    range_nm: float = 25.0


Navaid = Vor | Ils | Ndb


def navaid_from_dict(d: dict, elevation_m: float, runway_thresholds: dict[str, tuple[float, float]]) -> Navaid:
    kind = d["type"]
    if kind == "VORDME":
        return Vor(d["ident"], d["frequency"], d["x_m"], d["y_m"], elevation_m)
    if kind == "NDB":
        return Ndb(d["ident"], d["frequency"], d["x_m"], d["y_m"], elevation_m)
    if kind == "ILS":
        tx, ty = runway_thresholds[d["runway"]]
        to_threshold = math.hypot(d["localizer_x_m"] - tx, d["localizer_y_m"] - ty)
        half_width = math.degrees(math.atan2(LOC_HALF_WIDTH_AT_THRESHOLD_M, to_threshold))
        return Ils(
            ident=d["ident"],
            frequency=d["frequency"],
            runway=d["runway"],
            course_deg=d["course_deg"],
            glideslope_deg=d["glideslope_deg"],
            localizer_x_m=d["localizer_x_m"],
            localizer_y_m=d["localizer_y_m"],
            glideslope_x_m=d["glideslope_x_m"],
            glideslope_y_m=d["glideslope_y_m"],
            elevation_m=elevation_m,
            half_width_deg=half_width,
        )
    raise ValueError(f"unknown navaid type {kind!r}")
