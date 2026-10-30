"""Navigation receivers: what the VOR, DME, ILS and ADF receivers indicate at a position.

Positions are in the local frame (x east, y north, h MSL, metres). Magnetic
variation is zero at the training airport, so bearings are true = magnetic.
"""

import math
from dataclasses import dataclass

from ..sim.cockpit import Cockpit
from ..sim.units import FT, NM, wrap_180
from .navaids import Ils, Navaid, Ndb, Vor

VOR_DEG_PER_DOT = 5.0
GS_DEG_PER_DOT = 0.35  # 2 dots = 0.7 degrees, full scale for a 3 degree glideslope
VOR_CONE_DEG = 60.0  # above this elevation angle the VOR gives no usable radial
LOC_NARROW_DEG, LOC_NARROW_NM = 10.0, 25.0
LOC_WIDE_DEG, LOC_WIDE_NM = 35.0, 17.0
GS_AZIMUTH_DEG, GS_RANGE_NM = 8.0, 10.0
DME_RANGE_NM = 60.0
FREQ_TOLERANCE = 0.004


@dataclass(frozen=True)
class VorReading:
    ident: str
    radial: float  # degrees FROM the station
    bearing: float  # degrees TO the station
    to_from: str | None  # "TO" / "FROM", None in the cone of confusion
    deviation_dots: float | None  # for the selected course; + = course is to the right
    dme_nm: float | None


@dataclass(frozen=True)
class IlsReading:
    ident: str
    loc_dots: float | None  # + = localizer to the right (fly right)
    gs_dots: float | None  # + = aircraft above the glideslope (fly down)
    dme_nm: float | None


@dataclass(frozen=True)
class AdfReading:
    ident: str
    bearing: float  # degrees TO the station


@dataclass(frozen=True)
class NavReadings:
    vor1: VorReading | None
    vor2: VorReading | None
    ils: IlsReading | None
    adf1: AdfReading | None
    adf2: AdfReading | None


def line_of_sight_nm(height_ft: float) -> float:
    return 1.23 * math.sqrt(max(height_ft, 0.0))


def _bearing(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    return math.degrees(math.atan2(to_x - from_x, to_y - from_y)) % 360.0


def _slant_nm(sx: float, sy: float, sh: float, x: float, y: float, h: float) -> float:
    return math.sqrt((x - sx) ** 2 + (y - sy) ** 2 + (h - sh) ** 2) / NM


def _in_range(range_nm: float, distance_nm: float, h: float, station_h: float) -> bool:
    return distance_nm <= min(range_nm, line_of_sight_nm((h - station_h) / FT))


def read_vor(vor: Vor, course: float, x: float, y: float, h: float) -> VorReading | None:
    horizontal = math.hypot(x - vor.x_m, y - vor.y_m)
    if not _in_range(vor.range_nm, horizontal / NM, h, vor.elevation_m):
        return None
    radial = _bearing(vor.x_m, vor.y_m, x, y)
    bearing = (radial + 180.0) % 360.0
    elevation = math.degrees(math.atan2(h - vor.elevation_m, horizontal))
    dme = _slant_nm(vor.x_m, vor.y_m, vor.elevation_m, x, y, h) if vor.has_dme else None
    if elevation > VOR_CONE_DEG:
        return VorReading(vor.ident, radial, bearing, None, None, dme)
    off = wrap_180(bearing - course)
    if abs(off) < 90.0:
        to_from, needle = "TO", off
    else:
        to_from, needle = "FROM", -wrap_180(radial - course)
    return VorReading(vor.ident, radial, bearing, to_from, needle / VOR_DEG_PER_DOT, dme)


def read_ils(ils: Ils, x: float, y: float, h: float) -> IlsReading | None:
    # Localizer: angle of the aircraft off the approach course, seen from the antenna.
    loc_dist_nm = math.hypot(x - ils.localizer_x_m, y - ils.localizer_y_m) / NM
    from_loc = _bearing(ils.localizer_x_m, ils.localizer_y_m, x, y)
    theta = wrap_180(from_loc - (ils.course_deg + 180.0))
    in_front = abs(theta) < 90.0
    loc_ok = in_front and (
        (abs(theta) <= LOC_NARROW_DEG and loc_dist_nm <= LOC_NARROW_NM)
        or (abs(theta) <= LOC_WIDE_DEG and loc_dist_nm <= LOC_WIDE_NM)
    )
    loc_ok = loc_ok and _in_range(LOC_NARROW_NM, loc_dist_nm, h, ils.elevation_m)
    if not loc_ok:
        return None
    loc_dots = theta / (ils.half_width_deg / 2.0)

    gs_horizontal = math.hypot(x - ils.glideslope_x_m, y - ils.glideslope_y_m)
    gs_dots = None
    if abs(theta) <= GS_AZIMUTH_DEG and gs_horizontal / NM <= GS_RANGE_NM:
        elevation = math.degrees(math.atan2(h - ils.elevation_m, gs_horizontal))
        gs_dots = (elevation - ils.glideslope_deg) / GS_DEG_PER_DOT
    dme = None
    if ils.has_dme:
        dme = _slant_nm(ils.glideslope_x_m, ils.glideslope_y_m, ils.elevation_m, x, y, h)
    return IlsReading(ils.ident, loc_dots, gs_dots, dme)


def read_adf(ndb: Ndb, x: float, y: float, h: float) -> AdfReading | None:
    if not _in_range(ndb.range_nm, math.hypot(x - ndb.x_m, y - ndb.y_m) / NM, h, ndb.elevation_m):
        return None
    return AdfReading(ndb.ident, _bearing(x, y, ndb.x_m, ndb.y_m))


class NavReceivers:
    def __init__(self, navaids: tuple[Navaid, ...]):
        self.navaids = navaids

    def tuned(self, kind: type, frequency: float) -> Navaid | None:
        return next(
            (n for n in self.navaids if isinstance(n, kind) and abs(n.frequency - frequency) < FREQ_TOLERANCE), None
        )

    def read(self, c: Cockpit, x: float, y: float, h: float) -> NavReadings:
        vor1, vor2 = self.tuned(Vor, c.vor1_mhz), self.tuned(Vor, c.vor2_mhz)
        ils = self.tuned(Ils, c.ils_mhz)
        adf1, adf2 = self.tuned(Ndb, c.adf1_khz), self.tuned(Ndb, c.adf2_khz)
        return NavReadings(
            vor1=read_vor(vor1, c.vor1_crs, x, y, h) if vor1 else None,
            vor2=read_vor(vor2, c.vor2_crs, x, y, h) if vor2 else None,
            ils=read_ils(ils, x, y, h) if ils else None,
            adf1=read_adf(adf1, x, y, h) if adf1 else None,
            adf2=read_adf(adf2, x, y, h) if adf2 else None,
        )
