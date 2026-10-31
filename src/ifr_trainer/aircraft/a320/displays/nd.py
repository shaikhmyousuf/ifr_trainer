"""Navigation display, A320 style: ROSE ILS, ROSE VOR and ARC (map) modes."""

import math

import pygame

from ifr_trainer.displays.common import AMBER, BLACK, CYAN, GREEN, GREY, MAGENTA, RED, WHITE, YELLOW, Text
from ifr_trainer.nav.navaids import Ils, Ndb, Vor
from ifr_trainer.nav.receivers import AdfReading, NavReadings, VorReading
from ifr_trainer.sim.simulation import Simulation
from ifr_trainer.sim.units import NM

SIZE = (600, 600)
ROSE_CENTRE = (300, 320)
ROSE_RADIUS = 210
ARC_CENTRE = (300, 530)
ARC_RADIUS = 420
ARC_HALF_ANGLE = 50.0
DOT_PX = 40
GS_SCALE_X = 565
MAX_DOTS = 2.3
NEEDLE_COLOURS = {"VOR": WHITE, "ADF": GREEN}


class ND:
    def __init__(self, text: Text):
        self.text = text
        self.surface = pygame.Surface(SIZE)

    def draw(self, sim: Simulation, nav: NavReadings) -> pygame.Surface:
        s = self.surface
        s.fill(BLACK)
        c = sim.cockpit
        hdg = sim.heading_deg
        if c.nd_mode == "ARC":
            centre, radius = ARC_CENTRE, ARC_RADIUS
            self._map(s, sim, centre, radius, hdg)
            self._arc_scale(s, centre, radius, hdg, sim)
        else:
            centre, radius = ROSE_CENTRE, ROSE_RADIUS
            self._rose_scale(s, centre, radius, hdg, sim)
            if c.nd_mode == "ROSE ILS":
                dots = nav.ils.loc_dots if nav.ils else None
                self._course_pointer(s, centre, radius, hdg, c.ils_crs, dots, None, MAGENTA, "LOC")
                self._glideslope(s, nav)
            else:
                v = nav.vor1
                dots = v.deviation_dots if v else None
                self._course_pointer(s, centre, radius, hdg, c.vor1_crs, dots, v.to_from if v else None, CYAN, "VOR1")
        self._needles(s, centre, radius, hdg, nav, c.needle1, c.needle2, single_ring=c.nd_mode != "ARC")
        self._aircraft(s, centre)
        self._header(s, sim, nav)
        self._needle_info(s, nav, c.needle1, left=True)
        self._needle_info(s, nav, c.needle2, left=False)
        return s

    # --- Geometry ---------------------------------------------------------------------

    @staticmethod
    def _pt(centre: tuple[int, int], bearing: float, hdg: float, r: float) -> tuple[float, float]:
        a = math.radians(bearing - hdg)
        return centre[0] + r * math.sin(a), centre[1] - r * math.cos(a)

    # --- Compass scales ----------------------------------------------------------------

    def _rose_scale(self, s: pygame.Surface, centre, radius: int, hdg: float, sim: Simulation) -> None:
        pygame.draw.circle(s, WHITE, centre, radius, 2)
        for b in range(0, 360, 5):
            outer = self._pt(centre, b, hdg, radius)
            inner = self._pt(centre, b, hdg, radius - (14 if b % 10 == 0 else 8))
            pygame.draw.line(s, WHITE, outer, inner, 2)
            if b % 30 == 0:
                label = pygame.transform.rotate(self.text.render(str(b // 10), 18, WHITE), hdg - b)
                pos = self._pt(centre, b, hdg, radius - 32)
                s.blit(label, label.get_rect(center=(round(pos[0]), round(pos[1]))))
        self._heading_marks(s, centre, radius, hdg, sim)

    def _arc_scale(self, s: pygame.Surface, centre, radius: int, hdg: float, sim: Simulation) -> None:
        pts = [self._pt(centre, hdg + a, hdg, radius) for a in range(-int(ARC_HALF_ANGLE), int(ARC_HALF_ANGLE) + 1, 2)]
        pygame.draw.lines(s, WHITE, False, pts, 2)
        for b in range(math.ceil(hdg - ARC_HALF_ANGLE), math.floor(hdg + ARC_HALF_ANGLE) + 1):
            if b % 5:
                continue
            pygame.draw.line(
                s, WHITE, self._pt(centre, b, hdg, radius), self._pt(centre, b, hdg, radius + (14 if b % 10 == 0 else 8)), 2
            )
            if b % 30 == 0:
                label = pygame.transform.rotate(self.text.render(str((b % 360) // 10), 18, WHITE), hdg - b)
                pos = self._pt(centre, b, hdg, radius + 30)
                s.blit(label, label.get_rect(center=(round(pos[0]), round(pos[1]))))
        # Range rings: half and full range.
        rng = sim.cockpit.nd_range
        for frac in (0.5, 0.25, 0.75):
            ring = [self._pt(centre, hdg + a, hdg, radius * frac) for a in range(-50, 51, 4)]
            for i in range(0, len(ring) - 1, 2):
                pygame.draw.line(s, GREY, ring[i], ring[i + 1], 1)
        self.text.blit(s, f"{rng / 2:g}", 14, CYAN, self._pt(centre, hdg - 52, hdg, radius * 0.5))
        self._heading_marks(s, centre, radius, hdg, sim)

    def _heading_marks(self, s: pygame.Surface, centre, radius: int, hdg: float, sim: Simulation) -> None:
        # Lubber line (yellow) at the top, track diamond (green), FCU heading (cyan triangle).
        top = (centre[0], centre[1] - radius)
        pygame.draw.line(s, YELLOW, (top[0], top[1] - 16), (top[0], top[1] + 16), 4)
        tx, ty = self._pt(centre, sim.track_deg, hdg, radius - 6)
        pygame.draw.polygon(s, GREEN, [(tx, ty - 7), (tx + 6, ty), (tx, ty + 7), (tx - 6, ty)], 2)
        bug = sim.cockpit.hdg_bug
        tip = self._pt(centre, bug, hdg, radius + 2)
        left = self._pt(centre, bug - 2.5, hdg, radius + 16)
        right = self._pt(centre, bug + 2.5, hdg, radius + 16)
        pygame.draw.polygon(s, CYAN, [tip, left, right], 2)

    # --- Course pointer, deviation and glideslope ---------------------------------------

    def _course_pointer(
        self,
        s: pygame.Surface,
        centre,
        radius: int,
        hdg: float,
        course: float,
        dots: float | None,
        to_from: str | None,
        colour,
        flag: str,
    ) -> None:
        a = math.radians(course - hdg)
        ux, uy = math.sin(a), -math.cos(a)  # along the course
        px, py = math.cos(a), math.sin(a)  # to the right of the course
        cx, cy = centre
        for d in (-2, -1, 1, 2):
            pygame.draw.circle(s, WHITE, (round(cx + px * d * DOT_PX), round(cy + py * d * DOT_PX)), 5, 2)
        inner, outer = radius * 0.45, radius * 0.92
        head = (cx + ux * outer, cy + uy * outer)
        pygame.draw.line(s, colour, (cx + ux * inner, cy + uy * inner), head, 4)
        pygame.draw.line(s, colour, (cx - ux * inner, cy - uy * inner), (cx - ux * outer, cy - uy * outer), 4)
        pygame.draw.polygon(
            s, colour,
            [(head[0] + ux * 14, head[1] + uy * 14), (head[0] + px * 9, head[1] + py * 9), (head[0] - px * 9, head[1] - py * 9)],
        )
        if dots is None:
            self.text.blit(s, flag, 22, RED, (cx + 70, cy - 40))
            return
        d = max(-MAX_DOTS, min(MAX_DOTS, dots)) * DOT_PX
        bx, by = cx + px * d, cy + py * d
        length = inner - 8
        pygame.draw.line(s, colour, (bx - ux * length, by - uy * length), (bx + ux * length, by + uy * length), 4)
        if to_from:
            sign = 1 if to_from == "TO" else -1
            tip = (bx + ux * sign * (length * 0.6 + 12), by + uy * sign * (length * 0.6 + 12))
            base = (bx + ux * sign * length * 0.6, by + uy * sign * length * 0.6)
            pygame.draw.polygon(s, WHITE, [tip, (base[0] + px * 9, base[1] + py * 9), (base[0] - px * 9, base[1] - py * 9)], 2)

    def _glideslope(self, s: pygame.Surface, nav: NavReadings) -> None:
        cy = ROSE_CENTRE[1]
        for d in (-2, -1, 1, 2):
            pygame.draw.circle(s, WHITE, (GS_SCALE_X, cy + d * DOT_PX), 5, 2)
        pygame.draw.line(s, YELLOW, (GS_SCALE_X - 14, cy), (GS_SCALE_X + 14, cy), 3)
        gs = nav.ils.gs_dots if nav.ils else None
        if gs is None:
            self.text.blit(s, "GS", 18, RED, (GS_SCALE_X, cy - 3 * DOT_PX))
            return
        y = cy + max(-MAX_DOTS, min(MAX_DOTS, gs)) * DOT_PX
        pygame.draw.polygon(s, MAGENTA, [(GS_SCALE_X, y - 11), (GS_SCALE_X + 8, y), (GS_SCALE_X, y + 11), (GS_SCALE_X - 8, y)], 3)

    # --- Bearing needles ----------------------------------------------------------------

    @staticmethod
    def _needle_reading(nav: NavReadings, selection: str) -> VorReading | AdfReading | None:
        return {"VOR1": nav.vor1, "VOR2": nav.vor2, "ADF1": nav.adf1, "ADF2": nav.adf2}.get(selection)

    def _needles(self, s: pygame.Surface, centre, radius: int, hdg: float, nav: NavReadings, n1: str, n2: str, single_ring: bool) -> None:
        r_out = radius * (0.97 if single_ring else 0.6)
        r_in = radius * (0.6 if single_ring else 0.2)
        for selection, double in ((n1, False), (n2, True)):
            reading = self._needle_reading(nav, selection)
            if reading is None:
                continue
            colour = NEEDLE_COLOURS[selection[:3]]
            b = reading.bearing
            head_out, head_in = self._pt(centre, b, hdg, r_out), self._pt(centre, b, hdg, r_in)
            tail_out, tail_in = self._pt(centre, b + 180, hdg, r_out), self._pt(centre, b + 180, hdg, r_in)
            a = math.radians(b - hdg)
            px, py = math.cos(a) * 5, math.sin(a) * 5
            offsets = ((-px, -py), (px, py)) if double else ((0.0, 0.0),)
            for ox, oy in offsets:
                pygame.draw.line(s, colour, (head_in[0] + ox, head_in[1] + oy), (head_out[0] + ox, head_out[1] + oy), 2)
                pygame.draw.line(s, colour, (tail_in[0] + ox, tail_in[1] + oy), (tail_out[0] + ox, tail_out[1] + oy), 2)
            ux, uy = math.sin(a), -math.cos(a)
            tip = (head_out[0], head_out[1])
            pygame.draw.lines(s, colour, False, [(tip[0] - ux * 14 + px * 2, tip[1] - uy * 14 + py * 2), tip,
                                                  (tip[0] - ux * 14 - px * 2, tip[1] - uy * 14 - py * 2)], 2)

    # --- Map (ARC mode) -----------------------------------------------------------------

    def _map(self, s: pygame.Surface, sim: Simulation, centre, radius: int, hdg: float) -> None:
        st = sim.state
        scale = radius / (sim.cockpit.nd_range * NM)
        s.set_clip(pygame.Rect(0, 40, SIZE[0], SIZE[1] - 110))

        def to_screen(x: float, y: float) -> tuple[float, float] | None:
            dx, dy = x - st.x_m, y - st.y_m
            dist = math.hypot(dx, dy) * scale
            if dist > radius * 1.05:
                return None
            return self._pt(centre, math.degrees(math.atan2(dx, dy)), hdg, dist)

        tuned_ils = sim.nav.tuned(Ils, sim.cockpit.ils_mhz)
        for rwy in sim.airport.runways:
            h = math.radians(rwy.heading_deg)
            a = to_screen(rwy.threshold_x_m, rwy.threshold_y_m)
            b = to_screen(rwy.threshold_x_m + rwy.length_m * math.sin(h), rwy.threshold_y_m + rwy.length_m * math.cos(h))
            if a and b:
                pygame.draw.line(s, WHITE, a, b, 4)
            if tuned_ils and tuned_ils.runway == rwy.ident:
                far = 14 * NM  # localizer course line back along the approach
                pts = [to_screen(rwy.threshold_x_m - f * far * math.sin(h), rwy.threshold_y_m - f * far * math.cos(h))
                       for f in (i / 20 for i in range(21))]
                for i in range(0, 20, 2):
                    if pts[i] and pts[i + 1]:
                        pygame.draw.line(s, MAGENTA, pts[i], pts[i + 1], 2)
        tuned_vors = {sim.cockpit.vor1_mhz, sim.cockpit.vor2_mhz}
        tuned_ndbs = {sim.cockpit.adf1_khz, sim.cockpit.adf2_khz}
        for n in sim.airport.navaids:
            if isinstance(n, Vor):
                p = to_screen(n.x_m, n.y_m)
                if p:
                    colour = CYAN if any(abs(n.frequency - f) < 0.004 for f in tuned_vors) else MAGENTA
                    pygame.draw.circle(s, colour, (round(p[0]), round(p[1])), 7, 2)
                    pygame.draw.line(s, colour, (p[0] - 11, p[1]), (p[0] + 11, p[1]), 2)
                    pygame.draw.line(s, colour, (p[0], p[1] - 11), (p[0], p[1] + 11), 2)
                    self.text.blit(s, n.ident, 14, colour, (p[0] + 14, p[1] + 12), "midleft")
            elif isinstance(n, Ndb):
                p = to_screen(n.x_m, n.y_m)
                if p:
                    colour = CYAN if any(abs(n.frequency - f) < 0.004 for f in tuned_ndbs) else MAGENTA
                    pygame.draw.polygon(s, colour, [(p[0], p[1] - 9), (p[0] + 8, p[1] + 6), (p[0] - 8, p[1] + 6)], 2)
                    self.text.blit(s, n.ident, 14, colour, (p[0] + 12, p[1] + 10), "midleft")
        s.set_clip(None)

    # --- Aircraft symbol, header and needle data ---------------------------------------

    def _aircraft(self, s: pygame.Surface, centre) -> None:
        x, y = centre
        pygame.draw.line(s, YELLOW, (x, y - 14), (x, y + 22), 4)
        pygame.draw.line(s, YELLOW, (x - 22, y), (x + 22, y), 4)
        pygame.draw.line(s, YELLOW, (x - 8, y + 18), (x + 8, y + 18), 3)

    def _header(self, s: pygame.Surface, sim: Simulation, nav: NavReadings) -> None:
        t = self.text
        t.blit(s, "GS", 16, WHITE, (12, 18), "midleft")
        t.blit(s, f"{sim.groundspeed_kt:3.0f}", 18, GREEN, (40, 18), "midleft")
        t.blit(s, "TAS", 16, WHITE, (90, 18), "midleft")
        t.blit(s, f"{sim.tas_kt:3.0f}", 18, GREEN, (128, 18), "midleft")
        wind_from, wind_kt = sim.wind_from_deg, sim.wind_kt
        t.blit(s, f"{wind_from:03.0f}/{wind_kt:.0f}", 18, GREEN, (12, 42), "midleft")
        if wind_kt >= 2:
            # Arrow shows where the wind blows to, relative to the aircraft heading.
            a = math.radians(wind_from + 180 - sim.heading_deg)
            cx, cy = 30, 78
            ux, uy = math.sin(a), -math.cos(a)
            tip = (cx + ux * 16, cy + uy * 16)
            pygame.draw.line(s, GREEN, (cx - ux * 16, cy - uy * 16), tip, 3)
            pygame.draw.polygon(s, GREEN, [tip, (tip[0] - ux * 9 + uy * 6, tip[1] - uy * 9 - ux * 6),
                                           (tip[0] - ux * 9 - uy * 6, tip[1] - uy * 9 + ux * 6)])

        c = sim.cockpit
        if c.nd_mode == "ROSE ILS":
            ident = nav.ils.ident if nav.ils else ""
            self._station_block(s, "ILS1", f"{c.ils_mhz:6.2f}", c.ils_crs, ident, MAGENTA)
        elif c.nd_mode == "ROSE VOR":
            ident = nav.vor1.ident if nav.vor1 else ""
            self._station_block(s, "VOR1", f"{c.vor1_mhz:6.2f}", c.vor1_crs, ident, CYAN)
        else:
            t.blit(s, f"ARC {c.nd_range} NM", 16, GREY, (588, 18), "midright")

    def _station_block(self, s: pygame.Surface, label: str, freq: str, course: float, ident: str, colour) -> None:
        t = self.text
        t.blit(s, label, 16, WHITE, (470, 18), "midright")
        t.blit(s, freq, 18, colour, (588, 18), "midright")
        t.blit(s, "CRS", 16, WHITE, (500, 42), "midright")
        t.blit(s, f"{course % 360:03.0f}°", 18, CYAN, (588, 42), "midright")
        if ident:
            t.blit(s, ident, 18, colour, (588, 66), "midright")

    def _needle_info(self, s: pygame.Surface, nav: NavReadings, selection: str, left: bool) -> None:
        if selection == "OFF":
            return
        reading = self._needle_reading(nav, selection)
        colour = NEEDLE_COLOURS[selection[:3]]
        x = 14 if left else 586
        anchor = "midleft" if left else "midright"
        t = self.text
        t.blit(s, selection, 16, colour, (x, 540), anchor)
        if reading is None:
            t.blit(s, "----", 16, AMBER, (x, 562), anchor)
            return
        t.blit(s, reading.ident, 18, colour, (x, 562), anchor)
        dme = getattr(reading, "dme_nm", None)
        if dme is not None:
            t.blit(s, f"{dme:5.1f} NM", 16, GREEN, (x, 584), anchor)
