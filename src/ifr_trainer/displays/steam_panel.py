"""Round-dial panel shared by the light aircraft and business jet: the six-pack, NAV1 (VOR/LOC
with glideslope), NAV2, ADF and an engine gauge, above a lower panel with throttle, flaps,
trim, warning lights and the radio stack. Each aircraft subclasses ``SteamPanel`` for its
engine gauge, airspeed limits and extra lights.

Mouse wheel (or click left/right half) on NAV1 / NAV2 turns its OBS, on the heading indicator
moves the heading bug, and on a radio field tunes it.
"""

import math

import pygame

from ifr_trainer.displays.common import AMBER, BLACK, CYAN, GREEN, GREY, RED, WHITE, Text
from ifr_trainer.displays.fields import Field, change_minimum, course_field, freq_field
from ifr_trainer.nav.navaids import Ils, Vor
from ifr_trainer.nav.receivers import NavReadings
from ifr_trainer.sim.simulation import Simulation
from ifr_trainer.sim.units import wrap_deg

from .gauges import ND_TO_CDI_DOTS, Gauges

R = 100
COLS = (125, 355, 585, 815, 1045)
ROWS = (120, 350)
LOWER = pygame.Rect(10, 490, 1220, 280)
PANEL_BG = (24, 26, 30)
FIELD_BG = (35, 35, 45)
FIELD_HOVER = (55, 55, 70)
MINIMUM_LIGHT_S = 4.0
STALL_WARNING_MARGIN_DEG = 2.0  # the stall horn sounds a few knots above the stall


class SteamPanel:
    size = (1240, 780)
    vsi_full_scale_fpm = 2000
    message_y = 466
    help = [
        "[ / ]       pitch trim nose down / up (hold)",
        "Wheel over NAV1 / NAV2: OBS; over the heading indicator: heading bug",
    ]
    key_actions: dict[int, str] = {}

    def __init__(self, text: Text, speed_marks: dict):
        self.text = text
        self.marks = speed_marks
        self.g = Gauges(text, R)
        self.knobs = {  # gauge centre -> the cockpit setting the wheel turns there
            (COLS[3], ROWS[0]): course_field("vor1_crs"),
            (COLS[3], ROWS[1]): course_field("vor2_crs"),
            (COLS[1], ROWS[1]): course_field("hdg_bug"),
        }
        self.fields = self._layout()

    def _layout(self) -> list[Field]:
        x_freq, x_obs, y0, h = 700, 860, LOWER.y + 30, 24
        f = [
            Field("NAV1", pygame.Rect(x_freq, y0, 90, h), lambda c: f"{c.vor1_mhz:6.2f}", freq_field("vor1_mhz", 108.0, 117.95, 0.05, 1.0)),
            Field("OBS", pygame.Rect(x_obs, y0, 60, h), lambda c: f"{c.vor1_crs % 360:03.0f}", course_field("vor1_crs")),
            Field("NAV2", pygame.Rect(x_freq, y0 + 36, 90, h), lambda c: f"{c.vor2_mhz:6.2f}", freq_field("vor2_mhz", 108.0, 117.95, 0.05, 1.0)),
            Field("OBS", pygame.Rect(x_obs, y0 + 36, 60, h), lambda c: f"{c.vor2_crs % 360:03.0f}", course_field("vor2_crs")),
            Field("ADF", pygame.Rect(x_freq, y0 + 72, 90, h), lambda c: f"{c.adf1_khz:4.0f}", freq_field("adf1_khz", 190.0, 1750.0, 1.0, 10.0)),
            Field("HDG", pygame.Rect(x_freq, y0 + 108, 90, h), lambda c: f"{c.hdg_bug % 360:03.0f}", course_field("hdg_bug")),
            Field("MIN", pygame.Rect(x_freq, y0 + 144, 90, h), lambda c: "----" if c.minimum_ft is None else f"{c.minimum_ft:.0f}", change_minimum),
        ]
        return f

    # --- Input -------------------------------------------------------------------------

    def _target(self, pos):
        for f in self.fields:
            if f.rect.collidepoint(pos):
                return f.change, f.rect.centerx
        for centre, change in self.knobs.items():
            if math.dist(pos, centre) < R:
                return change, centre[0]
        return None, 0

    def click(self, sim: Simulation, pos: tuple[int, int], coarse: bool) -> bool:
        change, mid = self._target(pos)
        if change is None:
            return False
        change(sim.cockpit, 1 if pos[0] >= mid else -1, coarse)
        return True

    def wheel(self, sim: Simulation, pos: tuple[int, int], steps: int, coarse: bool) -> bool:
        change, _ = self._target(pos)
        if change is None:
            return False
        for _ in range(abs(steps)):
            change(sim.cockpit, 1 if steps > 0 else -1, coarse)
        return True

    def action(self, sim: Simulation, action: str) -> None:
        pass

    # --- Drawing -----------------------------------------------------------------------

    def draw(self, screen: pygame.Surface, sim: Simulation, throttle_needs_sync: bool) -> None:
        screen.fill(PANEL_BG)
        nav = sim.nav_readings()
        st, c, g = sim.state, sim.cockpit, self.g
        hdg = sim.heading_deg
        g.airspeed(screen, (COLS[0], ROWS[0]), sim.cas_kt, self.marks, self.vmax_kt(sim))
        g.attitude(screen, (COLS[1], ROWS[0]), math.degrees(st.theta), st.phi)
        g.altimeter(screen, (COLS[2], ROWS[0]), sim.alt_ft)
        self._nav1(screen, nav, c)
        self.engine_gauge(screen, (COLS[4], ROWS[0]), sim)
        turn = (st.q * math.sin(st.phi) + st.r * math.cos(st.phi)) / math.cos(st.theta)
        g.turn_coordinator(screen, (COLS[0], ROWS[1]), math.degrees(turn + 0.25 * st.p), st.ny)
        g.heading_indicator(screen, (COLS[1], ROWS[1]), hdg, c.hdg_bug)
        g.vertical_speed(screen, (COLS[2], ROWS[1]), sim.vs_fpm, self.vsi_full_scale_fpm)
        v2 = nav.vor2
        g.cdi(screen, (COLS[3], ROWS[1]), c.vor2_crs, "NAV 2",
              v2.deviation_dots * ND_TO_CDI_DOTS if v2 and v2.deviation_dots is not None else None,
              v2.to_from if v2 else None, None, has_gs=False, loc_mode=False)
        adf = nav.adf1
        g.adf(screen, (COLS[4], ROWS[1]), wrap_deg(adf.bearing - hdg) if adf else None, "ADF")
        self._lower(screen, sim, nav, throttle_needs_sync)

    def engine_gauge(self, s: pygame.Surface, centre: tuple[int, int], sim: Simulation) -> None:
        raise NotImplementedError

    def vmax_kt(self, sim: Simulation) -> float | None:
        """Moving limit speed for a barber pole on the airspeed indicator (None = fixed red line)."""
        return None

    def extra_lights(self, sim: Simulation) -> list[tuple[str, tuple[int, int, int], bool]]:
        """Further annunciators below the minimum light: (label, colour when on, on)."""
        return []

    def _nav1(self, s: pygame.Surface, nav: NavReadings, c) -> None:
        centre = (COLS[3], ROWS[0])
        if nav.ils:
            ils = nav.ils
            gs = ils.gs_dots * ND_TO_CDI_DOTS if ils.gs_dots is not None else None
            self.g.cdi(s, centre, c.vor1_crs, "NAV 1", ils.loc_dots * ND_TO_CDI_DOTS, None, gs, has_gs=True, loc_mode=True)
            dme = ils.dme_nm
        else:
            v = nav.vor1
            lateral = v.deviation_dots * ND_TO_CDI_DOTS if v and v.deviation_dots is not None else None
            self.g.cdi(s, centre, c.vor1_crs, "NAV 1", lateral, v.to_from if v else None, None, has_gs=True, loc_mode=False)
            dme = v.dme_nm if v else None
        box = pygame.Rect(0, 0, 92, 20)
        box.center = (centre[0], centre[1] + R + 12)
        pygame.draw.rect(s, BLACK, box)
        self.text.blit(s, "DME ---.-" if dme is None else f"DME {dme:5.1f}", 13, AMBER if dme is not None else GREY, box.center)

    def _lower(self, s: pygame.Surface, sim: Simulation, nav: NavReadings, throttle_needs_sync: bool) -> None:
        t = self.text
        pygame.draw.rect(s, (18, 19, 22), LOWER, border_radius=6)
        x0, y0 = LOWER.x, LOWER.y
        # Throttle: knob travels from idle (bottom) to full (top).
        slot = pygame.Rect(x0 + 50, y0 + 40, 8, 190)
        pygame.draw.rect(s, GREY, slot, border_radius=4)
        knob_y = slot.bottom - sim.inputs.lever * slot.h
        pygame.draw.circle(s, BLACK, (slot.centerx, round(knob_y)), 14)
        pygame.draw.circle(s, WHITE, (slot.centerx, round(knob_y)), 14, 2)
        t.blit(s, "THROTTLE", 13, WHITE, (slot.centerx, y0 + 18))
        t.blit(s, "FULL", 11, GREY, (slot.centerx + 34, slot.top))
        t.blit(s, "IDLE", 11, GREY, (slot.centerx + 34, slot.bottom))
        if throttle_needs_sync:
            t.blit(s, "MOVE THR", 12, AMBER, (slot.centerx, y0 + 262))
        # Flaps: lever position (cyan) and actual position (green).
        fx, fy = x0 + 160, y0 + 40
        t.blit(s, "FLAPS", 13, WHITE, (fx + 40, y0 + 18))
        names = [f.name for f in sim.airframe.flaps]
        step = 190 / (len(names) - 1)
        pygame.draw.line(s, GREY, (fx, fy), (fx, fy + 190), 2)
        for i, name in enumerate(names):
            y = fy + i * step
            pygame.draw.line(s, WHITE, (fx - 6, y), (fx + 6, y), 2)
            t.blit(s, f"{name}°", 13, WHITE, (fx + 30, y))
        pygame.draw.rect(s, GREEN, (fx - 4, fy, 8, sim.flap_pos * step))
        ly = fy + sim.inputs.flap_lever * step
        pygame.draw.polygon(s, CYAN, [(fx - 8, ly), (fx - 20, ly - 7), (fx - 20, ly + 7)])
        # Pitch trim: nose down at the top, nose up at the bottom, like the trim wheel indicator.
        lim = sim.airframe.controls
        tx, ty, th = x0 + 300, y0 + 40, 190
        t.blit(s, "TRIM", 13, WHITE, (tx + 20, y0 + 18))
        pygame.draw.rect(s, GREY, (tx - 4, ty, 8, th), border_radius=4)
        frac = (math.degrees(sim.surfaces.pitch_trim) + lim.pitch_trim_down_deg) / (lim.pitch_trim_up_deg + lim.pitch_trim_down_deg)
        py = ty + frac * th
        pygame.draw.polygon(s, WHITE, [(tx + 6, py), (tx + 18, py - 7), (tx + 18, py + 7)])
        t.blit(s, "NOSE DN", 11, GREY, (tx + 26, ty), "midleft")
        t.blit(s, "NOSE UP", 11, GREY, (tx + 26, ty + th), "midleft")
        # Warning lights.
        lx = x0 + 450
        self._light(s, (lx, y0 + 60), "STALL", RED, self._stall_warning(sim))
        self._light(s, (lx, y0 + 110), "MINIMUM", AMBER, self._minimum_light(sim))
        minimum = sim.cockpit.minimum_ft
        t.blit(s, "DA/MDA" if minimum else "", 11, GREY, (lx, y0 + 150))
        t.blit(s, f"{minimum:.0f} FT" if minimum else "", 14, CYAN, (lx, y0 + 168))
        for i, (label, colour, on) in enumerate(self.extra_lights(sim)):
            self._light(s, (lx, y0 + 210 + 44 * i), label, colour, on)
        # Radio stack.
        pygame.draw.line(s, GREY, (x0 + 560, y0 + 12), (x0 + 560, LOWER.bottom - 12), 1)
        t.blit(s, "RADIOS", 13, GREY, (x0 + 590, y0 + 14), "midleft")
        hover = pygame.mouse.get_pos()
        for f in self.fields:
            t.blit(s, f.label, 13, WHITE, (f.rect.x - 8, f.rect.centery), "midright")
            pygame.draw.rect(s, FIELD_HOVER if f.rect.collidepoint(hover) else FIELD_BG, f.rect, border_radius=3)
            t.blit(s, f.show(sim.cockpit), 15, CYAN, f.rect.center)
        c = sim.cockpit
        rows = ((nav.ils or nav.vor1, Ils if nav.ils else Vor, c.vor1_mhz, 0), (nav.vor2, Vor, c.vor2_mhz, 1),
                (nav.adf1, None, c.adf1_khz, 2))
        for reading, kind, freq, row in rows:
            y = self.fields[0].rect.centery + row * 36
            if reading:
                t.blit(s, reading.ident, 14, GREEN, (x0 + 950, y), "midleft")
            elif kind is not None and sim.nav.tuned(Vor, freq) is None and sim.nav.tuned(Ils, freq) is None:
                t.blit(s, "no stn", 12, AMBER, (x0 + 950, y), "midleft")
        t.blit(s, "wheel: tune  shift: coarse  click: -/+", 11, GREY, (x0 + 1210, LOWER.bottom - 14), "midright")

    def _light(self, s: pygame.Surface, centre, label: str, colour, on: bool) -> None:
        r = pygame.Rect(0, 0, 96, 30)
        r.center = centre
        pygame.draw.rect(s, colour if on else (40, 40, 46), r, border_radius=4)
        self.text.blit(s, label, 13, BLACK if on else GREY, r.center)

    @staticmethod
    def _stall_warning(sim: Simulation) -> bool:
        return sim.state.alpha > sim.model.aero.alpha_stall(sim.aero()) - math.radians(STALL_WARNING_MARGIN_DEG)

    @staticmethod
    def _minimum_light(sim: Simulation) -> bool:
        reached, minimum = sim.minimum_reached_time, sim.cockpit.minimum_ft
        if reached is None or minimum is None:
            return False
        return sim.time_s - reached < MINIMUM_LIGHT_S or sim.alt_ft < minimum
