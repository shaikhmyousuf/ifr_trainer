"""Lower panel: engines and configuration, plus clickable nav radios, EFIS and FCU references.

Fields respond to the mouse wheel (Shift = coarse step) and to clicks: left half
decreases, right half increases (or cycles for choices).
"""

import math
import time
import pygame

from ifr_trainer.displays.common import AMBER, CYAN, GREEN, GREY, RED, WHITE, Text
from ifr_trainer.displays.fields import Field, change_altitude, change_minimum, course_field, cycle, freq_field
from ifr_trainer.nav.navaids import Ils, Vor
from ifr_trainer.nav.receivers import NavReadings
from ifr_trainer.sim.simulation import Simulation

from ..cockpit import ND_MODES, ND_RANGES, NEEDLE1, NEEDLE2, A320Cockpit

SIZE = (1220, 150)
BACKGROUND = (18, 18, 24)
FIELD_BG = (35, 35, 45)
FIELD_HOVER = (55, 55, 70)
N1_MAX_SCALE = 110.0


class LowerPanel:
    def __init__(self, text: Text):
        self.text = text
        self.surface = pygame.Surface(SIZE)
        self.fields = self._layout()

    def _layout(self) -> list[Field]:
        f: list[Field] = []
        x_freq, x_crs, w_freq, w_crs, h = 600, 730, 90, 64, 22
        radios = (
            ("VOR1", "vor1_mhz", "vor1_crs", 108.0, 117.95),
            ("VOR2", "vor2_mhz", "vor2_crs", 108.0, 117.95),
            ("ILS", "ils_mhz", "ils_crs", 108.10, 111.95),
        )
        for row, (label, freq, crs, lo, hi) in enumerate(radios):
            y = 28 + row * 26
            f.append(Field(label, pygame.Rect(x_freq, y, w_freq, h), lambda c, a=freq: f"{getattr(c, a):6.2f}",
                           freq_field(freq, lo, hi, 0.05, 1.0)))
            f.append(Field("CRS", pygame.Rect(x_crs, y, w_crs, h), lambda c, a=crs: f"{getattr(c, a) % 360:03.0f}",
                           course_field(crs)))
        y = 28 + 3 * 26
        f.append(Field("ADF1", pygame.Rect(x_freq, y, w_crs, h), lambda c: f"{c.adf1_khz:4.0f}",
                       freq_field("adf1_khz", 190.0, 1750.0, 1.0, 10.0)))
        f.append(Field("ADF2", pygame.Rect(x_crs + 34, y, w_crs, h), lambda c: f"{c.adf2_khz:4.0f}",
                       freq_field("adf2_khz", 190.0, 1750.0, 1.0, 10.0)))

        def efis(label, x, y, w, show, change):
            f.append(Field(label, pygame.Rect(x, y, w, h), show, change))

        xa, xb = 975, 1135
        efis("ND", xa, 26, 100, lambda c: c.nd_mode,
             lambda c, d, big: setattr(c, "nd_mode", cycle(ND_MODES, c.nd_mode, d)))
        efis("RNG", xa, 50, 100, lambda c: f"{c.nd_range} NM",
             lambda c, d, big: setattr(c, "nd_range", ND_RANGES[min(max(ND_RANGES.index(c.nd_range) + d, 0), len(ND_RANGES) - 1)]))
        efis("NDL1", xa, 74, 100, lambda c: c.needle1,
             lambda c, d, big: setattr(c, "needle1", cycle(NEEDLE1, c.needle1, d)))
        efis("NDL2", xa, 98, 100, lambda c: c.needle2,
             lambda c, d, big: setattr(c, "needle2", cycle(NEEDLE2, c.needle2, d)))
        efis("LS", xb, 26, 76, lambda c: "ON" if c.ls else "OFF", lambda c, d, big: setattr(c, "ls", not c.ls))
        efis("FPV", xb, 50, 76, lambda c: "ON" if c.fpv else "OFF", lambda c, d, big: setattr(c, "fpv", not c.fpv))
        efis("HDG", xb, 74, 76, lambda c: f"{c.hdg_bug % 360:03.0f}", course_field("hdg_bug"))
        efis("ALT", xb, 98, 76, lambda c: f"{c.alt_target_ft:.0f}", change_altitude)
        efis("MIN", xb, 122, 76, lambda c: "----" if c.minimum_ft is None else f"{c.minimum_ft:.0f}", change_minimum)
        return f

    # --- Input -------------------------------------------------------------------------

    def field_at(self, pos: tuple[int, int]) -> Field | None:
        return next((f for f in self.fields if f.rect.collidepoint(pos)), None)

    def click(self, cockpit: A320Cockpit, pos: tuple[int, int], coarse: bool) -> bool:
        field = self.field_at(pos)
        if field is None:
            return False
        field.change(cockpit, 1 if pos[0] >= field.rect.centerx else -1, coarse)
        return True

    def wheel(self, cockpit: A320Cockpit, pos: tuple[int, int], steps: int, coarse: bool) -> bool:
        field = self.field_at(pos)
        if field is None:
            return False
        for _ in range(abs(steps)):
            field.change(cockpit, 1 if steps > 0 else -1, coarse)
        return True

    # --- Drawing -----------------------------------------------------------------------

    def draw(self, sim: Simulation, nav: NavReadings, throttle_needs_sync: bool, hover: tuple[int, int] | None) -> pygame.Surface:
        s = self.surface
        s.fill(BACKGROUND)
        for x in (300, 545, 915):
            pygame.draw.line(s, GREY, (x, 8), (x, SIZE[1] - 8), 1)
        self._engines(s, sim, throttle_needs_sync)
        self._config(s, sim)
        self._radio_idents(s, sim, nav)
        self.text.blit(s, "NAV RADIOS", 13, GREY, (560, 12), "midleft")
        self.text.blit(s, "EFIS / FCU", 13, GREY, (930, 12), "midleft")
        self.text.blit(s, "wheel: tune  shift: coarse  click: -/+", 11, GREY, (560, 140), "midleft")
        hovered = self.field_at(hover) if hover else None
        for f in self.fields:
            self.text.blit(s, f.label, 13, WHITE, (f.rect.x - 6, f.rect.centery), "midright")
            pygame.draw.rect(s, FIELD_HOVER if f is hovered else FIELD_BG, f.rect, border_radius=3)
            self.text.blit(s, f.show(sim.cockpit), 15, CYAN, f.rect.center)
        return s

    def _radio_idents(self, s: pygame.Surface, sim: Simulation, nav: NavReadings) -> None:
        """Station ident (green) when the tuned station is received, else nothing."""
        c = sim.cockpit
        for reading, y in ((nav.vor1, 28), (nav.vor2, 54), (nav.ils, 80)):
            if reading:
                self.text.blit(s, reading.ident, 14, GREEN, (802, y + 11), "midleft")
        for reading, x in ((nav.adf1, 670), (nav.adf2, 834)):
            if reading:
                self.text.blit(s, reading.ident, 14, GREEN, (x, 28 + 3 * 26 + 11), "midleft")
        # Tuned frequency with no station at all: amber hint.
        for kind, freq, y in ((Vor, c.vor1_mhz, 28), (Vor, c.vor2_mhz, 54), (Ils, c.ils_mhz, 80)):
            if sim.nav.tuned(kind, freq) is None:
                self.text.blit(s, "no stn", 12, AMBER, (802, y + 11), "midleft")

    def _engines(self, s: pygame.Surface, sim: Simulation, throttle_needs_sync: bool) -> None:
        n1, n1_cmd = sim.propulsion.n1, sim.propulsion.lever_to_n1(sim.inputs.lever)
        for cx in (55, 160):
            self._n1_gauge(s, (cx, 62), n1, n1_cmd, sim.propulsion.data.n1_max)
        self.text.blit(s, "N1 %", 13, CYAN, (107, 128))
        # Thrust lever: vertical, with detents.
        top, bottom, x = 18, 128, 270
        pygame.draw.line(s, GREY, (x, top), (x, bottom), 3)
        for name, pos in sim.propulsion.detents.items():
            y = bottom - pos * (bottom - top)
            pygame.draw.line(s, WHITE, (x - 8, y), (x + 8, y), 2)
            self.text.blit(s, name, 11, WHITE, (x - 12, y), "midright")
        y = bottom - sim.inputs.lever * (bottom - top)
        pygame.draw.rect(s, CYAN, (x - 12, y - 4, 24, 8), border_radius=3)
        if throttle_needs_sync:
            self.text.blit(s, "MOVE THR", 11, AMBER, (x - 30, 142))

    def _n1_gauge(self, s: pygame.Surface, c: tuple[int, int], n1: float, n1_cmd: float, n1_max: float) -> None:
        r = 42
        cx, cy = c

        def pt(n: float, radius: float) -> tuple[float, float]:
            a = math.radians(210.0 - 240.0 * min(max(n, 0.0), N1_MAX_SCALE) / N1_MAX_SCALE)
            return cx + radius * math.cos(a), cy - radius * math.sin(a)

        pygame.draw.lines(s, WHITE, False, [pt(n, r) for n in range(0, 105, 4)], 2)
        pygame.draw.lines(s, RED, False, [pt(n, r) for n in range(104, 111, 2)], 3)
        for n in range(20, 101, 20):
            pygame.draw.line(s, WHITE, pt(n, r), pt(n, r - 6), 2)
        pygame.draw.line(s, AMBER, pt(n1_max, r + 1), pt(n1_max, r + 8), 3)
        pygame.draw.circle(s, CYAN, [round(v) for v in pt(n1_cmd, r + 6)], 4, 2)
        pygame.draw.line(s, GREEN, (cx, cy), pt(n1, r - 3), 3)
        self.text.blit(s, f"{n1:5.1f}", 16, GREEN, (cx + 8, cy + 26))

    def _config(self, s: pygame.Surface, sim: Simulation) -> None:
        t = self.text
        names = [f.name for f in sim.airframe.flaps]
        n = len(names) - 1
        x0, y0, width = 320, 30, 120
        t.blit(s, "FLAP", 13, WHITE, (x0 + width / 2, 12))
        for i, name in enumerate(names):
            x = x0 + width * i / n
            pygame.draw.line(s, WHITE, (x, y0 + 8), (x, y0 + 14), 2)
            t.blit(s, name if name != "FULL" else "F", 12, WHITE, (x, y0 + 24))
        tx = x0 + width * sim.inputs.flap_lever / n
        pygame.draw.polygon(s, CYAN, [(tx, y0 + 6), (tx - 5, y0 - 2), (tx + 5, y0 - 2)])
        pygame.draw.line(s, GREEN, (x0, y0 + 11), (x0 + width * sim.flap_pos / n, y0 + 11), 4)
        moving = abs(sim.flap_pos - sim.inputs.flap_lever) > 0.01
        t.blit(s, f"CONF {names[sim.inputs.flap_lever]}", 14, CYAN if moving else GREEN, (x0 + width / 2, y0 + 44))

        t.blit(s, "GEAR", 13, WHITE, (500, 12))
        g = sim.gear_pos
        colour = GREEN if g >= 1.0 else (RED if g > 0.0 else None)
        for cx, cy in ((500, 34), (478, 58), (522, 58)):
            pts = [(cx - 9, cy - 7), (cx + 9, cy - 7), (cx, cy + 8)]
            pygame.draw.polygon(s, colour or GREY, pts, 0 if g >= 1.0 else (2 if colour else 1))

        sb = sim.surfaces.speedbrake
        label = {0.0: "RET", 0.5: "1/2", 1.0: "FULL"}.get(round(sb * 2) / 2, "MOV")
        t.blit(s, "SPD BRK", 12, WHITE, (500, 84))
        t.blit(s, label, 14, GREEN if sb == 0 else AMBER, (500, 100))
        ths = math.degrees(sim.surfaces.pitch_trim)
        t.blit(s, f"PITCH TRIM {abs(ths):3.1f} {'UP' if ths >= 0 else 'DN'}", 12, CYAN, (380, 100))

        warnings = []
        lim = sim.aircraft.speed_limits(sim)
        if sim.state.lift_limited and sim.cas_kt < lim.v_alpha_prot:
            warnings.append(("STALL", RED))
        if sim.cas_kt > lim.vmax + 4:
            warnings.append(("OVERSPEED", RED))
        if sim.agl_ft < 750 and sim.gear_pos < 1.0 and sim.vs_fpm < 0:
            warnings.append(("GEAR NOT DOWN", AMBER))
        x = 310
        for msg, colour in warnings:
            if colour != RED or int(time.monotonic() * 3) % 2 == 0:
                rect = t.blit(s, msg, 16, colour, (x, 130), "midleft")
                x = rect.right + 12
