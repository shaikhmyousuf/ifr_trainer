"""Round "steam gauge" instruments for the Cessna 172 panel.

Every gauge is drawn around a centre (cx, cy) with radius R. Dial angles are measured
clockwise from 12 o'clock, in degrees.
"""

import math

import pygame

from ifr_trainer.displays.common import AMBER, BLACK, GREEN, GREY, GROUND, RED, SKY, WHITE, YELLOW, Text

BEZEL = (52, 52, 58)
FACE = (16, 16, 19)
ORANGE = (255, 140, 0)
FLAG_RED = (210, 30, 30)
CDI_DOTS = 5  # dots each side on a VOR/LOC indicator
# The receivers report deviation in Airbus ND dots (VOR 5 deg/dot, LOC and GS 2 dots = full scale).
# A Cessna CDI has 5 dots each side with full scale 10 deg VOR, and full-scale LOC and GS
# the same as 2 ND dots, so every deviation scales by 2.5.
ND_TO_CDI_DOTS = 2.5
CARDINALS = {0: "N", 90: "E", 180: "S", 270: "W"}


def polar(c: tuple[float, float], r: float, deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    return c[0] + r * math.sin(a), c[1] - r * math.cos(a)


def arc(s: pygame.Surface, colour, c, r: float, a0: float, a1: float, width: int) -> None:
    n = max(2, int(abs(a1 - a0) / 2) + 1)
    pts = [polar(c, r, a0 + (a1 - a0) * i / (n - 1)) for i in range(n)]
    pygame.draw.lines(s, colour, False, pts, width)


def needle(s: pygame.Surface, c, length: float, deg: float, colour=WHITE, width: int = 4, tail: float = 14) -> None:
    pygame.draw.line(s, colour, polar(c, -tail, deg), polar(c, length, deg), width)
    pygame.draw.circle(s, GREY, (round(c[0]), round(c[1])), 6)


class Gauges:
    def __init__(self, text: Text, radius: int):
        self.text = text
        self.R = radius
        # Alpha mask that keeps only the inside of the dial (for the attitude indicator).
        size = 2 * radius
        self.mask = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.circle(self.mask, (255, 255, 255, 255), (radius, radius), radius - 8)

    def face(self, s: pygame.Surface, c, label: str | None = None) -> None:
        pygame.draw.circle(s, BEZEL, c, self.R)
        pygame.draw.circle(s, FACE, c, self.R - 8)
        if label:
            self.text.blit(s, label, 11, GREY, (c[0], c[1] + self.R * 0.42))

    def ticks(self, s, c, values, angle, r_out, length, width=2, colour=WHITE) -> None:
        for v in values:
            a = angle(v)
            pygame.draw.line(s, colour, polar(c, r_out, a), polar(c, r_out - length, a), width)

    # --- Airspeed indicator --------------------------------------------------------------

    @staticmethod
    def asi_angle(kt: float) -> float:
        return 25.0 + (min(max(kt, 30.0), 170.0) - 30.0) / 140.0 * 310.0

    def airspeed(self, s, c, kt: float, marks: dict) -> None:
        R, ang = self.R, self.asi_angle
        self.face(s, c)
        self.text.blit(s, "AIRSPEED", 11, GREY, (c[0], c[1] - R * 0.34))
        self.text.blit(s, "KNOTS", 11, GREY, (c[0], c[1] - R * 0.20))
        w0, w1 = marks["white_arc"]
        g0, g1 = marks["green_arc"]
        y0, y1 = marks["yellow_arc"]
        arc(s, WHITE, c, R - 26, ang(w0), ang(w1), 6)
        arc(s, GREEN, c, R - 17, ang(g0), ang(g1), 8)
        arc(s, YELLOW, c, R - 17, ang(y0), ang(y1), 8)
        pygame.draw.line(s, RED, polar(c, R - 10, ang(marks["red_line"])), polar(c, R - 30, ang(marks["red_line"])), 4)
        self.ticks(s, c, range(40, 161, 10), ang, R - 10, 12)
        self.ticks(s, c, range(45, 160, 10), ang, R - 10, 6, 1)
        for v in range(40, 161, 20):
            self.text.blit(s, str(v), 14, WHITE, polar(c, R - 42, ang(v)))
        needle(s, c, R - 14, ang(kt))

    # --- Attitude indicator --------------------------------------------------------------

    def attitude(self, s, c, theta_deg: float, phi: float) -> None:
        R = self.R
        self.face(s, c)
        sub = pygame.Surface((2 * R, 2 * R), pygame.SRCALPHA)
        lc = (R, R)
        sp, cp = math.sin(phi), math.cos(phi)
        dx, dy, ux, uy = cp, -sp, -sp, -cp
        px = 3.0  # pixels per degree of pitch
        hx, hy = R + theta_deg * px * sp, R + theta_deg * px * cp
        big = 400
        sub.fill(SKY)
        a, b = (hx - dx * big, hy - dy * big), (hx + dx * big, hy + dy * big)
        pygame.draw.polygon(sub, GROUND, [a, b, (b[0] - ux * big, b[1] - uy * big), (a[0] - ux * big, a[1] - uy * big)])
        pygame.draw.line(sub, WHITE, a, b, 2)
        for pitch in (-20, -10, -5, 5, 10, 20):
            half = 26 if abs(pitch) >= 10 else 13
            off = (theta_deg - pitch) * px
            mx, my = R + off * sp, R + off * cp
            pygame.draw.line(sub, WHITE, (mx - dx * half, my - dy * half), (mx + dx * half, my + dy * half), 2)
        # Bank scale: fixed to the case; the pointer turns with the horizon.
        for a in (-60, -30, -20, -10, 0, 10, 20, 30, 60):
            r0 = R - 10
            pygame.draw.line(sub, WHITE, polar(lc, r0, a), polar(lc, r0 - (16 if a in (0, -30, 30, -60, 60) else 9), a), 3)
        tip = polar(lc, R - 28, -math.degrees(phi))
        left, right = polar(lc, R - 40, -math.degrees(phi) - 5), polar(lc, R - 40, -math.degrees(phi) + 5)
        pygame.draw.polygon(sub, ORANGE, [tip, left, right])
        sub.blit(self.mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
        s.blit(sub, (c[0] - R, c[1] - R))
        # Miniature aircraft (fixed).
        cx, cy = c
        pygame.draw.line(s, ORANGE, (cx - 48, cy), (cx - 16, cy), 4)
        pygame.draw.line(s, ORANGE, (cx + 16, cy), (cx + 48, cy), 4)
        pygame.draw.lines(s, ORANGE, False, [(cx - 16, cy), (cx, cy + 9), (cx + 16, cy)], 4)
        pygame.draw.circle(s, ORANGE, (cx, cy), 3)

    # --- Altimeter -----------------------------------------------------------------------

    def altimeter(self, s, c, alt_ft: float) -> None:
        R = self.R
        self.face(s, c)
        self.text.blit(s, "ALT", 11, GREY, (c[0], c[1] - R * 0.30))
        self.ticks(s, c, range(0, 1000, 100), lambda v: v * 0.36, R - 10, 12)
        self.ticks(s, c, range(20, 1000, 20), lambda v: v * 0.36, R - 10, 5, 1)
        for i in range(10):
            self.text.blit(s, str(i), 16, WHITE, polar(c, R - 32, i * 36))
        window = pygame.Rect(0, 0, 50, 18)
        window.center = (c[0], c[1] + R * 0.34)
        pygame.draw.rect(s, BLACK, window)
        self.text.blit(s, "29.92", 11, WHITE, window.center)
        # 10,000 ft pointer (thin, with a triangle), 1,000 ft hand (short, wide), 100 ft hand (long).
        a10k = (alt_ft % 100000) / 100000 * 360
        pygame.draw.line(s, WHITE, c, polar(c, R - 14, a10k), 1)
        pygame.draw.polygon(s, WHITE, [polar(c, R - 12, a10k), polar(c, R - 24, a10k - 4), polar(c, R - 24, a10k + 4)])
        needle(s, c, R * 0.48, (alt_ft % 10000) / 10000 * 360, WHITE, 7, 8)
        needle(s, c, R - 18, (alt_ft % 1000) / 1000 * 360, WHITE, 3, 14)

    # --- Turn coordinator ----------------------------------------------------------------

    def turn_coordinator(self, s, c, turn_rate_deg_s: float, ny: float) -> None:
        R = self.R
        self.face(s, c)
        self.text.blit(s, "TURN COORDINATOR", 10, GREY, (c[0], c[1] - R * 0.55))
        self.text.blit(s, "2 MIN", 11, WHITE, (c[0], c[1] + R * 0.62))
        # Standard rate (3 deg/s) marks at 20 degrees either side of level.
        for a in (-90 - 20, -90, 90, 90 + 20):
            pygame.draw.line(s, WHITE, polar(c, R - 12, a), polar(c, R - 26, a), 4)
        self.text.blit(s, "L", 14, WHITE, polar(c, R - 38, -90 - 20))
        self.text.blit(s, "R", 14, WHITE, polar(c, R - 38, 90 + 20))
        rot = max(-35.0, min(35.0, turn_rate_deg_s / 3.0 * 20.0))
        span = R * 0.62

        def p(x, y):  # aircraft symbol, rotated about the centre
            a = math.radians(rot)
            return c[0] + x * math.cos(a) - y * math.sin(a), c[1] + x * math.sin(a) + y * math.cos(a)

        pygame.draw.line(s, WHITE, p(-span, 0), p(span, 0), 5)
        pygame.draw.circle(s, WHITE, (round(c[0]), round(c[1])), 8)
        pygame.draw.line(s, WHITE, p(0, -8), p(0, -20), 4)
        # Inclinometer: the ball moves opposite to lateral specific force.
        tube = pygame.Rect(0, 0, R * 0.9, 16)
        tube.center = (c[0], c[1] + R * 0.36)
        pygame.draw.rect(s, (200, 200, 190), tube, border_radius=8)
        for side in (-1, 1):
            x = tube.centerx + side * 11
            pygame.draw.line(s, BLACK, (x, tube.top), (x, tube.bottom), 2)
        bx = tube.centerx + max(-tube.w / 2 + 8, min(tube.w / 2 - 8, -ny * 300))
        pygame.draw.circle(s, BLACK, (round(bx), tube.centery), 7)

    # --- Heading indicator ---------------------------------------------------------------

    def heading_indicator(self, s, c, heading: float, bug: float) -> None:
        R = self.R
        self.face(s, c)
        card = lambda h: h - heading  # noqa: E731 - angle of a heading on the rotating card
        self.ticks(s, c, range(0, 360, 10), card, R - 10, 12)
        self.ticks(s, c, range(5, 360, 10), card, R - 10, 6, 1)
        for h in range(0, 360, 30):
            self.text.blit(s, CARDINALS.get(h, str(h // 10)), 16, WHITE, polar(c, R - 34, card(h)))
        b = card(bug)
        pygame.draw.polygon(s, ORANGE, [polar(c, R - 10, b - 5), polar(c, R - 10, b + 5), polar(c, R - 22, b + 5),
                                        polar(c, R - 16, b), polar(c, R - 22, b - 5)])
        pygame.draw.polygon(s, ORANGE, [polar(c, R - 4, 0), polar(c, R - 16, -5), polar(c, R - 16, 5)])
        cx, cy = c
        pygame.draw.line(s, ORANGE, (cx, cy - 26), (cx, cy + 24), 4)
        pygame.draw.line(s, ORANGE, (cx - 24, cy - 4), (cx + 24, cy - 4), 4)
        pygame.draw.line(s, ORANGE, (cx - 10, cy + 20), (cx + 10, cy + 20), 3)

    # --- Vertical speed ------------------------------------------------------------------

    @staticmethod
    def vsi_angle(fpm: float) -> float:
        return 270.0 + max(-2000.0, min(2000.0, fpm)) / 2000.0 * 160.0

    def vertical_speed(self, s, c, fpm: float) -> None:
        R = self.R
        self.face(s, c)
        self.text.blit(s, "VERTICAL SPEED", 10, GREY, (c[0] + 8, c[1] - R * 0.28))
        self.text.blit(s, "100 FT PER MIN", 10, GREY, (c[0] + 8, c[1] + R * 0.28))
        self.text.blit(s, "UP", 11, WHITE, (c[0] - R * 0.25, c[1] - R * 0.52))
        self.text.blit(s, "DN", 11, WHITE, (c[0] - R * 0.25, c[1] + R * 0.52))
        self.ticks(s, c, range(-2000, 2001, 500), self.vsi_angle, R - 10, 13, 3)
        self.ticks(s, c, range(-1900, 2000, 100), self.vsi_angle, R - 10, 6, 1)
        for v in (0, 5, 10, 15, 20):
            for sign in ((1, -1) if v else (1,)):
                self.text.blit(s, str(v), 15, WHITE, polar(c, R - 32, self.vsi_angle(sign * v * 100)))
        needle(s, c, R - 14, self.vsi_angle(fpm))

    # --- VOR / LOC indicator -------------------------------------------------------------

    def cdi(self, s, c, obs: float, label: str, lateral: float | None, to_from: str | None,
            glideslope: float | None, has_gs: bool, loc_mode: bool) -> None:
        """``lateral`` and ``glideslope`` in CDI dots (None = flag). + lateral = course to the right;
        + glideslope = aircraft above the path (needle below)."""
        R = self.R
        self.face(s, c)
        card = lambda h: h - obs  # noqa: E731
        self.ticks(s, c, range(0, 360, 10), card, R - 10, 9)
        for h in range(0, 360, 30):
            self.text.blit(s, CARDINALS.get(h, str(h // 10)), 14, WHITE, polar(c, R - 28, card(h)))
        pygame.draw.polygon(s, WHITE, [polar(c, R - 38, 0), polar(c, R - 48, -6), polar(c, R - 48, 6)])
        self.text.blit(s, label, 11, GREY, (c[0], c[1] + R * 0.34))
        dot = R * 0.09
        cx, cy = c
        for i in range(1, CDI_DOTS + 1):
            for side in (-1, 1):
                pygame.draw.circle(s, WHITE, (round(cx + side * i * dot), round(cy)), 3, 1)
                if has_gs:
                    pygame.draw.circle(s, WHITE, (round(cx + R * 0.56), round(cy + side * i * dot)), 3, 1)
        pygame.draw.circle(s, WHITE, (round(cx), round(cy)), 7, 1)
        top, bottom = cy - R * 0.52, cy + R * 0.52
        if lateral is None:
            self._flag(s, (cx - R * 0.28, cy - R * 0.24), "NAV")
        else:
            x = cx + max(-5.5, min(5.5, lateral)) * dot
            pygame.draw.line(s, WHITE, (x, top), (x, bottom), 3)
        if not loc_mode and to_from is not None and lateral is not None:
            self.text.blit(s, "TO" if to_from == "TO" else "FR", 13, WHITE, (cx + R * 0.28, cy - R * 0.24))
        if has_gs:
            if glideslope is None:
                self._flag(s, (cx + R * 0.28, cy + R * 0.2), "GS")
            else:
                y = cy + max(-5.5, min(5.5, glideslope)) * dot
                pygame.draw.line(s, WHITE, (cx - R * 0.52, y), (cx + R * 0.52, y), 3)

    def _flag(self, s, pos, label: str) -> None:
        r = pygame.Rect(0, 0, 30, 16)
        r.center = (round(pos[0]), round(pos[1]))
        pygame.draw.rect(s, FLAG_RED, r)
        self.text.blit(s, label, 11, WHITE, r.center)

    # --- ADF -----------------------------------------------------------------------------

    def adf(self, s, c, relative_bearing: float | None, label: str) -> None:
        R = self.R
        self.face(s, c)
        self.ticks(s, c, range(0, 360, 10), lambda v: v, R - 10, 9)
        for h in range(0, 360, 30):
            self.text.blit(s, str(h // 10), 14, WHITE, polar(c, R - 28, h))
        self.text.blit(s, label, 11, GREY, (c[0], c[1] + R * 0.4))
        cx, cy = c
        pygame.draw.line(s, ORANGE, (cx, cy - 18), (cx, cy + 16), 3)
        pygame.draw.line(s, ORANGE, (cx - 16, cy - 2), (cx + 16, cy - 2), 3)
        # No signal: the needle parks at the 3 o'clock position.
        a = 90.0 if relative_bearing is None else relative_bearing
        tip = polar(c, R - 20, a)
        pygame.draw.line(s, YELLOW, polar(c, -(R - 30), a), tip, 4)
        pygame.draw.polygon(s, YELLOW, [tip, polar(c, R - 36, a - 5), polar(c, R - 36, a + 5)])

    # --- Tachometer ----------------------------------------------------------------------

    @staticmethod
    def tach_angle(rpm: float) -> float:
        return -125.0 + min(max(rpm, 0.0), 3500.0) / 3500.0 * 250.0

    def tachometer(self, s, c, rpm: float) -> None:
        R = self.R
        self.face(s, c, "RPM x100")
        ang = self.tach_angle
        arc(s, GREEN, c, R - 17, ang(2100), ang(2700), 8)
        pygame.draw.line(s, RED, polar(c, R - 10, ang(2700)), polar(c, R - 30, ang(2700)), 4)
        self.ticks(s, c, range(0, 3501, 500), ang, R - 10, 12)
        self.ticks(s, c, range(100, 3500, 100), ang, R - 10, 5, 1)
        for v in range(0, 36, 5):
            self.text.blit(s, str(v), 14, WHITE, polar(c, R - 36, ang(v * 100)))
        self.text.blit(s, f"{rpm:4.0f}", 14, AMBER if rpm > 2700 else GREEN, (c[0], c[1] + R * 0.62))
        needle(s, c, R - 14, ang(rpm))
