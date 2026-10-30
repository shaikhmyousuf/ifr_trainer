"""Primary flight display, A320 style: attitude, speed and altitude tapes, VSI, heading, FMA."""

import math

import pygame

from ..sim.simulation import Simulation
from ..sim.units import wrap_180
from .common import (
    AMBER,
    BLACK,
    CYAN,
    GREEN,
    GREY,
    GROUND,
    RED,
    SKY,
    TAPE,
    TAPE_DARK,
    WHITE,
    YELLOW,
    Text,
    draw_barber,
)

SIZE = (600, 600)
CX, CY = 300, 270
ATT_RECT = pygame.Rect(165, 110, 270, 320)
LADDER_RECT = pygame.Rect(195, 140, 210, 255)
SPD_RECT = pygame.Rect(60, 110, 80, 320)
ALT_RECT = pygame.Rect(460, 110, 70, 320)
VSI_RECT = pygame.Rect(540, 130, 42, 280)
HDG_RECT = pygame.Rect(180, 460, 240, 40)
PX_PER_DEG_PITCH = 8.0
PX_PER_KT = 4.0
PX_PER_FT = 0.29
PX_PER_DEG_HDG = 8.0
BANK_RADIUS = 145


def vsi_fraction(vs_fpm: float) -> float:
    """Non-linear VSI scale: fraction of half-height for a vertical speed."""
    v = abs(vs_fpm)
    if v <= 1000:
        f = 0.45 * v / 1000
    elif v <= 2000:
        f = 0.45 + 0.30 * (v - 1000) / 1000
    else:
        f = 0.75 + 0.25 * min(v - 2000, 4000) / 4000
    return math.copysign(f, vs_fpm)


class PFD:
    def __init__(self, text: Text):
        self.text = text
        self.surface = pygame.Surface(SIZE)

    def draw(self, sim: Simulation) -> pygame.Surface:
        s = self.surface
        s.fill(BLACK)
        st = sim.state
        self._attitude(s, math.degrees(st.theta), st.phi)
        self._speed_tape(s, sim)
        self._altitude_tape(s, sim)
        self._vsi(s, sim.vs_fpm)
        self._heading(s, sim.heading_deg, sim.track_deg)
        self._fma(s, sim)
        agl = sim.agl_ft
        if agl < 2500:
            step = 10 if agl > 50 else (5 if agl > 10 else 1)
            self.text.blit(s, f"{round(agl / step) * step:.0f}", 24, GREEN, (CX, 412))
        return s

    # --- Attitude ----------------------------------------------------------------------

    def _attitude(self, s: pygame.Surface, theta_deg: float, phi: float) -> None:
        sp, cp = math.sin(phi), math.cos(phi)
        dx, dy = cp, -sp  # along the horizon
        ux, uy = -sp, -cp  # towards the sky
        hx, hy = CX + theta_deg * PX_PER_DEG_PITCH * sp, CY + theta_deg * PX_PER_DEG_PITCH * cp
        big = 1000
        a = (hx - dx * big, hy - dy * big)
        b = (hx + dx * big, hy + dy * big)
        s.set_clip(ATT_RECT)
        s.fill(SKY, ATT_RECT)
        pygame.draw.polygon(
            s, GROUND, [a, b, (b[0] - ux * big, b[1] - uy * big), (a[0] - ux * big, a[1] - uy * big)]
        )
        pygame.draw.line(s, WHITE, a, b, 2)

        s.set_clip(LADDER_RECT)
        for k in range(-32, 33):  # every 2.5 degrees
            pitch = k * 2.5
            if k == 0 or abs(pitch - theta_deg) > 20:
                continue
            half = 45 if k % 4 == 0 else (22 if k % 2 == 0 else 10)
            off = (theta_deg - pitch) * PX_PER_DEG_PITCH
            mx, my = CX + off * sp, CY + off * cp
            pygame.draw.line(s, WHITE, (mx - dx * half, my - dy * half), (mx + dx * half, my + dy * half), 2)
            if k % 4 == 0:
                label = pygame.transform.rotate(self.text.render(str(abs(int(pitch))), 14, WHITE), math.degrees(phi))
                for side in (-1, 1):
                    pos = (mx + side * dx * (half + 14), my + side * dy * (half + 14))
                    s.blit(label, label.get_rect(center=(round(pos[0]), round(pos[1]))))

        s.set_clip(ATT_RECT)
        self._bank_scale(s, phi)
        s.set_clip(None)
        self._aircraft_symbol(s)

    def _bank_scale(self, s: pygame.Surface, phi: float) -> None:
        r = BANK_RADIUS

        def pt(t: float, radius: float) -> tuple[float, float]:
            return CX + radius * math.sin(t), CY - radius * math.cos(t)

        # White scale rotates with the horizon; the yellow roll index is fixed to the aircraft.
        pygame.draw.lines(s, WHITE, False, [pt(math.radians(a) - phi, r) for a in range(-45, 46, 3)], 2)
        for a in (-45, -30, -20, -10, 10, 20, 30, 45):
            t = math.radians(a) - phi
            pygame.draw.line(s, WHITE, pt(t, r), pt(t, r + (14 if abs(a) in (30, 45) else 8)), 2)
        for a in (-67, 67):
            t = math.radians(a) - phi
            for dr in (0, 5):
                pygame.draw.line(s, GREEN, pt(t - 0.03, r + dr), pt(t + 0.03, r + dr), 2)
        pygame.draw.polygon(s, WHITE, [pt(-phi, r), pt(-phi - 0.05, r + 12), pt(-phi + 0.05, r + 12)], 2)
        tip = CY - r + 3
        pygame.draw.polygon(s, YELLOW, [(CX, tip), (CX - 8, tip + 12), (CX + 8, tip + 12)])

    def _aircraft_symbol(self, s: pygame.Surface) -> None:
        for side in (-1, 1):
            outer, inner = CX + side * 115, CX + side * 45
            wing = pygame.Rect(min(outer, inner), CY - 4, abs(outer - inner), 8)
            drop = pygame.Rect(inner - 4, CY - 4, 8, 20)
            for rect in (wing, drop):
                pygame.draw.rect(s, BLACK, rect)
            pygame.draw.rect(s, YELLOW, wing, 2)
            pygame.draw.rect(s, YELLOW, drop, 2)
        centre = pygame.Rect(CX - 5, CY - 5, 10, 10)
        pygame.draw.rect(s, BLACK, centre)
        pygame.draw.rect(s, YELLOW, centre, 2)

    # --- Speed -------------------------------------------------------------------------

    def _speed_tape(self, s: pygame.Surface, sim: Simulation) -> None:
        r = SPD_RECT
        cas = sim.cas_kt
        lim = sim.speed_limits()

        def y_of(v: float) -> float:
            return CY - (v - cas) * PX_PER_KT

        s.fill(TAPE, r)
        s.set_clip(r)
        for v in range(int(max(30, cas - 45)) // 10 * 10, int(cas + 50), 10):
            if v < 30:
                continue
            y = y_of(v)
            pygame.draw.line(s, WHITE, (r.right - 12, y), (r.right, y), 2)
            if v % 20 == 0:
                self.text.blit(s, f"{v:3d}", 18, WHITE, (r.right - 16, y), "midright")

        x = r.right - 8
        y_vmax = y_of(lim.vmax)
        if y_vmax > r.top:
            draw_barber(s, x, r.top, y_vmax, 8, RED, BLACK)
        y_vls, y_prot, y_amax = y_of(lim.vls), y_of(lim.v_alpha_prot), y_of(lim.v_alpha_max)
        if y_vls < r.bottom:
            pygame.draw.rect(s, AMBER, (x + 2, y_vls, 3, max(0, y_prot - y_vls)))
            draw_barber(s, x, y_prot, y_amax, 8, AMBER, BLACK)
            pygame.draw.rect(s, RED, (x, y_amax, 8, max(0, r.bottom - y_amax)))
        if lim.green_dot is not None:
            pygame.draw.circle(s, GREEN, (r.right - 4, round(y_of(lim.green_dot))), 5, 2)
        s.set_clip(None)

        pygame.draw.line(s, YELLOW, (r.x - 6, CY), (r.right + 20, CY), 3)
        pygame.draw.polygon(s, YELLOW, [(r.right + 4, CY), (r.right + 16, CY - 7), (r.right + 16, CY + 7)])
        trend = sim.speed_trend_kt
        if abs(trend) > 2.0:
            tx = r.right - 26
            y2 = min(max(y_of(cas + trend), r.top), r.bottom)
            pygame.draw.line(s, YELLOW, (tx, CY), (tx, y2), 2)
            d = -6 if y2 < CY else 6
            pygame.draw.polygon(s, YELLOW, [(tx, y2), (tx - 5, y2 - d), (tx + 5, y2 - d)])

    # --- Altitude ----------------------------------------------------------------------

    def _altitude_tape(self, s: pygame.Surface, sim: Simulation) -> None:
        r = ALT_RECT
        alt = sim.alt_ft

        def y_of(a: float) -> float:
            return CY - (a - alt) * PX_PER_FT

        s.fill(TAPE, r)
        s.set_clip(r)
        for a in range(int((alt - 700) // 100 * 100), int(alt + 700), 100):
            y = y_of(a)
            pygame.draw.line(s, WHITE, (r.x, y), (r.x + (12 if a % 500 == 0 else 7), y), 2)
            if a % 500 == 0:
                self.text.blit(s, f"{a // 100:d}", 18, WHITE, (r.x + 16, y), "midleft")
        y_ground = y_of(sim.airport.elevation_ft)
        if y_ground < r.bottom:
            pygame.draw.rect(s, RED, (r.right - 8, y_ground, 8, r.bottom - y_ground))
        s.set_clip(None)

        rounded = int(round(alt / 20.0)) * 20
        box = pygame.Rect(r.x - 6, CY - 17, r.w + 14, 34)
        pygame.draw.rect(s, BLACK, box)
        pygame.draw.rect(s, YELLOW, box, 2)
        self.text.blit(s, f"{rounded // 100:3d}", 22, GREEN, (box.x + 52, CY), "midright")
        self.text.blit(s, f"{rounded % 100:02d}", 16, GREEN, (box.x + 54, CY), "midleft")

    def _vsi(self, s: pygame.Surface, vs_fpm: float) -> None:
        r = VSI_RECT
        half = r.h / 2
        pygame.draw.rect(s, TAPE_DARK, r)
        for v in (-6000, -2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000, 6000):
            y = CY - vsi_fraction(v) * half
            major = abs(v) in (0, 1000, 2000, 6000)
            pygame.draw.line(s, YELLOW if v == 0 else WHITE, (r.x, y), (r.x + (10 if major else 6), y), 2)
            if major and v:
                self.text.blit(s, str(abs(v) // 1000), 13, WHITE, (r.x + 13, y), "midleft")
        colour = AMBER if abs(vs_fpm) > 6000 else GREEN
        tip = (r.x + 4, CY - vsi_fraction(vs_fpm) * half)
        s.set_clip(r)
        pygame.draw.line(s, colour, (r.right + 70, CY), tip, 3)
        s.set_clip(None)
        if abs(vs_fpm) >= 200:
            y = tip[1] + (-14 if vs_fpm > 0 else 14)
            self.text.blit(s, f"{int(abs(vs_fpm)) // 100:02d}", 14, colour, (r.centerx + 6, y))

    # --- Heading and FMA ---------------------------------------------------------------

    def _heading(self, s: pygame.Surface, hdg: float, track: float) -> None:
        r = HDG_RECT
        s.fill(TAPE, r)
        s.set_clip(r)
        for d in range(int(hdg) - 20, int(hdg) + 21):
            if d % 5:
                continue
            x = CX + (d - hdg) * PX_PER_DEG_HDG
            pygame.draw.line(s, WHITE, (x, r.top), (x, r.top + (10 if d % 10 == 0 else 6)), 2)
            if d % 10 == 0:
                self.text.blit(s, str((d % 360) // 10 or 36), 16, WHITE, (x, r.top + 24))
        tx = CX + wrap_180(track - hdg) * PX_PER_DEG_HDG
        pygame.draw.polygon(s, GREEN, [(tx, r.top + 2), (tx + 6, r.top + 9), (tx, r.top + 16), (tx - 6, r.top + 9)], 2)
        s.set_clip(None)
        pygame.draw.line(s, YELLOW, (CX, r.top - 10), (CX, r.top + 14), 3)

    def _fma(self, s: pygame.Surface, sim: Simulation) -> None:
        for x in (120, 240, 360, 480):
            pygame.draw.line(s, GREY, (x, 8), (x, 62), 2)
        lever = sim.inputs.lever
        detents = sim.aircraft.engines.detents
        mode = next((name for name in ("TOGA", "MCT") if abs(lever - detents[name]) < 1e-3), None)
        if mode:
            box = pygame.Rect(14, 10, 92, 46)
            pygame.draw.rect(s, WHITE, box, 2)
            self.text.blit(s, "MAN", 16, WHITE, (box.centerx, box.y + 13))
            self.text.blit(s, mode, 16, WHITE, (box.centerx, box.y + 33))
        flaps = sim.aircraft.flap_config_at(sim.flap_pos)
        if sim.flap_pos > 0.01:
            self.text.blit(s, f"CONF {flaps.name}", 14, CYAN, (540, 22))
