"""Engine and systems panel: N1, thrust lever, flaps, gear, speedbrake, warnings, info."""

import math
import time

import pygame

from ..sim.aero import Surfaces
from ..sim.simulation import Simulation
from .common import AMBER, BLACK, CYAN, GREEN, GREY, RED, TAPE_DARK, WHITE, Text

SIZE = (600, 600)
N1_MAX_SCALE = 110.0
N1_ARC_START = 210.0  # degrees, maths convention (ccw from +x)
N1_ARC_SWEEP = 240.0


def _n1_angle(n1: float) -> float:
    return math.radians(N1_ARC_START - N1_ARC_SWEEP * min(max(n1, 0.0), N1_MAX_SCALE) / N1_MAX_SCALE)


class EnginePanel:
    def __init__(self, text: Text):
        self.text = text
        self.surface = pygame.Surface(SIZE)

    def draw(self, sim: Simulation, throttle_needs_sync: bool, joystick_name: str) -> pygame.Surface:
        s = self.surface
        s.fill(BLACK)
        n1_cmd = sim.engines.lever_to_n1(sim.inputs.lever)
        for cx in (150, 450):
            self._n1_gauge(s, (cx, 120), sim.engines.n1, n1_cmd, sim.aircraft.engines.n1_max)
        self.text.blit(s, "N1 %", 16, CYAN, (300, 120))
        self._lever(s, sim, throttle_needs_sync)
        self._flaps(s, sim)
        self._gear(s, sim.gear_pos)
        self._speedbrake(s, sim.surfaces)
        self._controls(s, sim)
        self._warnings(s, sim)
        self._info(s, sim, joystick_name)
        return s

    def _n1_gauge(self, s: pygame.Surface, c: tuple[int, int], n1: float, n1_cmd: float, n1_max: float) -> None:
        r = 80
        cx, cy = c

        def pt(n: float, radius: float) -> tuple[float, float]:
            a = _n1_angle(n)
            return cx + radius * math.cos(a), cy - radius * math.sin(a)

        pygame.draw.lines(s, WHITE, False, [pt(n, r) for n in range(0, 105, 3)], 2)
        pygame.draw.lines(s, RED, False, [pt(n, r) for n in range(104, 111, 2)], 4)
        for n in range(20, 101, 10):
            pygame.draw.line(s, WHITE, pt(n, r), pt(n, r - (10 if n % 20 == 0 else 6)), 2)
            if n % 20 == 0:
                self.text.blit(s, str(n // 10), 14, WHITE, pt(n, r - 22))
        pygame.draw.line(s, AMBER, pt(n1_max, r + 2), pt(n1_max, r + 12), 3)
        pygame.draw.circle(s, CYAN, [round(v) for v in pt(n1_cmd, r + 8)], 5, 2)
        pygame.draw.line(s, GREEN, (cx, cy), pt(n1, r - 4), 3)
        box = pygame.Rect(cx - 10, cy + 18, 70, 28)
        pygame.draw.rect(s, GREY, box, 1)
        self.text.blit(s, f"{n1:5.1f}", 20, GREEN, box.center)

    def _lever(self, s: pygame.Surface, sim: Simulation, throttle_needs_sync: bool) -> None:
        top, bottom, x = 260, 520, 70
        pygame.draw.line(s, GREY, (x, top), (x, bottom), 4)
        for name, pos in sim.aircraft.engines.detents.items():
            y = bottom - pos * (bottom - top)
            pygame.draw.line(s, WHITE, (x - 12, y), (x + 12, y), 2)
            self.text.blit(s, name, 14, WHITE, (x + 18, y), "midleft")
        y = bottom - sim.inputs.lever * (bottom - top)
        pygame.draw.rect(s, CYAN, (x - 18, y - 5, 36, 10), border_radius=3)
        self.text.blit(s, "THR LVR", 14, WHITE, (x + 10, top - 22))
        if throttle_needs_sync:
            self.text.blit(s, "MOVE THR", 13, AMBER, (x + 10, bottom + 20))
            self.text.blit(s, "TO SYNC", 13, AMBER, (x + 10, bottom + 36))

    def _flaps(self, s: pygame.Surface, sim: Simulation) -> None:
        x0, y0, width = 200, 270, 180
        names = [f.name for f in sim.aircraft.flaps]
        n = len(names) - 1
        self.text.blit(s, "FLAP", 16, WHITE, (x0 + width / 2, y0 - 10))
        for i, name in enumerate(names):
            x = x0 + width * i / n
            pygame.draw.line(s, WHITE, (x, y0 + 14), (x, y0 + 22), 2)
            self.text.blit(s, name if name != "FULL" else "F", 14, WHITE, (x, y0 + 34))
        target_x = x0 + width * sim.inputs.flap_lever / n
        pygame.draw.polygon(s, CYAN, [(target_x, y0 + 12), (target_x - 6, y0 + 2), (target_x + 6, y0 + 2)])
        actual_x = x0 + width * sim.flap_pos / n
        pygame.draw.line(s, GREEN, (x0, y0 + 18), (actual_x, y0 + 18), 4)
        in_transit = abs(sim.flap_pos - sim.inputs.flap_lever) > 0.01
        label = f"CONF {names[sim.inputs.flap_lever]}" if sim.inputs.flap_lever else "CONF 0"
        self.text.blit(s, label, 18, CYAN if in_transit else GREEN, (x0 + width / 2, y0 + 62))

    def _gear(self, s: pygame.Surface, gear_pos: float) -> None:
        self.text.blit(s, "GEAR", 16, WHITE, (500, 260))
        colour = GREEN if gear_pos >= 1.0 else (RED if gear_pos > 0.0 else None)
        for cx, cy in ((500, 288), (465, 322), (535, 322)):
            pts = [(cx - 14, cy - 10), (cx + 14, cy - 10), (cx, cy + 12)]
            if colour:
                pygame.draw.polygon(s, colour, pts, 0 if gear_pos >= 1.0 else 3)
            else:
                pygame.draw.polygon(s, GREY, pts, 1)

    def _speedbrake(self, s: pygame.Surface, surfaces: Surfaces) -> None:
        self.text.blit(s, "SPD BRK", 14, WHITE, (500, 352))
        if surfaces.ground_spoiler > 0.05:
            self.text.blit(s, "GND SPLR", 16, GREEN, (500, 372))
            return
        speedbrake = surfaces.speedbrake
        label = {0.0: "RET", 0.5: "1/2", 1.0: "FULL"}.get(round(speedbrake * 2) / 2, "MOV")
        self.text.blit(s, label, 16, GREEN if speedbrake == 0 else AMBER, (500, 372))

    def _controls(self, s: pygame.Surface, sim: Simulation) -> None:
        """Pitch trim, rudder position and wheel brakes, under the flaps."""
        ths = math.degrees(sim.surfaces.ths)
        self.text.blit(s, f"PITCH TRIM {abs(ths):3.1f} {'UP' if ths >= 0 else 'DN'}", 14, CYAN, (290, 352))
        x0, width, y = 220, 140, 380
        self.text.blit(s, "RUD", 14, WHITE, (x0 - 30, y))
        pygame.draw.line(s, GREY, (x0, y), (x0 + width, y), 2)
        pygame.draw.line(s, WHITE, (x0 + width / 2, y - 6), (x0 + width / 2, y + 6), 2)
        limit = math.radians(sim.aircraft.controls.rudder_deg)
        x = x0 + width / 2 + sim.surfaces.rudder / limit * width / 2
        pygame.draw.polygon(s, GREEN, [(x, y - 2), (x - 6, y - 10), (x + 6, y - 10)])
        if sim.inputs.brakes > 0:
            self.text.blit(s, "BRAKES", 14, AMBER, (500, 392))

    def _warnings(self, s: pygame.Surface, sim: Simulation) -> None:
        lim = sim.speed_limits()
        warnings: list[tuple[str, tuple[int, int, int]]] = []
        if sim.state.lift_limited and sim.cas_kt < lim.v_alpha_prot:
            warnings.append(("STALL", RED))
        if sim.cas_kt > lim.vmax + 4:
            warnings.append(("OVERSPEED", RED))
        if sim.agl_ft < 750 and sim.gear_pos < 1.0 and sim.vs_fpm < 0:
            warnings.append(("GEAR NOT DOWN", AMBER))
        for i, (msg, colour) in enumerate(warnings):
            blink = colour != RED or int(time.monotonic() * 3) % 2 == 0
            if blink:
                self.text.blit(s, msg, 22, colour, (290, 225 + i * 26 - 10))

    def _info(self, s: pygame.Surface, sim: Simulation, joystick_name: str) -> None:
        box = pygame.Rect(170, 400, 410, 185)
        pygame.draw.rect(s, TAPE_DARK, box)
        t = int(sim.time_s)
        wind_from = math.degrees(math.atan2(-sim.wind[0], -sim.wind[1])) % 360
        wind_kt = math.hypot(*sim.wind) / 0.514444
        lines = [
            f"{sim.airport.ident} RWY {sim.scenario.runway}  THR {sim.threshold_distance_nm:5.1f} NM",
            f"AGL {sim.agl_ft:6.0f} FT   GS {sim.groundspeed_kt:4.0f} KT",
            f"TRK {sim.track_deg:05.1f}   WIND {wind_from:03.0f}/{wind_kt:.0f}",
            f"LAW {sim.fcc.law:6s} MASS {sim.mass / 1000:4.1f} T  TIME {t // 3600:02d}:{t // 60 % 60:02d}:{t % 60:02d}",
            f"INPUT {joystick_name[:30]}",
            "H = help",
        ]
        for i, line in enumerate(lines):
            self.text.blit(s, line, 15, WHITE if i < 4 else GREY, (box.x + 12, box.y + 18 + i * 27), "midleft")
