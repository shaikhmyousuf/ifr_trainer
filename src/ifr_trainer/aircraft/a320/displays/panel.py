"""A320 panel: PFD and ND side by side above the lower panel, plus the EFIS keys."""

import pygame

from ifr_trainer.displays.common import BLACK, Text
from ifr_trainer.sim.simulation import Simulation

from ..cockpit import ND_MODES, ND_RANGES
from .lower_panel import LowerPanel
from .nd import ND
from .pfd import PFD

PFD_POS, ND_POS, LOWER_POS = (10, 10), (630, 10), (10, 620)


class A320Panel:
    size = (1240, 780)
    message_y = 596
    help = [
        "M / N / L / V   ND mode / ND range / ILS scales / flight path vector on PFD",
        "1 2 3 4     thrust lever to IDLE / CL / MCT / TOGA",
        "At the minimum you are still in cloud: TOGA, pitch up, gear up, flaps one step.",
    ]
    key_actions = {pygame.K_m: "nd_mode", pygame.K_n: "nd_range", pygame.K_l: "ls", pygame.K_v: "fpv"}

    def __init__(self, text: Text):
        self.pfd, self.nd, self.lower = PFD(text), ND(text), LowerPanel(text)
        self.lower_rect = pygame.Rect(LOWER_POS, self.lower.surface.get_size())

    def _lower_pos(self, pos: tuple[int, int]) -> tuple[int, int] | None:
        r = self.lower_rect
        return (pos[0] - r.x, pos[1] - r.y) if r.collidepoint(pos) else None

    def draw(self, screen: pygame.Surface, sim: Simulation, throttle_needs_sync: bool) -> None:
        nav = sim.nav_readings()
        screen.fill(BLACK)
        screen.blit(self.pfd.draw(sim, nav), PFD_POS)
        screen.blit(self.nd.draw(sim, nav), ND_POS)
        hover = self._lower_pos(pygame.mouse.get_pos())
        screen.blit(self.lower.draw(sim, nav, throttle_needs_sync, hover), LOWER_POS)

    def click(self, sim: Simulation, pos: tuple[int, int], coarse: bool) -> bool:
        local = self._lower_pos(pos)
        return local is not None and self.lower.click(sim.cockpit, local, coarse)

    def wheel(self, sim: Simulation, pos: tuple[int, int], steps: int, coarse: bool) -> bool:
        local = self._lower_pos(pos)
        return local is not None and self.lower.wheel(sim.cockpit, local, steps, coarse)

    def action(self, sim: Simulation, action: str) -> None:
        c = sim.cockpit
        if action == "nd_mode":
            c.nd_mode = ND_MODES[(ND_MODES.index(c.nd_mode) + 1) % len(ND_MODES)]
        elif action == "nd_range":
            c.nd_range = ND_RANGES[(ND_RANGES.index(c.nd_range) + 1) % len(ND_RANGES)]
        elif action == "ls":
            c.ls = not c.ls
        elif action == "fpv":
            c.fpv = not c.fpv
