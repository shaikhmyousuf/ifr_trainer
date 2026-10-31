"""Cessna 172 panel: the shared round-dial panel with a tachometer."""

import pygame

from ifr_trainer.displays.steam_panel import SteamPanel
from ifr_trainer.sim.simulation import Simulation


class C172Panel(SteamPanel):
    help = SteamPanel.help + ["Power controls altitude, pitch and trim control speed: trim off every pressure."]

    def engine_gauge(self, s: pygame.Surface, centre: tuple[int, int], sim: Simulation) -> None:
        self.g.tachometer(s, centre, sim.propulsion.rpm)
