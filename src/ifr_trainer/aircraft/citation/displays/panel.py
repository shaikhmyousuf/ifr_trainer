"""Business jet panel: the shared round-dial panel with a fan speed (N1) gauge, a barber pole
for VMO / MMO on the airspeed indicator, and gear and speedbrake annunciators."""

import pygame

from ifr_trainer.displays.common import AMBER, GREEN, RED
from ifr_trainer.displays.steam_panel import SteamPanel
from ifr_trainer.sim.atmosphere import cas_from_tas, isa
from ifr_trainer.sim.simulation import Simulation
from ifr_trainer.sim.units import KT

N1_RED_LINE = 104.0


class CitationPanel(SteamPanel):
    vsi_full_scale_fpm = 4000
    help = SteamPanel.help + [
        "G gear, B speedbrake. Trim off every pressure; the jet is slippery: plan descents early.",
    ]

    def engine_gauge(self, s: pygame.Surface, centre: tuple[int, int], sim: Simulation) -> None:
        self.g.n1(s, centre, sim.propulsion.n1, N1_RED_LINE)

    def vmax_kt(self, sim: Simulation) -> float:
        ac = sim.airframe
        air = isa(sim.state.h_m)
        mmo_cas = cas_from_tas(ac.mmo * air.speed_of_sound, air) / KT
        return min(ac.vmo_kt, mmo_cas)

    def extra_lights(self, sim: Simulation) -> list[tuple[str, tuple[int, int, int], bool]]:
        gear = sim.gear_pos
        in_transit = 0.0 < gear < 1.0
        return [
            ("GEAR UNSAFE" if in_transit else "GEAR DOWN", RED if in_transit else GREEN, gear > 0.0),
            ("SPD BRK", AMBER, sim.surfaces.speedbrake > 0.01),
        ]
