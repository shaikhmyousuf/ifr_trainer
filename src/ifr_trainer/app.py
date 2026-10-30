"""Main loop: fixed-step simulation at 120 Hz, rendering at up to 60 Hz."""

import argparse
from pathlib import Path

import pygame

from .displays.common import AMBER, BLACK, GREEN, RED, WHITE, Text
from .displays.lower_panel import LowerPanel
from .displays.nd import ND
from .displays.pfd import PFD
from .input.controls import Controls, JoystickMapping
from .input.joystick_test import run_joystick_test
from .sim.aircraft import AircraftData
from .sim.airport import Airport
from .sim.cockpit import ND_MODES, ND_RANGES
from .sim.scenario import load_scenarios
from .sim.simulation import Phase, Severity, Simulation

SIM_DT = 1.0 / 120.0
MAX_FRAME_S = 0.25
WINDOW = (1240, 780)
PFD_POS, ND_POS, PANEL_POS = (10, 10), (630, 10), (10, 620)
MESSAGE_S = 20.0

HELP = [
    "Mouse       hold left button + drag = sidestick (release centres)",
    "            drag down = pull (nose up), sideways = roll",
    "Wheel       thrust lever (over a panel field: tune it, Shift = coarse)",
    "Arrows      stick, half deflection (Shift = full)",
    "W / S       thrust lever up / down",
    "1 2 3 4     lever to IDLE / CL / MCT / TOGA",
    "F / Shift+F flaps extend / retract",
    "G           gear     B  speedbrake",
    "M / N / L   ND mode / ND range / ILS scales on PFD",
    "P           pause    Ctrl+R  reset scenario",
    "H           help     Esc  quit",
    "At the minimum you are still in cloud: TOGA, pitch up, gear up, flaps one step.",
]


def run(scenario_name: str, controls_path: Path | None) -> None:
    scenarios = load_scenarios()
    if scenario_name not in scenarios:
        raise SystemExit(f"unknown scenario {scenario_name!r}; try --list-scenarios")
    aircraft = AircraftData.load()
    sim = Simulation(aircraft, Airport.load(), scenarios[scenario_name])

    pygame.init()
    screen = pygame.display.set_mode(WINDOW, pygame.SCALED | pygame.RESIZABLE)
    pygame.display.set_caption(f"IFR Trainer — {scenario_name}")
    text = Text()
    pfd, nd, panel = PFD(text), ND(text), LowerPanel(text)
    panel_rect = pygame.Rect(PANEL_POS, panel.surface.get_size())
    controls = Controls(JoystickMapping.load(controls_path), aircraft.engines.detents, len(aircraft.flaps))
    controls.sync(sim.inputs)
    clock = pygame.time.Clock()
    accumulator = 0.0
    paused = False
    show_help = False

    def panel_pos(pos: tuple[int, int]) -> tuple[int, int] | None:
        return (pos[0] - panel_rect.x, pos[1] - panel_rect.y) if panel_rect.collidepoint(pos) else None

    while True:
        frame = min(clock.tick(60) / 1000.0, MAX_FRAME_S)
        coarse = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
        for event in pygame.event.get():
            # Mouse over the lower panel tunes fields instead of flying the aircraft.
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                local = panel_pos(event.pos)
                if local and panel.click(sim.cockpit, local, coarse):
                    continue
            if event.type == pygame.MOUSEWHEEL:
                local = panel_pos(pygame.mouse.get_pos())
                if local and panel.wheel(sim.cockpit, local, event.y, coarse):
                    continue
            action = "quit" if event.type == pygame.QUIT else controls.handle_event(event)
            c = sim.cockpit
            if action == "quit":
                pygame.quit()
                return
            if action == "reset":
                controls.sync(sim.reset())
                accumulator = 0.0
            elif action == "pause":
                paused = not paused
            elif action == "help":
                show_help = not show_help
            elif action == "nd_mode":
                c.nd_mode = ND_MODES[(ND_MODES.index(c.nd_mode) + 1) % len(ND_MODES)]
            elif action == "nd_range":
                c.nd_range = ND_RANGES[(ND_RANGES.index(c.nd_range) + 1) % len(ND_RANGES)]
            elif action == "ls":
                c.ls = not c.ls

        inputs = controls.poll(frame)
        if not paused:
            accumulator += frame
            while accumulator >= SIM_DT:
                sim.step(SIM_DT, inputs)
                accumulator -= SIM_DT

        nav = sim.nav_readings()
        screen.fill(BLACK)
        screen.blit(pfd.draw(sim, nav), PFD_POS)
        screen.blit(nd.draw(sim, nav), ND_POS)
        screen.blit(panel.draw(sim, nav, controls.throttle_needs_sync, panel_pos(pygame.mouse.get_pos())), PANEL_POS)
        if paused:
            text.blit(screen, "PAUSED", 36, AMBER, (WINDOW[0] / 2, 300))
        crashed = sim.phase is Phase.CRASHED
        if sim.message and (crashed or sim.time_s - sim.message_time < MESSAGE_S):
            colour = {Severity.INFO: GREEN, Severity.CAUTION: AMBER, Severity.CRASH: RED}[sim.severity]
            banner = text.render(sim.message, 18, colour)
            box = banner.get_rect(center=(WINDOW[0] / 2, 596)).inflate(24, 10)
            pygame.draw.rect(screen, BLACK, box)
            pygame.draw.rect(screen, colour, box, 1)
            screen.blit(banner, banner.get_rect(center=box.center))
        if crashed:
            text.blit(screen, "Ctrl+R to fly again", 16, WHITE, (WINDOW[0] / 2, 572))
        if show_help:
            overlay = pygame.Surface((760, 40 + 26 * len(HELP)), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 225))
            for i, line in enumerate(HELP):
                text.blit(overlay, line, 16, WHITE, (20, 30 + i * 26), "midleft")
            screen.blit(overlay, overlay.get_rect(center=(WINDOW[0] / 2, 320)))
        pygame.display.flip()


def main() -> None:
    parser = argparse.ArgumentParser(prog="ifr-trainer", description="A320-style instrument flying trainer")
    parser.add_argument("--scenario", default="ils36", help="start scenario (default: ils36)")
    parser.add_argument("--list-scenarios", action="store_true", help="list scenarios and exit")
    parser.add_argument("--controls", type=Path, help="path to a controls.toml with your joystick mapping")
    parser.add_argument("--joystick-test", action="store_true", help="show live joystick axes and buttons")
    args = parser.parse_args()

    if args.list_scenarios:
        for name, sc in load_scenarios().items():
            print(f"{name:20s} {sc.description}")
    elif args.joystick_test:
        run_joystick_test()
    else:
        run(args.scenario, args.controls)
