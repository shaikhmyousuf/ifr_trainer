"""Main loop: fixed-step simulation at 120 Hz, rendering at up to 60 Hz.

Aircraft-independent: the aircraft module supplies the panel that draws the displays and
handles panel clicks and keys.
"""

import argparse
from pathlib import Path

import pygame

from .aircraft import AIRCRAFT, DEFAULT_AIRCRAFT, load_aircraft
from .displays.common import AMBER, BLACK, GREEN, RED, WHITE, Text
from .input.controls import Controls, JoystickMapping
from .input.joystick_test import run_joystick_test
from .sim.aircraft import Aircraft
from .sim.airport import Airport
from .sim.scenario import Scenario, load_scenarios
from .sim.simulation import Phase, Severity, Simulation

SIM_DT = 1.0 / 120.0
MAX_FRAME_S = 0.25
MESSAGE_S = 20.0
DEFAULT_SCENARIO = "ils36"

HELP = [
    "Mouse       hold left button + drag = stick (release centres)",
    "            drag down = pull (nose up), sideways = roll",
    "Wheel       thrust lever (over a panel field: tune it, Shift = coarse)",
    "Arrows      stick, half deflection (Shift = full)",
    "W / S       thrust lever up / down",
    "F / Shift+F flaps extend / retract",
    "G           gear     B  speedbrake",
    "P           pause    Ctrl+R  reset scenario",
    "H           help     Esc  quit",
]


def run(aircraft: Aircraft, scenario: Scenario, controls_path: Path | None) -> None:
    sim = Simulation(aircraft, Airport.load(), scenario)

    pygame.init()
    text = Text()
    panel = aircraft.new_panel(text)
    width, height = panel.size
    screen = pygame.display.set_mode(panel.size, pygame.SCALED | pygame.RESIZABLE)
    pygame.display.set_caption(f"IFR Trainer — {aircraft.title} — {scenario.name}")
    controls = Controls(
        JoystickMapping.load(controls_path), sim.propulsion.detents, len(sim.airframe.flaps), panel.key_actions
    )
    controls.sync(sim.inputs)
    help_lines = HELP + panel.help
    clock = pygame.time.Clock()
    accumulator = 0.0
    paused = False
    show_help = False

    while True:
        frame = min(clock.tick(60) / 1000.0, MAX_FRAME_S)
        coarse = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
        for event in pygame.event.get():
            # Mouse over a panel field tunes it instead of flying the aircraft.
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if panel.click(sim, event.pos, coarse):
                    continue
            if event.type == pygame.MOUSEWHEEL:
                if panel.wheel(sim, pygame.mouse.get_pos(), event.y, coarse):
                    continue
            action = "quit" if event.type == pygame.QUIT else controls.handle_event(event)
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
            elif action is not None:
                panel.action(sim, action)

        inputs = controls.poll(frame)
        if not paused:
            accumulator += frame
            while accumulator >= SIM_DT:
                sim.step(SIM_DT, inputs)
                accumulator -= SIM_DT

        panel.draw(screen, sim, controls.throttle_needs_sync)
        if paused:
            text.blit(screen, "PAUSED", 36, AMBER, (width / 2, 300))
        crashed = sim.phase is Phase.CRASHED
        if sim.message and (crashed or sim.time_s - sim.message_time < MESSAGE_S):
            colour = {Severity.INFO: GREEN, Severity.CAUTION: AMBER, Severity.CRASH: RED}[sim.severity]
            banner = text.render(sim.message, 18, colour)
            box = banner.get_rect(center=(width / 2, panel.message_y)).inflate(24, 10)
            pygame.draw.rect(screen, BLACK, box)
            pygame.draw.rect(screen, colour, box, 1)
            screen.blit(banner, banner.get_rect(center=box.center))
        if crashed:
            text.blit(screen, "Ctrl+R to fly again", 16, WHITE, (width / 2, panel.message_y - 24))
        if show_help:
            overlay = pygame.Surface((760, 40 + 26 * len(help_lines)), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 225))
            for i, line in enumerate(help_lines):
                text.blit(overlay, line, 16, WHITE, (20, 30 + i * 26), "midleft")
            screen.blit(overlay, overlay.get_rect(center=(width / 2, height * 0.41)))
        pygame.display.flip()


def describe(sc: Scenario) -> str:
    return f"{sc.name:16s} {sc.description} ({sc.cas_kt:.0f} kt, flaps {sc.flaps})"


def main() -> None:
    parser = argparse.ArgumentParser(prog="ifr-trainer", description="Instrument flying trainer")
    parser.add_argument("--aircraft", default=DEFAULT_AIRCRAFT, choices=sorted(AIRCRAFT), help="aircraft type")
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO, help=f"start scenario (default: {DEFAULT_SCENARIO})")
    parser.add_argument("--list-aircraft", action="store_true", help="list aircraft types and exit")
    parser.add_argument("--list-scenarios", action="store_true", help="list the aircraft's scenarios and exit")
    parser.add_argument("--controls", type=Path, help="path to a controls.toml with your joystick mapping")
    parser.add_argument("--joystick-test", action="store_true", help="show live joystick axes and buttons")
    args = parser.parse_args()

    if args.joystick_test:
        run_joystick_test()
        return
    if args.list_aircraft:
        for name in AIRCRAFT:
            print(f"{name:8s} {load_aircraft(name).title}")
        return
    aircraft = load_aircraft(args.aircraft)
    scenarios = load_scenarios(aircraft)
    if args.list_scenarios:
        for sc in scenarios.values():
            print(describe(sc))
    elif args.scenario not in scenarios:
        raise SystemExit(f"unknown scenario {args.scenario!r} for {aircraft.id}; try --list-scenarios")
    else:
        run(aircraft, scenarios[args.scenario], args.controls)
