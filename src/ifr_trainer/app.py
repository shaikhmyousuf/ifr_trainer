"""Main loop: fixed-step simulation at 120 Hz, rendering at up to 60 Hz."""

import argparse
from pathlib import Path

import pygame

from .displays.common import AMBER, BLACK, GREEN, RED, WHITE, Text
from .displays.engine_panel import EnginePanel
from .displays.pfd import PFD
from .input.controls import Controls, JoystickMapping
from .input.joystick_test import run_joystick_test
from .sim.aircraft import AircraftData
from .sim.airport import Airport
from .sim.scenario import load_scenarios
from .sim.simulation import Phase, Severity, Simulation

SIM_DT = 1.0 / 120.0
MAX_FRAME_S = 0.25
WINDOW = (1240, 640)
MESSAGE_S = 20.0

HELP = [
    "Mouse       hold left button + drag = sidestick (release centres)",
    "            drag down = pull (nose up), sideways = roll",
    "Wheel       thrust lever",
    "Arrows      stick, half deflection (Shift = full)",
    "Z / X       rudder pedals left / right (steer on the ground)",
    "Space       wheel brakes (hold)",
    "W / S       thrust lever up / down",
    "1 2 3 4     lever to IDLE / CL / MCT / TOGA",
    "F / Shift+F flaps extend / retract",
    "G           gear     B  speedbrake",
    "P           pause    Ctrl+R  reset scenario",
    "H           help     Esc  quit",
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
    pfd = PFD(text)
    panel = EnginePanel(text)
    controls = Controls(JoystickMapping.load(controls_path), aircraft.engines.detents, len(aircraft.flaps))
    controls.sync(sim.inputs)
    clock = pygame.time.Clock()
    accumulator = 0.0
    paused = False
    show_help = False

    while True:
        frame = min(clock.tick(60) / 1000.0, MAX_FRAME_S)
        for event in pygame.event.get():
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

        inputs = controls.poll(frame)
        if not paused:
            accumulator += frame
            while accumulator >= SIM_DT:
                sim.step(SIM_DT, inputs)
                accumulator -= SIM_DT

        screen.fill(BLACK)
        screen.blit(pfd.draw(sim), (20, 20))
        screen.blit(panel.draw(sim, controls.throttle_needs_sync, controls.joystick_name), (620, 20))
        if paused:
            text.blit(screen, "PAUSED", 36, AMBER, (WINDOW[0] / 2, 40))
        crashed = sim.phase is Phase.CRASHED
        if sim.message and (crashed or sim.time_s - sim.message_time < MESSAGE_S):
            colour = {Severity.INFO: GREEN, Severity.CAUTION: AMBER, Severity.CRASH: RED}[sim.severity]
            text.blit(screen, sim.message, 18, colour, (WINDOW[0] / 2, WINDOW[1] - 40))
        if crashed:
            text.blit(screen, "Ctrl+R to fly again", 16, WHITE, (WINDOW[0] / 2, WINDOW[1] - 16))
        if show_help:
            overlay = pygame.Surface((680, 40 + 26 * len(HELP)), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 220))
            for i, line in enumerate(HELP):
                text.blit(overlay, line, 16, WHITE, (20, 30 + i * 26), "midleft")
            screen.blit(overlay, overlay.get_rect(center=(WINDOW[0] / 2, WINDOW[1] / 2)))
        pygame.display.flip()


def main() -> None:
    parser = argparse.ArgumentParser(prog="ifr-trainer", description="A320-style IFR hand-flying trainer")
    parser.add_argument("--scenario", default="final36", help="start scenario (default: final36)")
    parser.add_argument("--list-scenarios", action="store_true", help="list scenarios and exit")
    parser.add_argument("--controls", type=Path, help="path to a controls.toml with your joystick mapping")
    parser.add_argument("--joystick-test", action="store_true", help="show live joystick axes and buttons")
    args = parser.parse_args()

    if args.list_scenarios:
        for name, sc in load_scenarios().items():
            print(f"{name:16s} {sc.description}")
    elif args.joystick_test:
        run_joystick_test()
    else:
        run(args.scenario, args.controls)
