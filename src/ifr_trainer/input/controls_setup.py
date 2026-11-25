"""Joystick setup: a window that asks the pilot to move each control and press each button, and
saves the mapping as the user's own controls.toml. No axis numbers or invert flags to work out.
"""

from dataclasses import replace
from pathlib import Path

import pygame

from .controls import Control, JoystickMapping, describe_control, hat_directions, user_controls_path

AXIS_DETECT = 0.6  # travel from rest that picks an axis
# Axes: (field prefix, what to do first, then the movement that identifies the axis and its direction).
# Each movement must read +1 for the simulator (pull back, roll right) or -1 inverted (throttle
# forward with throttle_invert), so in every case a negative reading means "invert".
AXIS_STEPS = [
    ("pitch", "Let go of the stick", "Pull the stick fully back"),
    ("roll", "Let go of the stick", "Push the stick fully right"),
    ("throttle", "Pull the throttle back to idle", "Push the throttle fully forward"),
]
BUTTON_STEPS = [
    ("flaps_down", "Flaps down (extend)"),
    ("flaps_up", "Flaps up (retract)"),
    ("gear", "Gear up / down"),
    ("speedbrake", "Speedbrake"),
    ("trim_nose_up", "Pitch trim nose up (held; C172 and jet)"),
    ("trim_nose_down", "Pitch trim nose down (held; C172 and jet)"),
    ("fpv", "Flight path vector on the PFD (A320 and jet)"),
    ("pause", "Pause"),
    ("reset", "Reset the scenario"),
]

BG = (10, 10, 20)
WHITE = (255, 255, 255)
GREY = (150, 150, 160)
CYAN = (0, 255, 255)
GREEN = (0, 220, 0)
AMBER = (255, 170, 0)


def pick_axis(rest: list[float], now: list[float], exclude: set[int]) -> tuple[int, bool] | None:
    """The axis moved furthest from rest, if it moved far enough: (axis, invert)."""
    moved = [(abs(v - r), a) for a, (r, v) in enumerate(zip(rest, now)) if a not in exclude]
    if not moved or max(moved)[0] < AXIS_DETECT:
        return None
    axis = max(moved)[1]
    return axis, now[axis] < rest[axis]


def build_mapping(
    base: JoystickMapping, device: int, axes: dict[str, tuple[int, bool]], buttons: dict[str, Control]
) -> JoystickMapping:
    """``base`` (deadzone, expo, buttons added by hand) with the controls found by the setup.
    Hand-added buttons are kept unless the setup gave their button to something else."""
    axis_fields = {}
    for name, _, _ in AXIS_STEPS:
        axis, invert = axes.get(name, (-1, base.throttle_invert if name == "throttle" else False))
        axis_fields[f"{name}_axis"], axis_fields[f"{name}_invert"] = axis, invert
    used = {b for b in buttons.values() if b != -1}
    kept = {a: b for a, b in base.buttons.items() if a not in buttons and b not in used}
    return replace(base, device=device, **axis_fields, buttons={**buttons, **kept})


class Setup:
    """The steps, one at a time: choose the device, then each axis, then each button."""

    def __init__(self, path: Path):
        self.path = path
        self.base = JoystickMapping.load()  # the user's own file if there is one, so deadzone and expo survive
        self.sticks: list[pygame.joystick.JoystickType] = []
        self.device: int | None = None
        self.step = 0  # index into AXIS_STEPS + BUTTON_STEPS
        self.rest: list[float] | None = None  # axis readings at rest, once confirmed
        self.axes: dict[str, tuple[int, bool]] = {}
        self.buttons: dict[str, Control] = {}
        self.note = ""
        self.saved = False
        self.refresh()

    @property
    def steps(self) -> int:
        return len(AXIS_STEPS) + len(BUTTON_STEPS)

    @property
    def stick(self) -> pygame.joystick.JoystickType | None:
        return self.sticks[self.device] if self.device is not None else None

    def refresh(self) -> None:
        self.sticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        if self.device is not None and self.device >= len(self.sticks):
            self.device, self.step, self.rest = None, 0, None
        if self.device is None and len(self.sticks) == 1:
            self.device = 0

    def axis_values(self) -> list[float]:
        return [self.stick.get_axis(a) for a in range(self.stick.get_numaxes())]

    def go(self, step: int) -> None:
        self.step, self.rest, self.note = max(step, 0), None, ""
        if self.step < len(AXIS_STEPS):
            self.axes.pop(AXIS_STEPS[self.step][0], None)
        elif self.step < self.steps:
            self.buttons.pop(BUTTON_STEPS[self.step - len(AXIS_STEPS)][0], None)
        else:
            mapping = build_mapping(self.base, self.device, self.axes, self.buttons)
            mapping.save(self.path)
            self.saved = True

    def handle(self, event: pygame.event.Event) -> bool:
        """Process an event. False when the window should close."""
        if event.type == pygame.QUIT:
            return False
        if event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
            self.refresh()
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE or (self.saved and event.key == pygame.K_RETURN):
                return False
            if self.saved or self.device is None:
                return True
            if event.key == pygame.K_BACKSPACE:
                self.go(self.step - 1)
            elif event.key == pygame.K_RETURN and self.step < len(AXIS_STEPS) and self.rest is None:
                self.rest = self.axis_values()
            elif event.key == pygame.K_s and self.step >= len(AXIS_STEPS):
                self.buttons[BUTTON_STEPS[self.step - len(AXIS_STEPS)][0]] = -1
                self.go(self.step + 1)
            elif event.key == pygame.K_s and AXIS_STEPS[self.step][0] == "throttle":
                self.go(self.step + 1)  # no throttle axis: build_mapping sets -1
            return True
        if event.type == pygame.JOYBUTTONDOWN and not self.saved:
            index = next((i for i, js in enumerate(self.sticks) if js.get_instance_id() == event.instance_id), None)
            if self.device is None:
                self.device = index
            elif index == self.device and self.step < len(AXIS_STEPS):
                if self.rest is None:
                    self.rest = self.axis_values()
            elif index == self.device:
                self.assign(event.button)
        if event.type == pygame.JOYHATMOTION and not self.saved and self.step >= len(AXIS_STEPS):
            if self.stick is not None and event.instance_id == self.stick.get_instance_id():
                pressed = hat_directions(event.hat, event.value)
                if len(pressed) == 1:  # a diagonal is ambiguous: wait for a single direction
                    self.assign(pressed.pop())
        return True

    def assign(self, control: Control) -> None:
        """The current button step's action goes on ``control`` (a button or hat direction)."""
        taken = next((a for a, b in self.buttons.items() if b == control), None)
        if taken:
            self.note = f"{describe_control(control).capitalize()} is already used for {taken}."
        else:
            self.buttons[BUTTON_STEPS[self.step - len(AXIS_STEPS)][0]] = control
            self.go(self.step + 1)

    def update(self) -> None:
        if self.saved or self.stick is None or self.step >= len(AXIS_STEPS) or self.rest is None:
            return
        name = AXIS_STEPS[self.step][0]
        exclude = {axis for n, (axis, _) in self.axes.items() if n != name}
        found = pick_axis(self.rest, self.axis_values(), exclude)
        if found:
            self.axes[name] = found
            self.go(self.step + 1)

    # --- Drawing -----------------------------------------------------------------------

    def draw(self, screen: pygame.Surface, big: pygame.font.Font, font: pygame.font.Font) -> None:
        screen.fill(BG)

        def line(text: str, y: int, colour=WHITE, f=font) -> None:
            screen.blit(f.render(text, True, colour), (24, y))

        line("Joystick setup", 18, CYAN, big)
        if self.saved:
            line("Saved. The simulator now uses this mapping:", 80)
            line(str(self.path), 110, GREEN)
            line("Edit that file for deadzone and expo, or run the setup again to redo it.", 150, GREY)
            line("Enter or Esc to close.", 190, GREY)
            return
        if not self.sticks:
            line("No joystick detected. Plug one in.", 80, AMBER, big)
            line("Esc to quit.", 130, GREY)
            return
        if self.device is None:
            line("Press any button on the joystick you want to use.", 80, WHITE, big)
            for i, js in enumerate(self.sticks):
                line(f"device {i}: {js.get_name()}", 130 + 24 * i, GREY)
            return

        line(f"{self.stick.get_name()}   -   step {self.step + 1} of {self.steps}", 56, GREY)
        if self.step < len(AXIS_STEPS):
            name, rest_prompt, move_prompt = AXIS_STEPS[self.step]
            if self.rest is None:
                line(f"{rest_prompt},", 100, WHITE, big)
                line("then press Enter or any button on the stick.", 134, WHITE, big)
            else:
                line(f"{move_prompt}.", 100, WHITE, big)
            hint = "S = no throttle on this joystick" if name == "throttle" else ""
        else:
            label = BUTTON_STEPS[self.step - len(AXIS_STEPS)][1]
            line("Press the button (or push the hat) for:", 100, WHITE, big)
            line(label, 134, GREEN, big)
            hint = "S = skip (not assigned)"
        line(self.note, 172, AMBER)
        line(hint, 200, GREY)
        line("Backspace = redo the previous step   Esc = quit without saving", 222, GREY)

        y = 260
        for a, v in enumerate(self.axis_values()):
            line(f"axis {a}: {v:+.2f}", y, GREY)
            pygame.draw.rect(screen, (80, 80, 90), (180, y + 4, 200, 10), 1)
            pygame.draw.rect(screen, GREEN, (280 + min(v, 0) * 100, y + 4, abs(v) * 100, 10))
            y += 20
        found = [f"{n}: axis {a}{' (inverted)' if inv else ''}" for n, (a, inv) in self.axes.items()]
        found += [f"{n}: {describe_control(b)}" for n, b in self.buttons.items()]
        for i, text in enumerate(found):
            screen.blit(font.render(text, True, WHITE), (420, 260 + 20 * i))


def run_controls_setup(path: Path | None = None) -> None:
    pygame.init()
    screen = pygame.display.set_mode((760, 480))
    pygame.display.set_caption("Joystick setup (Esc to quit)")
    big = pygame.font.SysFont("menlo,dejavusansmono,couriernew", 20)
    font = pygame.font.SysFont("menlo,dejavusansmono,couriernew", 15)
    setup = Setup(path or user_controls_path())
    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if not setup.handle(event):
                pygame.quit()
                return
        setup.update()
        setup.draw(screen, big, font)
        pygame.display.flip()
        clock.tick(60)
