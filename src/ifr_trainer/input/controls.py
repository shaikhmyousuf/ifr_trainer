"""Joystick and keyboard input, turned into PilotInputs."""

from dataclasses import dataclass, field, fields
from pathlib import Path

import pygame

from ..data_files import load_toml
from ..sim.simulation import PilotInputs

THROTTLE_PICKUP = 0.05  # axis travel needed before a hardware throttle takes over
DETENT_SNAP = 0.015
MOUSE_STICK_RANGE_PX = 150.0  # drag distance for full stick deflection
MOUSE_WHEEL_LEVER_STEP = 0.02


@dataclass(frozen=True)
class JoystickMapping:
    device: int = 0
    pitch_axis: int = 1
    pitch_invert: bool = False
    roll_axis: int = 0
    roll_invert: bool = False
    throttle_axis: int = 2
    throttle_invert: bool = True
    deadzone: float = 0.05
    expo: float = 0.3
    buttons: dict[str, int] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path | None = None) -> "JoystickMapping":
        d = load_toml("controls.toml", path)["joystick"]
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in known})


def shape_axis(value: float, deadzone: float, expo: float) -> float:
    """Apply a deadzone (rescaled so output still reaches +/-1) and cubic expo."""
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    v = min((abs(value) - deadzone) / (1.0 - deadzone), 1.0)
    return sign * ((1.0 - expo) * v + expo * v**3)


class Controls:
    KEY_STICK_RATE = 3.0  # stick travel per second while an arrow key is held
    KEY_STICK_RETURN = 6.0
    KEY_STICK_LIMIT = 0.5  # arrow keys alone; hold Shift for full deflection
    KEY_LEVER_RATE = 0.25

    def __init__(self, mapping: JoystickMapping, detents: dict[str, float], flap_positions: int):
        self.mapping = mapping
        self.detents = detents
        self.flap_positions = flap_positions
        self.joystick: pygame.joystick.JoystickType | None = None
        self.inputs = PilotInputs()
        self.throttle_live = False
        self._throttle_ref = 0.0
        self._key_pitch = 0.0
        self._key_roll = 0.0
        self._mouse_anchor: tuple[int, int] | None = None
        self.open_joystick()

    def open_joystick(self) -> None:
        if pygame.joystick.get_count() > self.mapping.device:
            self.joystick = pygame.joystick.Joystick(self.mapping.device)
            self._throttle_ref = self._raw_throttle() or 0.0

    @property
    def throttle_needs_sync(self) -> bool:
        """True while a hardware throttle exists but has not yet been moved to take over."""
        return self._raw_throttle() is not None and not self.throttle_live

    @property
    def joystick_name(self) -> str:
        return self.joystick.get_name() if self.joystick else "no joystick (keyboard)"

    def sync(self, inputs: PilotInputs) -> None:
        """Adopt the simulation's inputs after a reset. The hardware throttle must be moved to take over."""
        self.inputs = PilotInputs(**vars(inputs))
        self.throttle_live = False
        self._throttle_ref = self._raw_throttle() or 0.0

    def handle_event(self, event: pygame.event.Event) -> str | None:
        """Process a discrete event. Returns an app action ('quit', 'reset', 'pause', 'help') or None."""
        if event.type == pygame.JOYDEVICEADDED and self.joystick is None:
            self.open_joystick()
        elif event.type == pygame.JOYDEVICEREMOVED and self.joystick is not None:
            if event.instance_id == self.joystick.get_instance_id():
                self.joystick = None
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self._mouse_anchor = event.pos
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self._mouse_anchor = None
        elif event.type == pygame.MOUSEWHEEL:
            self._set_lever_manually(self.inputs.lever + event.y * MOUSE_WHEEL_LEVER_STEP)
        elif event.type == pygame.JOYBUTTONDOWN:
            for action, button in self.mapping.buttons.items():
                if button == event.button:
                    return self._action(action)
        elif event.type == pygame.KEYDOWN:
            key, mods = event.key, event.mod
            if key == pygame.K_ESCAPE:
                return "quit"
            if key == pygame.K_r and mods & (pygame.KMOD_CTRL | pygame.KMOD_META):
                return "reset"
            if key == pygame.K_f:
                return self._action("flaps_up" if mods & pygame.KMOD_SHIFT else "flaps_down")
            simple = {
                pygame.K_g: "gear",
                pygame.K_b: "speedbrake",
                pygame.K_p: "pause",
                pygame.K_h: "help",
                pygame.K_m: "nd_mode",
                pygame.K_n: "nd_range",
                pygame.K_l: "ls",
            }
            if key in simple:
                return self._action(simple[key])
            names = {pygame.K_1: "IDLE", pygame.K_2: "CL", pygame.K_3: "MCT", pygame.K_4: "TOGA"}
            if key in names:
                self._set_lever_manually(self.detents[names[key]])
        return None

    def _action(self, action: str) -> str | None:
        i = self.inputs
        if action == "flaps_down":
            i.flap_lever = min(i.flap_lever + 1, self.flap_positions - 1)
        elif action == "flaps_up":
            i.flap_lever = max(i.flap_lever - 1, 0)
        elif action == "gear":
            i.gear_down = not i.gear_down
        elif action == "speedbrake":
            i.speedbrake = {0.0: 0.5, 0.5: 1.0}.get(i.speedbrake, 0.0)
        else:
            return action
        return None

    def _set_lever_manually(self, lever: float) -> None:
        self.inputs.lever = min(max(lever, 0.0), 1.0)
        self.throttle_live = False
        self._throttle_ref = self._raw_throttle() or 0.0

    def _raw_throttle(self) -> float | None:
        m = self.mapping
        if self.joystick is None or not 0 <= m.throttle_axis < self.joystick.get_numaxes():
            return None
        return self.joystick.get_axis(m.throttle_axis)

    def _axis(self, index: int, invert: bool) -> float:
        if self.joystick is None or not 0 <= index < self.joystick.get_numaxes():
            return 0.0
        v = self.joystick.get_axis(index)
        return shape_axis(-v if invert else v, self.mapping.deadzone, self.mapping.expo)

    def _key_axis(self, current: float, negative: bool, positive: bool, limit: float, dt: float) -> float:
        if positive != negative:
            target = 1.0 if positive else -1.0
            return max(-limit, min(limit, current + target * self.KEY_STICK_RATE * dt))
        step = self.KEY_STICK_RETURN * dt
        return 0.0 if abs(current) <= step else current - step * (1 if current > 0 else -1)

    def poll(self, dt: float) -> PilotInputs:
        keys = pygame.key.get_pressed()
        m = self.mapping
        limit = 1.0 if pygame.key.get_mods() & pygame.KMOD_SHIFT else self.KEY_STICK_LIMIT
        self._key_pitch = self._key_axis(self._key_pitch, keys[pygame.K_UP], keys[pygame.K_DOWN], limit, dt)
        self._key_roll = self._key_axis(self._key_roll, keys[pygame.K_LEFT], keys[pygame.K_RIGHT], limit, dt)
        mouse_pitch = mouse_roll = 0.0
        if self._mouse_anchor is not None:
            # Spring-loaded like a sidestick: deflection follows the drag, centres on release.
            x, y = pygame.mouse.get_pos()
            mouse_roll = (x - self._mouse_anchor[0]) / MOUSE_STICK_RANGE_PX
            mouse_pitch = (y - self._mouse_anchor[1]) / MOUSE_STICK_RANGE_PX
        i = self.inputs
        i.stick_pitch = max(-1.0, min(1.0, self._axis(m.pitch_axis, m.pitch_invert) + self._key_pitch + mouse_pitch))
        i.stick_roll = max(-1.0, min(1.0, self._axis(m.roll_axis, m.roll_invert) + self._key_roll + mouse_roll))

        if keys[pygame.K_w] or keys[pygame.K_s]:
            delta = (self.KEY_LEVER_RATE if keys[pygame.K_w] else -self.KEY_LEVER_RATE) * dt
            self._set_lever_manually(i.lever + delta)
        raw = self._raw_throttle()
        if raw is not None:
            if not self.throttle_live and abs(raw - self._throttle_ref) > THROTTLE_PICKUP:
                self.throttle_live = True
            if self.throttle_live:
                lever = (1.0 - raw) / 2.0 if m.throttle_invert else (raw + 1.0) / 2.0
                for detent in self.detents.values():
                    if abs(lever - detent) < DETENT_SNAP:
                        lever = detent
                i.lever = lever
        return i
