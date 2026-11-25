"""Joystick mapping files and the setup that writes them."""

import pygame
import pytest

from ifr_trainer.input.controls import JoystickMapping, controls_file, user_controls_path
from ifr_trainer.input.controls_setup import AXIS_STEPS, BUTTON_STEPS, Setup, build_mapping, pick_axis


@pytest.fixture(autouse=True)
def config_home(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    return tmp_path / "config"


def test_bundled_default_without_a_user_file():
    assert controls_file() is None
    assert JoystickMapping.load() == JoystickMapping.load(None)
    assert JoystickMapping.load().buttons["gear"] == 6


def test_user_file_overrides_the_default_and_keeps_what_it_leaves_out():
    path = user_controls_path()
    path.parent.mkdir(parents=True)
    path.write_text("[joystick]\nthrottle_axis = 3\n[joystick.buttons]\ngear = 2\nfpv = 9\n")
    assert controls_file() == path
    m = JoystickMapping.load()
    assert m.throttle_axis == 3 and m.pitch_axis == 1  # pitch from the default
    assert m.buttons["gear"] == 2 and m.buttons["fpv"] == 9 and m.buttons["flaps_down"] == 5


def test_explicit_file_wins_over_the_user_file(tmp_path):
    JoystickMapping(throttle_axis=3).save(user_controls_path())
    other = tmp_path / "other.toml"
    JoystickMapping(throttle_axis=-1).save(other)
    assert JoystickMapping.load(other).throttle_axis == -1
    assert JoystickMapping.load().throttle_axis == 3


def test_saved_mapping_loads_back_unchanged():
    m = JoystickMapping(device=1, pitch_invert=True, throttle_axis=3, deadzone=0.08, buttons={"gear": 2, "pause": -1})
    m.save(user_controls_path())
    loaded = JoystickMapping.load()
    assert loaded.buttons["gear"] == 2 and loaded.buttons["pause"] == -1
    assert replace_buttons(loaded) == replace_buttons(m)


def replace_buttons(m: JoystickMapping) -> JoystickMapping:
    return JoystickMapping(**{**vars(m), "buttons": {}})


def test_pick_axis_takes_the_axis_moved_furthest():
    rest = [0.0, 0.0, 0.0, -0.4]
    assert pick_axis(rest, [0.1, 0.3, 0.0, -0.4], set()) is None  # not far enough yet
    assert pick_axis(rest, [0.2, 0.9, 0.0, -0.4], set()) == (1, False)
    assert pick_axis(rest, [0.0, 0.0, 0.1, -1.0], set()) == (3, True)  # forward reads -1: inverted
    assert pick_axis(rest, [0.0, 0.9, 0.0, -0.4], {1}) is None  # already used for another control


def test_build_mapping_keeps_hand_added_buttons_unless_reassigned():
    base = JoystickMapping(deadzone=0.1, buttons={"nd_mode": 9, "ls": 10, "gear": 6})
    m = build_mapping(base, 0, {"pitch": (1, True), "roll": (0, False)}, {"gear": 2, "flaps_down": 10})
    assert m.pitch_axis == 1 and m.pitch_invert and m.throttle_axis == -1  # throttle skipped
    assert m.deadzone == 0.1
    assert m.buttons == {"gear": 2, "flaps_down": 10, "nd_mode": 9}  # ls lost button 10 to the flaps


class FakeStick:
    def __init__(self):
        self.axes = [0.0, 0.0, 0.0, -0.4]

    def get_axis(self, a):
        return self.axes[a]

    def get_numaxes(self):
        return len(self.axes)

    def get_instance_id(self):
        return 7

    def get_name(self):
        return "Fake stick"


def test_setup_walks_through_every_control_and_saves(monkeypatch):
    stick = FakeStick()
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", lambda i: stick)
    setup = Setup(user_controls_path())
    assert setup.device == 0  # the only joystick is chosen without asking

    def key(k):
        assert setup.handle(pygame.event.Event(pygame.KEYDOWN, key=k))

    def button(b):
        assert setup.handle(pygame.event.Event(pygame.JOYBUTTONDOWN, instance_id=7, button=b))

    def move(axes):
        stick.axes = axes
        setup.update()

    key(pygame.K_RETURN)  # stick at rest
    move([0.0, -1.0, 0.0, -0.4])  # pulled back reads -1 on this stick
    button(0)  # rest confirmed with a stick button
    move([1.0, 0.0, 0.0, -0.4])  # right
    stick.axes = [0.0, 0.0, 0.0, 1.0]  # throttle at idle
    key(pygame.K_RETURN)
    move([0.0, 0.0, 0.0, -1.0])  # forward
    assert setup.step == len(AXIS_STEPS)
    button(4)  # flaps down
    button(4)  # same button for flaps up: refused
    assert "already used" in setup.note
    button(5)
    key(pygame.K_BACKSPACE)  # redo flaps up
    button(3)
    for _ in BUTTON_STEPS[2:]:
        key(pygame.K_s)
    assert setup.saved
    m = JoystickMapping.load()
    assert (m.pitch_axis, m.pitch_invert, m.roll_axis, m.roll_invert) == (1, True, 0, False)
    assert (m.throttle_axis, m.throttle_invert) == (3, True)
    assert m.buttons["flaps_down"] == 4 and m.buttons["flaps_up"] == 3 and m.buttons["gear"] == -1


def test_setup_without_joystick_shows_a_message(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 0)
    pygame.init()
    screen = pygame.display.set_mode((760, 480))
    setup = Setup(user_controls_path())
    font = pygame.font.SysFont("menlo", 15)
    setup.draw(screen, font, font)
    assert setup.device is None and not setup.handle(pygame.event.Event(pygame.QUIT))


def test_hat_directions_save_and_load():
    JoystickMapping(buttons={"trim_nose_up": "hat0_down", "gear": 2}).save(user_controls_path())
    assert 'trim_nose_up = "hat0_down"' in user_controls_path().read_text()
    assert JoystickMapping.load().buttons["trim_nose_up"] == "hat0_down"


class FakeHatStick(FakeStick):
    def __init__(self):
        super().__init__()
        self.hat = (0, 0)

    def get_numhats(self):
        return 1

    def get_hat(self, h):
        return self.hat

    def get_numbuttons(self):
        return 12

    def get_button(self, b):
        return 0


def test_hat_held_trims_and_a_new_direction_is_an_action(aircraft, monkeypatch):
    from ifr_trainer.input.controls import Controls

    stick = FakeHatStick()
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", lambda i: stick)
    pygame.init()
    mapping = JoystickMapping(buttons={"trim_nose_up": "hat0_down", "trim_nose_down": "hat0_up", "gear": "hat0_left"})
    controls = Controls(mapping, aircraft.engines.detents, len(aircraft.airframe.flaps))
    stick.hat = (0, -1)  # thumb back
    assert controls.poll(0.016).pitch_trim == 1.0
    stick.hat = (0, 1)
    assert controls.poll(0.016).pitch_trim == -1.0
    stick.hat = (0, 0)
    assert controls.poll(0.016).pitch_trim == 0.0
    gear = controls.inputs.gear_down
    controls.handle_event(pygame.event.Event(pygame.JOYHATMOTION, instance_id=7, hat=0, value=(-1, 0)))
    controls.handle_event(pygame.event.Event(pygame.JOYHATMOTION, instance_id=7, hat=0, value=(-1, 1)))  # still held
    assert controls.inputs.gear_down != gear  # toggled once


def test_setup_takes_a_hat_direction(monkeypatch):
    stick = FakeHatStick()
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", lambda i: stick)
    setup = Setup(user_controls_path())
    setup.go(len(AXIS_STEPS) + 4)  # trim nose up

    def hat(value):
        setup.handle(pygame.event.Event(pygame.JOYHATMOTION, instance_id=7, hat=0, value=value))

    hat((1, -1))  # diagonal: ignored
    assert BUTTON_STEPS[setup.step - len(AXIS_STEPS)][0] == "trim_nose_up"
    hat((0, -1))
    hat((0, 0))  # released: ignored
    hat((0, -1))  # same direction for nose down: refused
    assert "already used" in setup.note
    hat((0, 1))
    assert setup.buttons == {"trim_nose_up": "hat0_down", "trim_nose_down": "hat0_up"}
