"""Clickable panel fields shared by aircraft panels: radios, courses, references, the minimum.

Fields respond to the mouse wheel (Shift = coarse step) and to clicks: left half
decreases, right half increases (or cycles for choices).
"""

from dataclasses import dataclass
from typing import Callable

import pygame

from ..sim.cockpit import Cockpit


def cycle(options: tuple, current, direction: int):
    return options[(options.index(current) + direction) % len(options)]


def wrap_deg(value: float) -> float:
    v = value % 360.0
    return 360.0 if v == 0.0 else v


@dataclass
class Field:
    label: str
    rect: pygame.Rect  # value box, panel coordinates
    show: Callable[[Cockpit], str]
    change: Callable[[Cockpit, int, bool], None]  # (cockpit, direction, coarse)


def freq_field(attr: str, lo: float, hi: float, fine: float, coarse: float):
    def change(c: Cockpit, direction: int, big: bool) -> None:
        value = getattr(c, attr) + direction * (coarse if big else fine)
        span = hi - lo + fine
        setattr(c, attr, round(lo + (value - lo) % span, 2))

    return change


def course_field(attr: str):
    def change(c: Cockpit, direction: int, big: bool) -> None:
        setattr(c, attr, wrap_deg(round(getattr(c, attr)) + direction * (10 if big else 1)))

    return change


def change_altitude(c: Cockpit, direction: int, big: bool) -> None:
    c.alt_target_ft = min(max(c.alt_target_ft + direction * (1000 if big else 100), 0.0), 39000.0)


def change_minimum(c: Cockpit, direction: int, big: bool) -> None:
    if c.minimum_ft is None:
        c.minimum_ft = 700.0
        return
    value = c.minimum_ft + direction * (100 if big else 10)
    c.minimum_ft = None if value < 500 else value
