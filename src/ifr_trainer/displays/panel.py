"""The panel interface: an aircraft's displays and panel controls, as the main loop sees them."""

from typing import Protocol

import pygame

from ..sim.simulation import Simulation


class Panel(Protocol):
    size: tuple[int, int]  # window size in pixels
    message_y: int  # vertical centre of the message banner
    help: list[str]  # aircraft-specific lines for the help overlay
    key_actions: dict[int, str]  # extra keys -> action names handled by ``action``

    def draw(self, screen: pygame.Surface, sim: Simulation, throttle_needs_sync: bool) -> None: ...

    def click(self, sim: Simulation, pos: tuple[int, int], coarse: bool) -> bool:
        """A left click at window ``pos``. True if the panel used it (otherwise it moves the stick)."""
        ...

    def wheel(self, sim: Simulation, pos: tuple[int, int], steps: int, coarse: bool) -> bool:
        """Mouse wheel at window ``pos``. True if the panel used it (otherwise it moves the thrust lever)."""
        ...

    def action(self, sim: Simulation, action: str) -> None:
        """An action from ``key_actions`` or a joystick button."""
        ...
