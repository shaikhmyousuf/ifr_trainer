"""Colours, cached text rendering and shared drawing helpers (Airbus-style palette)."""

import pygame

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 230, 0)
AMBER = (255, 170, 0)
RED = (255, 30, 30)
YELLOW = (255, 255, 0)
CYAN = (0, 230, 255)
MAGENTA = (255, 0, 255)
GREY = (120, 120, 130)
SKY = (40, 120, 215)
GROUND = (130, 85, 40)
TAPE = (65, 65, 75)
TAPE_DARK = (45, 45, 55)

FONT_FAMILY = "menlo,dejavusansmono,couriernew,monospace"


class Text:
    """Renders text with a cache, since most labels repeat every frame."""

    MAX_CACHE = 4000

    def __init__(self) -> None:
        self._fonts: dict[int, pygame.font.Font] = {}
        self._cache: dict[tuple[str, int, tuple[int, int, int]], pygame.Surface] = {}

    def render(self, text: str, size: int, color: tuple[int, int, int]) -> pygame.Surface:
        key = (text, size, color)
        surf = self._cache.get(key)
        if surf is None:
            font = self._fonts.get(size)
            if font is None:
                font = self._fonts[size] = pygame.font.SysFont(FONT_FAMILY, size, bold=True)
            if len(self._cache) > self.MAX_CACHE:
                self._cache.clear()
            surf = self._cache[key] = font.render(text, True, color)
        return surf

    def blit(
        self,
        target: pygame.Surface,
        text: str,
        size: int,
        color: tuple[int, int, int],
        pos: tuple[float, float],
        anchor: str = "center",
    ) -> pygame.Rect:
        surf = self.render(text, size, color)
        rect = surf.get_rect(**{anchor: (round(pos[0]), round(pos[1]))})
        target.blit(surf, rect)
        return rect


def draw_barber(
    surf: pygame.Surface,
    x: float,
    y_top: float,
    y_bottom: float,
    width: int,
    color_a: tuple[int, int, int],
    color_b: tuple[int, int, int],
    segment: int = 8,
) -> None:
    """A vertical barber-pole strip of alternating colours."""
    y = y_top
    flip = False
    while y < y_bottom:
        h = min(segment, y_bottom - y)
        pygame.draw.rect(surf, color_b if flip else color_a, (round(x), round(y), width, max(1, round(h))))
        y += segment
        flip = not flip
