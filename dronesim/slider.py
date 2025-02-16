import pygame
from dataclasses import dataclass
from typing import Callable

@dataclass
class Slider:
    color: tuple[float, float, float]
    deactivated_color: tuple[float, float, float]
    minimum: float
    maximum: float
    value: float
    width: int
    height: int
    decimal_places: float
    rendered_text: pygame.Surface
    enabled: bool = True
    dragging: bool = False
    value_renderer: Callable[[float], pygame.Surface] | None = None

    def render(self) -> pygame.Surface:
        out = pygame.Surface((self.width, self.height))
        out.blit(self.rendered_text, (0, self.height // 2 - self.rendered_text.get_height() // 2))
        min_x = self._slider_start()
        max_x = self._slider_end()
        radius = 5
        pygame.draw.line(out, self.deactivated_color, (min_x, self.height // 2), (max_x, self.height // 2), 3)
        x = min_x + (max_x - min_x) * (self.value - self.minimum)/(self.maximum - self.minimum)
        c = self.color if self.enabled else self.deactivated_color
        pygame.draw.line(out, c, (min_x, self.height // 2), (x, self.height // 2), 3)
        pygame.draw.circle(out, c, (x, self.height//2), radius)
        if self.value_renderer is not None:
            surf = self.value_renderer(self.value)
            out.blit(surf, (max_x + 7, self.height // 2 - surf.get_height() // 2))
        out.set_colorkey((0, 0, 0))
        return out.convert()
    
    def _slider_start(self):
        return self.width*.15

    def _slider_end(self):
        return self.width*.8

    def tick(self, x: int, y: int, event: pygame.event.Event) -> None:
        area = pygame.Rect(x, y, self.width, self.height)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == pygame.BUTTON_LEFT and area.collidepoint(event.pos) and self.enabled:
            self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == pygame.BUTTON_LEFT:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging and self.enabled:
            min_x = self._slider_start()
            max_x = self._slider_end()
            mouse_x = pygame.mouse.get_pos()[0]
            val = (self.maximum - self.minimum) * (mouse_x - x - min_x) / (max_x - min_x)
            self.value = min(max(val, self.minimum), self.maximum)

