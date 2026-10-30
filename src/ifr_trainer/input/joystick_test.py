"""A small window showing live joystick axes and buttons, for writing controls.toml."""

import pygame


def run_joystick_test() -> None:
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("Joystick test (Esc to quit)")
    font = pygame.font.SysFont("menlo,dejavusansmono,couriernew", 16)
    clock = pygame.time.Clock()
    sticks: dict[int, pygame.joystick.JoystickType] = {}
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        sticks[js.get_instance_id()] = js

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                pygame.quit()
                return
            if event.type == pygame.JOYDEVICEADDED:
                js = pygame.joystick.Joystick(event.device_index)
                sticks[js.get_instance_id()] = js
            if event.type == pygame.JOYDEVICEREMOVED:
                sticks.pop(event.instance_id, None)

        screen.fill((10, 10, 20))
        y = 10
        if not sticks:
            screen.blit(font.render("No joystick detected. Plug one in.", True, (255, 170, 0)), (10, y))
        for index, js in enumerate(sticks.values()):
            screen.blit(font.render(f"device {index}: {js.get_name()}", True, (0, 255, 255)), (10, y))
            y += 24
            for a in range(js.get_numaxes()):
                v = js.get_axis(a)
                screen.blit(font.render(f"axis {a}: {v:+.2f}", True, (255, 255, 255)), (20, y))
                pygame.draw.rect(screen, (80, 80, 90), (160, y + 4, 200, 10), 1)
                pygame.draw.rect(screen, (0, 220, 0), (260 + min(v, 0) * 100, y + 4, abs(v) * 100, 10))
                y += 20
            pressed = [str(b) for b in range(js.get_numbuttons()) if js.get_button(b)]
            screen.blit(font.render(f"buttons pressed: {' '.join(pressed) or '-'}", True, (255, 255, 255)), (20, y))
            y += 30
        pygame.display.flip()
        clock.tick(30)
