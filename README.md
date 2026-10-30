# IFR Trainer

A lightweight instrument-flying trainer for hand flying with A320-style displays.
It is loosely based on the A320, not a replica, and not a certified training device.
See [docs/PLAN.md](docs/PLAN.md) for the design and milestones.

## Setup

```bash
uv sync
```

## Fly

```bash
uv run ifr-trainer                       # default scenario: 12 NM final RWY 36
uv run ifr-trainer --list-scenarios
uv run ifr-trainer --scenario final36_xwind
```

## Joystick

1. `uv run ifr-trainer --joystick-test` shows live axis and button numbers.
2. Copy `src/ifr_trainer/data/controls.toml`, set your axes and buttons, then run
   `uv run ifr-trainer --controls my_controls.toml`.

After a reset, the thrust lever stays at the trimmed position until you move the
hardware throttle ("MOVE THR TO SYNC").

| Key | Action |
|---|---|
| Arrows | stick (keyboard fallback) |
| W / S | thrust lever up / down |
| 1 2 3 4 | lever to IDLE / CL / MCT / TOGA |
| F / Shift+F | flaps extend / retract |
| G | gear |
| B | speedbrake (RET → 1/2 → FULL) |
| P | pause |
| Ctrl+R | reset scenario |
| H | help |
| Esc | quit |

## Test

```bash
uv run pytest
```
