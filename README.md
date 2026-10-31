# IFR Trainer

A lightweight instrument-flying trainer for hand flying. Aircraft are modules that share
one environment (airport, navaids, weather, physics) but bring their own handling,
engines, flight controls and displays. The first module is loosely based on the A320.
Every exercise is flown in cloud and starts in the air. Approaches end with a
go-around on instruments at the minimum; there is no takeoff or landing.
It is loosely based on the A320, not a replica, and not a certified training device.
See [docs/PLAN.md](docs/PLAN.md) for the design and milestones.

## Setup

```bash
uv sync
```

## Fly

```bash
uv run ifr-trainer                      # default: A320, vectors for ILS 36
uv run ifr-trainer --list-aircraft
uv run ifr-trainer --list-scenarios     # for the chosen aircraft (--aircraft a320)
uv run ifr-trainer --aircraft a320 --scenario vor36
```

Scenarios include ILS (with and without crosswind), VOR/DME and NDB approaches,
VOR radial tracking and a hold. At the minimum ("MINIMUM" on the PFD and the
altitude digits turn amber), go around: TOGA, pitch to about 15°, gear up, flaps
one step. The go-around is scored by how far you sank below the minimum.

## Displays (A320)

* **PFD**: attitude, speed and altitude tapes, VSI, heading with the FCU heading bug,
  altitude target, BARO minimum, and with **LS** on: localizer and glideslope
  scales, ILS course, ident, frequency and DME.
* **ND**: ROSE ILS, ROSE VOR (course pointer, deviation, TO/FROM) and ARC (map),
  two bearing needles (VOR or ADF), wind, GS/TAS, DME.
* **Lower panel**: engines, flaps, gear, speedbrake, pitch trim, and the nav radios
  and EFIS/FCU settings. Hover a field and use the mouse wheel to change it
  (Shift = coarse), or click its left/right half for −/+.

## Controls

No joystick is needed: the mouse works as a spring-loaded sidestick. With a joystick:

1. `uv run ifr-trainer --joystick-test` shows live axis and button numbers.
2. Copy `src/ifr_trainer/data/controls.toml`, set your axes and buttons, then run
   `uv run ifr-trainer --controls my_controls.toml`.

After a reset, the thrust lever stays at the trimmed position until you move the
hardware throttle ("MOVE THR").

| Input | Action |
|---|---|
| Mouse: hold left button + drag | sidestick, springs back to centre on release (drag down = pull) |
| Mouse wheel | thrust lever (over a panel field: tune it) |
| Arrows | stick, half deflection (hold Shift for full) |
| W / S | thrust lever up / down |
| 1 2 3 … | thrust lever to the aircraft's detents (A320: IDLE / CL / MCT / TOGA) |
| F / Shift+F | flaps extend / retract |
| G | gear |
| B | speedbrake (RET → 1/2 → FULL) |
| M / N / L | A320: ND mode / ND range / ILS scales on the PFD |
| P | pause |
| Ctrl+R | reset scenario |
| H | help |
| Esc | quit |

## Adding an aircraft

1. Create a package `src/ifr_trainer/aircraft/<name>/` with:
   * `aircraft.toml`: the shared airframe format (mass, geometry, inertia, aero derivatives,
     controls, gear, flaps; see `sim/airframe.py`) plus anything the module reads itself;
   * `scenarios.toml`: its speed, flaps and cockpit settings for each shared scenario it offers;
   * `__init__.py`: an `Aircraft` subclass (see `sim/aircraft.py`) and a `build()` function.
     It supplies the engines, the flight control system, the cockpit state and the panel.
2. Add it to `AIRCRAFT` in `src/ifr_trainer/aircraft/__init__.py`.

`tests/test_aircraft_modules.py` then checks it automatically: the interface, and that
every scenario starts trimmed and flies.

## Test

```bash
uv run pytest
```
