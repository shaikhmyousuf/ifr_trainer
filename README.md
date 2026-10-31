# IFR Trainer

A lightweight instrument-flying trainer for hand flying. Aircraft are modules that share
one environment (airport, navaids, weather, physics) but bring their own handling,
engines, flight controls and displays:

* **A320** (`--aircraft a320`, default): loosely based on the A320, with a fly-by-wire
  normal law, PFD and ND.
* **Cessna 172P** (`--aircraft c172`): built from openly available data (JSBSim's `c172p`
  model and the C172P published figures), with direct controls, manual trim, a piston
  engine with a fixed-pitch propeller, and steam gauges.
* **Business jet** (`--aircraft citation`): a generic light jet on Cessna Citation II data
  (TU Delft's PH-LAB research aircraft: flight-test derivatives and measured drag), with
  direct controls, manual trim, two turbofans, speedbrakes and round dials.

Every exercise is flown in cloud and starts in the air. Approaches end with a
go-around on instruments at the minimum; there is no takeoff or landing.
It is not a certified training device.
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
uv run ifr-trainer --aircraft c172 --scenario ils36
uv run ifr-trainer --aircraft citation --scenario ils36_established
```

Scenarios include ILS (with and without crosswind), VOR/DME and NDB approaches,
VOR radial tracking and a hold. At the minimum ("MINIMUM" on the PFD and the
altitude digits turn amber), go around: TOGA, pitch to about 15°, gear up, flaps
one step (C172: full throttle, pitch up, flaps 20 then up; business jet: full thrust,
10° nose up, flaps 15, gear up once climbing). The go-around is scored
by how far you sank below the minimum.

## Displays (A320)

* **PFD**: attitude, speed and altitude tapes, VSI, heading with the FCU heading bug,
  altitude target, BARO minimum, and with **LS** on: localizer and glideslope
  scales, ILS course, ident, frequency and DME.
* **ND**: ROSE ILS, ROSE VOR (course pointer, deviation, TO/FROM) and ARC (map),
  two bearing needles (VOR or ADF), wind, GS/TAS, DME.
* **Lower panel**: engines, flaps, gear, speedbrake, pitch trim, and the nav radios
  and EFIS/FCU settings. Hover a field and use the mouse wheel to change it
  (Shift = coarse), or click its left/right half for −/+.

## Displays (C172)

* **Six-pack**: airspeed (white, green and yellow arcs, red line), attitude, altimeter,
  turn coordinator with ball, heading indicator with heading bug, vertical speed.
* **NAV1**: VOR or localizer with glideslope (5 dots each side), DME. **NAV2**: VOR.
  **ADF**: relative bearing. **Tachometer**.
* **Lower panel**: throttle, flaps, trim position, STALL and MINIMUM lights, and the
  radio stack (NAV1, NAV2, ADF, heading bug, minimum). The mouse wheel over NAV1 or
  NAV2 turns its OBS, over the heading indicator it moves the bug.

## Displays (business jet)

The same round-dial panel, with a jet airspeed indicator (white flap arc, barber pole at
VMO / MMO), an N1 gauge, a 4000 fpm vertical speed scale, and GEAR DOWN / GEAR UNSAFE and
SPD BRK lights.

In the Cessna and the jet the stick moves the surfaces directly. Trim off every steady pressure with
`[` and `]`; power controls altitude, pitch and trim control speed. The trainer has no
rudder pedals: an automatic rudder keeps the ball centred.

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
| [ / ] | pitch trim nose down / up, hold (C172 and jet; joystick buttons `trim_nose_down` / `trim_nose_up`) |
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
