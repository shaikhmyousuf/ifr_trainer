# IFR Trainer — Plan

A lightweight instrument-flying trainer for practising **hand flying** with
**A320-style displays**. It is loosely based on the A320, not a replica, and not
a certified training device.

**Scope: instrument flying only.** Every exercise starts in the air and is flown
in cloud. Approaches end at the minimum with a go-around on instruments. Takeoff,
landing, ground handling and pilot rudder are out of scope (the rudder only
coordinates turns automatically).

## Decisions

| Topic | Decision |
|---|---|
| Purpose | Hand flying under IFR: raw-data approaches to minima and go-around, holds, tracking, partial panel. No autopilot, no takeoff or landing. |
| Controls | Joystick (sidestick); keyboard fallback for testing |
| Flight model | Our own, written in Python — no compiled flight dynamics library |
| Rendering | pygame-ce, 2D only. No outside view (we are in cloud) |
| World | Fictional airport **XIFR "Trainer Field"**, single runway 36/18 |

## Why it can be light and still realistic

* **No outside view.** Everything drawn is 2D glass-cockpit symbology.
* **Fly-by-wire does most of the "feel".** In normal law the stick commands load
  factor (pitch) and roll rate, with automatic trim and a flight path that holds
  when you let go. A good normal law on top of a reasonable aerodynamic model
  feels more like an Airbus than a perfect aero model would without one.
* **Fixed 120 Hz simulation step**, separate from a 60 Hz render rate.
* Static artwork is cached; only moving symbols are redrawn each frame.

## Architecture

```
src/ifr_trainer/
├── data/          aircraft, airport, scenarios, controls (TOML)
├── sim/           atmosphere, aircraft data, engines, flight model, speeds,
│                  trim, airport, scenario, simulation (orchestrator)
├── fbw/           normal law (later: protections, alternate/direct law)
├── nav/           navaids and VOR / DME / ILS / ADF receivers
├── input/         joystick, mouse and keyboard, joystick test tool
├── displays/      PFD, ND, lower panel (engines, config, radios, EFIS), (M6) standby
├── training/      (M5) scenarios, scoring, recorder, debrief
└── app.py         main loop
```

## Flight model progression

1. **M1 — Point mass in wind axes.** State: position, altitude, TAS, flight path
   angle, heading, bank, load factor. Lift is limited by CLmax (so it can stall),
   drag is a drag polar plus extra drag from flaps, gear, speedbrake and Mach.
   Engine N1 spools up slowly from idle. A simple normal law drives it. Without a
   pitch axis this already gives the Airbus feel of flight-path stability with
   neutral speed stability.
2. **M2 — 6 degrees of freedom.** Rigid body with quaternion attitude, aero
   coefficients from linear stability derivatives, sideslip, automatic turn
   coordination. (A ground model was built, then removed when takeoff and landing
   left the scope.)
3. **M3 — Full normal law** on the 6-DOF model: C* pitch law, roll-rate command,
   protections (high angle of attack and alpha floor, overspeed, pitch/bank).

Data is approximate A320-200: 64 t, wing area 122.6 m², CFM56-class engines,
CONF 0/1/2/3/FULL. It is tuned against figures such as about 3° nose-up pitch at
Vapp on a 3° glidepath, approach N1 around 55–60 %, and VLS in CONF FULL around
133 kt at 64 t.

## Airport XIFR — Trainer Field (fictional)

* Elevation 500 ft; magnetic variation 0° (true heading = magnetic heading).
* Origin of the local frame is the RWY 36 threshold. x is east and y is north,
  in metres.
* RWY 36/18: 3000 m × 45 m, heading 360°/180°.
* ILS 36 `ITRS` 110.30, ILS 18 `ITRN` 109.50, 3.0° glideslope, DME at the glideslope.
* VOR/DME `TRN` 113.40 on the field; VOR/DME `WES` 115.20, 15 NM west and 10 NM north.
* NDB `TR` 380 kHz, 4 NM south (locator for RWY 36); NDB `TN` 335 kHz, 4 NM north (RWY 18).
* Minima: ILS DA 700 ft (200 ft above the field), VOR/DME MDA 1000 ft, NDB MDA 1100 ft.

## Milestones

1. **Core loop** — window, fixed timestep, joystick, mouse and keyboard input,
   ISA atmosphere, point-mass model with simple normal law, PFD, engine/flap/gear panel.
2. **6-DOF model**, plus tests for trim and stability. *Done:* rigid body with
   quaternion attitude and stability-derivative aerodynamics with stall. The normal
   law is turned into surface deflections by dynamic inversion, with an
   auto-trimming THS and automatic turn coordination.
3. **Normal law and protections**: full high angle-of-attack protection and alpha
   floor, high-speed protection, and alternate/direct law for failure training.
4. **Navigation**. *Done:* VOR (radial, TO/FROM, 5°/dot), DME (slant range),
   ILS (localizer full scale 105 m at the threshold, glideslope 0.35°/dot, ICAO
   coverage), ADF; line-of-sight range and the VOR cone of confusion. ND in ROSE
   ILS / ROSE VOR / ARC with bearing needles; PFD ILS scales, heading bug, altitude
   target, BARO minimum. Clickable radios and EFIS/FCU panel. Minimum callout,
   go-around scoring and terrain impact. ILS, VOR/DME, NDB, tracking and hold
   scenarios.
5. **Training layer**: freeze/reposition, turbulence, flight recorder, approach
   scoring (localizer/glideslope/speed deviations, go-around quality), debrief plots,
   hold entry and timing aids.
6. **Failures**: unreliable airspeed, partial panel on standby instruments,
   engine failure, degraded laws.

Each milestone keeps pytest coverage of the physics so tuning cannot silently
break the handling.
