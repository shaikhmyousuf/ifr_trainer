# IFR Trainer — Plan

A lightweight instrument-flying trainer for practising **hand flying**. Aircraft
are **modules**: they share one environment but each has its own handling, engines,
flight controls and displays. The first module is loosely based on the A320; a
Cessna 172P module is built from open data; a generic jet is planned. Not a certified
training device.

**Scope: instrument flying only.** Every exercise starts in the air and is flown
in cloud. Approaches end at the minimum with a go-around on instruments. Takeoff,
landing, ground handling and pilot rudder are out of scope (the rudder only
coordinates turns automatically).

## Decisions

| Topic | Decision |
|---|---|
| Purpose | Hand flying under IFR: raw-data approaches to minima and go-around, holds, tracking, partial panel. No autopilot, no takeoff or landing. |
| Aircraft | Modular: A320 and Cessna 172P (done), a generic jet (planned), in the same environment |
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
├── app.py         main loop, aircraft-independent
├── data/          shared data: airport, scenarios (common part), joystick mapping
├── sim/           the shared environment and physics:
│                  atmosphere, airport, scenario, cockpit (radios, references, minimum),
│                  airframe data, stability-derivative aero, 6-DOF flight model, trim,
│                  turbofan engines (for any jet), direct controls (for aircraft without
│                  fly-by-wire), simulation (orchestrator, monitoring),
│                  aircraft.py: the interface every aircraft module implements
├── nav/           navaids and VOR / DME / ILS / ADF receivers
├── input/         joystick, mouse and keyboard, joystick setup and test tools
├── displays/      shared drawing helpers, the panel interface, round-dial gauges and
│                  the round-dial panel (SteamPanel) used by the C172
└── aircraft/      aircraft modules, listed in aircraft/__init__.py
    └── a320/      aircraft.toml, scenarios.toml, A320 class, fly-by-wire (fbw/),
                   speeds (VLS, green dot), cockpit (EFIS settings),
                   displays (PFD, ND, lower panel, layout)
    ├── c172/      aircraft.toml, scenarios.toml, piston engine and propeller, panel
    └── citation/  aircraft.toml, scenarios.toml, speeds, cockpit, displays (PFD, ND and
                   lower panel: a copy of the A320's, adapted)
```

**The aircraft interface** (`sim/aircraft.py`). The simulation asks an aircraft for:

| Part | A320 | Cessna 172P | Business jet |
|---|---|---|---|
| Airframe data (`aircraft.toml`, shared format) | approximate A320-200 | JSBSim `c172p` | TU Delft Citation II |
| Engines (`Propulsion`) | `Turbofan` (shared) | piston engine, fixed-pitch propeller (JSBSim `prop_75in2f`), RPM as a state | `Turbofan` (shared) |
| Flight controls (`FlightControls`) | normal law by dynamic inversion | `DirectControls` (shared): stick moves the surfaces, manual trim, automatic rudder | `DirectControls`, with stick expo |
| Cockpit state | radios + EFIS settings | `Nav1Cockpit` (shared): NAV1 tunes VOR or ILS | radios + EFIS settings (copy of the A320's) |
| Panel (displays) | PFD, ND, lower panel | `SteamPanel` + tachometer | PFD, ND, lower panel (copy of the A320's, adapted) |
| Scenario settings (`scenarios.toml`) | speeds, CONF, ND modes | speeds, flaps, NAV1 on the localizer | speeds, flaps, gear, ND modes |

Scenarios are split the same way: `data/scenarios.toml` holds the exercise (position,
altitude, wind, radios, minimum); each aircraft adds how it flies it. The A320's
handling was verified bit-for-bit identical across the refactor.

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
CONF 0/1/2/3/FULL. Airbus publishes no aerodynamic data, so the coefficients are
estimates, checked in `tests/a320/test_performance.py` against published figures and
rules of thumb (all at 64 t):

* **Matches**: VS1G 150 kt clean and 108 kt CONF FULL (VLS 133); best lift-to-drag
  speed within 6 % of green dot from 50 to 75 t; 3.3° nose up and N1 about 55 % at VAPP
  on a 3° glidepath; idle descent at 250 kt of 2.9° (the 3:1 rule); level idle
  deceleration about 12 kt per NM; L/D 17.2 at FL350 M0.78.
* **Fitted**: the thrust density exponent (0.95), to put REC MAX near FL390 and cruise
  N1 near 83 %. At 0.75 there was 1100 fpm of climb left at FL390.
* **Known gaps**: the lift slope does not rise with Mach, so cruise angle of attack is
  about 3.4° (real aircraft about 2.5°); cruise L/D is a little optimistic; low-altitude
  climb rate at CL thrust (about 4000 fpm at 250 kt, FL100) is unverified.

## Cessna 172P data

Sources: JSBSim `aircraft/c172p`, `engine/eng_io320` and `engine/prop_75in2f`
(github.com/JSBSim-Team/jsbsim), and the C172P published figures (2400 lb, 160 BHP at
2700 RPM, VNE 158, VNO 127, VFE 110 / 85 KIAS, stall 51 / 46 KCAS, 700 fpm, 123 kt).

* **Taken directly from JSBSim**: geometry, inertia, lift curve (C<sub>Lα</sub> 4.41,
  C<sub>Lmax</sub> 1.47 at 16°), flap lift, drag and pitching moment, all moment and side-force
  derivatives, control travel, propeller thrust and power tables, engine manifold pressure.
  JSBSim's aerodynamic reference point (2.2 in behind the CG) is folded into C<sub>m</sub>.
* **Fitted to the published performance**: the drag polar (C<sub>D0</sub> 0.035, e 0.90) and an
  engine friction term that grows with RPM². JSBSim's own drag table used in this linear model
  gave 137 kt and 420 fpm.
* **Result vs the C172P book**: full throttle 124 kt (123), climb 630 fpm at 74 kt (700),
  static 2240 RPM (2300 to 2420), stall 52.6 / 47.3 KCAS (51 / 46), 72% power at 8000 ft.
  Best climb comes at about 84 kt rather than 74: JSBSim's propeller is relatively
  inefficient at low speed.
* **Not modelled**: propeller wash on the tail, P-factor and torque (the automatic rudder
  hides them), mixture, carburettor icing, gyro precession.

## Business jet data (Cessna Citation II)

Sources, all open: the TU Delft flight dynamics course data for the PH-LAB Citation II
(`Cit_par`: linear stability derivatives and radii of gyration, in the course's student
repositories on GitHub); five student groups' flight-test measurements of the values the
course leaves for the flight test (C<sub>D0</sub>, e, C<sub>Lα</sub>, C<sub>mα</sub>,
C<sub>mδe</sub>); Delfosse et al., EuroGNC 2022 (CC BY 4.0: geometry, masses, stall angle);
a TU Eindhoven thesis (elevator travel, linear model modes); FAA TCDS A22CE and published
figures (JT15D-4 2500 lbf, flap and gear limits, VMO 262 / MMO 0.705, 82 KCAS landing stall,
43,000 ft).

* **Taken directly from the data**: geometry, inertia (from the Delft radii of gyration),
  all moment and force derivatives (pitch-rate and α̇ terms doubled: Delft uses q c / V,
  this simulator q c / 2V), elevator travel; the flight-measured median C<sub>D0</sub> 0.022,
  e 0.81, C<sub>Lα</sub> 4.5, C<sub>mα</sub> −0.60, C<sub>mδe</sub> −1.3.
* **Fitted to the published performance**: the turbofan thrust lapse (density exponent
  0.92, Mach lapse 0.30 plus 0.8 σ² at low altitude, a term added to the shared turbofan
  model with a default of 0). Result: 3350 fpm sea-level climb at 6033 kg (book 3370),
  389 KTAS at FL350 (385), ceiling 43,000 ft.
* **Estimated**: flap lift and drag increments (set so the landing stall is 82 KCAS and the
  stall angle stays near Delfosse's 14°; approach attitude 1.8° at VREF + 5), gear and
  speedbrake drag, aileron and rudder travel, trim range.
* **Checked against the Delft / Eindhoven models**: short period ζ 0.39, ω<sub>n</sub> 4.2 rad/s
  at 5000 m, 166 m/s (model 0.43, 4.59); phugoid period 73 s (74). Phugoid damping is lower
  (0.10 against 0.16): this simulator has no C<sub>Xu</sub> term.
* **Known gaps**: the roll time constant (about 0.2 s) is shorter than the 0.33 to 0.5 s of
  DLR's Citation model; the Delft aileron yaw is slightly proverse; no engine-out asymmetry
  (engines are lumped). The stick has an expo curve: the elevator is powerful against the
  static stability (C<sub>mδe</sub> / C<sub>mα</sub> ≈ 2.2, the C172 0.57) and the trainer
  has no control forces.

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
   *Done so far:* C* pitch law (load factor blended with pitch rate, crossover
   120 m/s) with bank compensation feedforward; auto-trim freezes beyond 33° of bank,
   outside 0.5–1.25 g and at the angle-of-attack limit.
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
