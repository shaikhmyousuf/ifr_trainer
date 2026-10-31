"""Business jet (Citation II data): performance against the published figures, dynamics against
the TU Delft / TU Eindhoven Citation II models, and the handling of a jet without fly-by-wire.
"""

import math

import pytest

from ifr_trainer.aircraft import load_aircraft
from ifr_trainer.sim.aero import AeroConfig, AeroModel
from ifr_trainer.sim.atmosphere import RHO0, cas_from_tas, isa, tas_from_cas
from ifr_trainer.sim.scenario import Scenario
from ifr_trainer.sim.simulation import Phase, Severity, Simulation
from ifr_trainer.sim.trim import trim_flight
from ifr_trainer.sim.units import FPM, FT, G, KT

from ..conftest import fly, fly_until

MTOW = 6033.0  # 13,300 lb
MLW = 5760.0  # 12,700 lb


@pytest.fixture(scope="module")
def jet():
    return load_aircraft("citation")


@pytest.fixture
def make(jet, airport):
    def make_sim(**overrides) -> Simulation:
        fields = dict(name="test", description="", runway="36", distance_nm=30.0, altitude_ft=5000.0, cas_kt=180.0)
        fields.update(overrides)
        return Simulation(jet, airport, Scenario(**fields))

    return make_sim


def mean_cas(sim, seconds):
    total, n = 0.0, round(seconds * 120)
    for _ in range(n):
        sim.step(1 / 120, sim.inputs)
        total += sim.cas_kt
    return total / n


def record(sim, seconds, value):
    out = []
    for _ in range(round(seconds * 120)):
        sim.step(1 / 120, sim.inputs)
        out.append(value())
    return out


def oscillation(signal, dt):
    """Period, damping ratio and natural frequency from the first two positive peaks."""
    peaks = [i for i in range(1, len(signal) - 1) if signal[i - 1] < signal[i] >= signal[i + 1] and signal[i] > 0]
    period = (peaks[1] - peaks[0]) * dt
    decrement = math.log(signal[peaks[0]] / signal[peaks[1]])
    zeta = decrement / math.sqrt(4 * math.pi**2 + decrement**2)
    return period, zeta, 2 * math.pi / period / math.sqrt(1 - zeta**2)


def _config(af, flaps=0):
    c = af.flaps[flaps]
    return AeroConfig(af.cl0 + c.cl0_delta, c.cl_max, af.cd0 + c.cd0_delta, af.aero.cm0 + c.cm0_delta)


def _steady_climb_fpm(jet, alt_ft, mass, cas_kt=None, mach=None):
    """Climb rate at full thrust, clean, holding the speed."""
    af, prop = jet.airframe, jet.new_propulsion()
    air = isa(alt_ft * FT)
    tas = mach * air.speed_of_sound if mach else tas_from_cas(cas_kt * KT, air)
    lo, hi = -10.0, 25.0
    for _ in range(40):
        mid = (lo + hi) / 2
        trim = trim_flight(af, AeroModel(af), prop, _config(af), mass, alt_ft * FT, tas, math.radians(mid))
        lo, hi = (mid, hi) if prop.steady_thrust(1.0, air, tas / air.speed_of_sound) > trim.thrust else (lo, mid)
    return tas * math.sin(math.radians(lo)) / FPM


# --- Book performance --------------------------------------------------------------------


def test_landing_stall_speed_matches_book(jet):
    af = jet.airframe
    vso = math.sqrt(2 * MLW * G / (RHO0 * af.wing_area_m2 * af.flaps[-1].cl_max)) / KT
    assert vso == pytest.approx(82, abs=3)  # published: 82 KCAS, flaps 40 and gear down


def test_sea_level_climb_at_mtow_near_book(jet):
    assert _steady_climb_fpm(jet, 0, MTOW, cas_kt=160) == pytest.approx(3370, abs=250)


def test_cruise_speed_at_fl350_near_book(jet):
    af, prop = jet.airframe, jet.new_propulsion()
    air = isa(35000 * FT)
    lo, hi = 300.0, 450.0
    for _ in range(40):
        mid = (lo + hi) / 2
        trim = trim_flight(af, AeroModel(af), prop, _config(af), af.mass_kg, 35000 * FT, mid * KT)
        lo, hi = (mid, hi) if prop.steady_thrust(1.0, air, mid * KT / air.speed_of_sound) > trim.thrust else (lo, mid)
    assert lo == pytest.approx(385, abs=15)  # KTAS
    assert lo * KT / air.speed_of_sound < 0.705  # just below MMO


def test_ceiling_near_43000_ft(jet):
    assert _steady_climb_fpm(jet, 41000, jet.airframe.mass_kg, mach=0.62) > 100
    assert _steady_climb_fpm(jet, 45000, jet.airframe.mass_kg, mach=0.62) < 100


# --- Dynamics against the Delft and Eindhoven models ---------------------------------------


def test_short_period_matches_citation_model(make):
    # TU Eindhoven linear model at 5000 m, 166 m/s: damping 0.43, natural frequency 4.59 rad/s.
    sim = make(altitude_ft=5000 / FT, cas_kt=cas_from_tas(166.0, isa(5000.0)) / KT)
    fly(sim, 2)
    sim.inputs.stick_pitch = 0.3
    fly(sim, 0.15)
    sim.inputs.stick_pitch = 0.0
    q = record(sim, 4, lambda: sim.state.q)
    _, zeta, wn = oscillation([v - q[-1] for v in q], 1 / 120)
    assert zeta == pytest.approx(0.43, abs=0.08)
    assert wn == pytest.approx(4.59, rel=0.15)


def test_phugoid_period_matches_citation_model(make):
    # Same model: phugoid natural frequency 0.0846 rad/s (74 s period), damping 0.16.
    sim = make(altitude_ft=5000 / FT, cas_kt=cas_from_tas(166.0, isa(5000.0)) / KT)
    fly(sim, 2)
    sim.inputs.stick_pitch = 0.15
    fly(sim, 1)
    sim.inputs.stick_pitch = 0.0
    cas = record(sim, 300, lambda: sim.cas_kt)[::120]
    settled = sum(cas[-60:]) / 60
    period, zeta, _ = oscillation([v - settled for v in cas], 1.0)
    assert period == pytest.approx(74, rel=0.1)
    assert 0.03 < zeta < 0.25


# --- Handling ----------------------------------------------------------------------------


def test_trimmed_level_flight_holds_hands_off(make):
    sim = make()
    fly(sim, 60)
    assert sim.alt_ft == pytest.approx(5000, abs=15)
    assert sim.cas_kt == pytest.approx(180, abs=1)
    assert abs(math.degrees(sim.surfaces.elevator)) < 0.1  # the trim carries the load


def test_approach_attitude_and_thrust(jet, make):
    vref = 1.3 * math.sqrt(2 * jet.airframe.mass_kg * G / (RHO0 * jet.airframe.wing_area_m2 * jet.airframe.flaps[-1].cl_max)) / KT
    sim = make(altitude_ft=1500.0, cas_kt=vref + 10, flaps=2, gear_down=True, flight_path_deg=-3.0)
    fly(sim, 10)
    assert 0.0 < math.degrees(sim.state.theta) < 4.0  # a flat attitude on a 3 degree glide path
    assert 50 < sim.propulsion.n1 < 75


def test_released_stick_returns_to_trimmed_speed(make):
    sim = make()
    sim.inputs.stick_pitch = 0.15
    fly(sim, 3)
    sim.inputs.stick_pitch = 0.0
    fly(sim, 240)
    assert mean_cas(sim, 60) == pytest.approx(180, abs=5)


def test_trim_switch_changes_the_trimmed_speed(make):
    sim = make()
    sim.inputs.pitch_trim = 1.0  # nose up
    fly(sim, 2)
    sim.inputs.pitch_trim = 0.0
    fly(sim, 180)
    assert mean_cas(sim, 60) < 170


def test_speedbrake_about_doubles_the_idle_descent(make):
    def idle_descent_fpm(speedbrake):
        sim = make(altitude_ft=10000.0, cas_kt=220.0)
        sim.inputs.lever = 0.0
        sim.inputs.speedbrake = speedbrake
        for _ in range(90 * 120):  # hold 220 kt with the stick
            sim.inputs.stick_pitch = max(-0.5, min(0.5, 0.05 * (sim.cas_kt - 220.0)))
            sim.step(1 / 120, sim.inputs)
        assert sim.cas_kt == pytest.approx(220, abs=3)
        return sim.vs_fpm

    clean, brakes = idle_descent_fpm(0.0), idle_descent_fpm(1.0)
    assert -1600 < clean < -800
    assert 1.6 < brakes / clean < 3.0


def test_automatic_rudder_keeps_turns_coordinated(make):
    sim = make()
    sim.inputs.stick_roll = 0.3
    fly_until(sim, lambda: math.degrees(sim.state.phi) > 25, 5)
    sim.inputs.stick_roll = 0.0
    fly(sim, 2)
    worst = max(abs(math.degrees(b)) for b in record(sim, 20, lambda: sim.state.beta))
    assert worst < 1.0


def test_go_around_at_minimum_on_instruments(make):
    # ILS final at 3 NM, VREF + 10, flaps 40, gear down. At DA: full thrust, rotate to 10 degrees
    # nose up and hold it, flaps 15, then gear up once climbing.
    gs_alt = 500 + (3 * 1852 + 300) * math.tan(math.radians(3)) / FT
    sim = make(distance_nm=3.0, altitude_ft=gs_alt, cas_kt=118.0, flaps=2, gear_down=True, flight_path_deg=-3.0,
               cockpit={"minimum_ft": 700.0})
    assert fly_until(sim, lambda: sim.minimum_reached_time is not None, 120)
    sim.inputs.lever = 1.0
    sim.inputs.flap_lever = 1
    slowest = sim.cas_kt
    for i in range(25 * 120):
        sim.inputs.stick_pitch = max(-1.0, min(1.0, 0.08 * (10.0 - math.degrees(sim.state.theta)) - 0.3 * sim.state.q))
        if i == 5 * 120:
            sim.inputs.gear_down = False
        sim.step(1 / 120, sim.inputs)
        slowest = min(slowest, sim.cas_kt)
    assert sim.message.startswith("GO-AROUND") and sim.severity is Severity.INFO, sim.message
    assert sim.phase is Phase.FLYING
    assert sim.vs_fpm > 1500
    assert slowest > 105  # stays above VREF while the flaps come up
