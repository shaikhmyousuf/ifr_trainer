"""Cessna 172 handling and performance, against the C172P published figures.

The C172 has no flight control computer: these tests check the characteristics that make
it fly differently from the A320 (speed stability, manual trim, power for altitude, a
slightly unstable spiral) as well as its book performance.
"""

import math

import pytest

from ifr_trainer.aircraft import load_aircraft
from ifr_trainer.sim.aero import AeroConfig, AeroModel
from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.atmosphere import RHO0, isa, tas_from_cas
from ifr_trainer.sim.scenario import Scenario
from ifr_trainer.sim.simulation import Phase, Severity, Simulation
from ifr_trainer.sim.trim import trim_flight
from ifr_trainer.sim.units import FPM, FT, G, KT

from ..conftest import fly, fly_until


@pytest.fixture(scope="module")
def c172():
    return load_aircraft("c172")


@pytest.fixture
def make(c172, airport):
    def make_sim(**overrides) -> Simulation:
        fields = dict(name="test", description="", runway="36", distance_nm=20.0, altitude_ft=3000.0, cas_kt=90.0)
        fields.update(overrides)
        return Simulation(c172, airport, Scenario(**fields))

    return make_sim


def mean_cas(sim, seconds):
    total, n = 0.0, round(seconds * 120)
    for _ in range(n):
        sim.step(1 / 120, sim.inputs)
        total += sim.cas_kt
    return total / n


def _clean(af):
    c = af.flaps[0]
    return AeroConfig(af.cl0 + c.cl0_delta, c.cl_max, af.cd0 + c.cd0_delta, af.aero.cm0 + c.cm0_delta)


def _full_throttle_excess(c172, cas_kt, h_m, gamma_deg=0.0):
    """Thrust available at full throttle minus the thrust needed for steady flight."""
    af, prop = c172.airframe, c172.new_propulsion()
    air = isa(h_m)
    tas = tas_from_cas(cas_kt * KT, air)
    trim = trim_flight(af, AeroModel(af), prop, _clean(af), af.mass_kg, h_m, tas, math.radians(gamma_deg))
    return prop.steady_thrust(1.0, air, tas / air.speed_of_sound) - trim.thrust, tas, prop


# --- Book performance --------------------------------------------------------------------


def test_stall_speeds_match_book(c172):
    af = c172.airframe
    vs = [math.sqrt(2 * af.mass_kg * G / (RHO0 * af.wing_area_m2 * cfg.cl_max)) / KT for cfg in (af.flaps[0], af.flaps[-1])]
    assert vs[0] == pytest.approx(51, abs=3)  # POH: 51 KCAS flaps up, 2400 lb
    assert vs[1] == pytest.approx(46, abs=3)  # POH: 46 KCAS flaps down


def test_full_throttle_speed_at_sea_level_near_book(c172):
    lo, hi = 70.0, 170.0
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _full_throttle_excess(c172, mid, 0.0)[0] > 0 else (lo, mid)
    _, tas, prop = _full_throttle_excess(c172, lo, 0.0)
    assert tas / KT == pytest.approx(123, abs=5)  # POH: 123 kt
    assert prop.steady_rps(1.0, isa(0.0), tas) * 60 < 2750


def test_full_throttle_climb_near_book(c172):
    lo, hi = 0.0, 15.0
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _full_throttle_excess(c172, 74, 0.0, mid)[0] > 0 else (lo, mid)
    _, tas, _ = _full_throttle_excess(c172, 74, 0.0, lo)
    assert tas * math.sin(math.radians(lo)) / FPM == pytest.approx(700, abs=110)  # POH: 700 fpm


def test_static_rpm_and_fixed_pitch_rpm_rises_with_speed(c172):
    prop = c172.new_propulsion()
    air = isa(0.0)
    static = prop.steady_rps(1.0, air, 0.0) * 60
    assert 2150 < static < 2420  # POH: 2300 to 2420
    assert prop.steady_rps(1.0, air, 50.0) * 60 > static + 200


# --- Handling ----------------------------------------------------------------------------


def test_trimmed_cruise_holds_hands_off(make):
    sim = make()
    fly(sim, 60)
    assert sim.alt_ft == pytest.approx(3000, abs=10)
    assert sim.cas_kt == pytest.approx(90, abs=1)
    assert 2000 < sim.propulsion.rpm < 2300
    assert abs(math.degrees(sim.surfaces.elevator)) < 0.1  # the trim carries the load


def test_released_stick_returns_to_trimmed_speed(make):
    # Speed stability: after a pull the aircraft porpoises (the phugoid) back to its trimmed speed.
    sim = make()
    sim.inputs.stick_pitch = 0.2
    fly(sim, 3)
    sim.inputs.stick_pitch = 0.0
    fly(sim, 150)
    assert mean_cas(sim, 30) == pytest.approx(90, abs=3)


def test_steady_back_pressure_settles_at_a_lower_speed(make):
    # Unlike the A320's normal law, holding the stick back does not keep the nose rising.
    sim = make()
    sim.inputs.stick_pitch = 0.1
    fly(sim, 90)
    assert 70 < mean_cas(sim, 40) < 87
    assert math.degrees(sim.state.theta) < 15


def test_trim_switch_changes_the_trimmed_speed(make):
    sim = make()
    sim.inputs.pitch_trim = 1.0  # nose up
    fly(sim, 4)
    sim.inputs.pitch_trim = 0.0
    fly(sim, 90)
    assert mean_cas(sim, 40) < 85


def test_power_climbs_at_about_the_same_speed(make):
    sim = make()
    sim.inputs.lever = 1.0
    fly(sim, 10)
    vs = []
    for _ in range(20 * 120):
        sim.step(1 / 120, sim.inputs)
        vs.append(sim.vs_fpm)
    assert sum(vs) / len(vs) > 250
    assert sim.cas_kt == pytest.approx(90, abs=6)


def test_automatic_rudder_keeps_turns_coordinated(make):
    sim = make()
    sim.inputs.stick_roll = 0.3
    fly_until(sim, lambda: math.degrees(sim.state.phi) > 20, 5)
    sim.inputs.stick_roll = 0.0
    fly(sim, 2)
    worst = 0.0
    for _ in range(20 * 120):
        sim.step(1 / 120, sim.inputs)
        worst = max(worst, abs(math.degrees(sim.state.beta)))
    assert worst < 1.0


def test_spiral_mode_slowly_tightens(make):
    # Released in a 20 degree bank, a C172 slowly overbanks into a descending spiral.
    sim = make()
    sim.inputs.stick_roll = 0.3
    fly_until(sim, lambda: math.degrees(sim.state.phi) > 20, 5)
    sim.inputs.stick_roll = 0.0
    fly(sim, 30)
    assert 22 < math.degrees(sim.state.phi) < 45
    assert sim.vs_fpm < -200


def test_go_around_at_minimum_on_instruments(make):
    # ILS final at 2 NM, 70 kt, flaps 20 (about 370 fpm down); at the minimum: full throttle, pitch up.
    gs_alt = 500 + (2 * 1852 + 300) * math.tan(math.radians(3)) / FT
    sim = make(distance_nm=2.0, altitude_ft=gs_alt, cas_kt=70.0, flaps=2, flight_path_deg=-3.0,
               cockpit={"minimum_ft": 700.0})
    assert fly_until(sim, lambda: sim.minimum_reached_time is not None, 120)
    sim.inputs.lever = 1.0
    sim.inputs.stick_pitch = 0.3
    assert fly_until(sim, lambda: sim.message.startswith(("GO-AROUND", "LATE")), 20)
    assert sim.severity is Severity.INFO, sim.message
    sim.inputs.stick_pitch = 0.0
    sim.inputs.flap_lever = 1
    fly(sim, 20)
    assert sim.phase is Phase.FLYING and sim.vs_fpm > 200
