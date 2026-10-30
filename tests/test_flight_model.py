import math

import pytest

from ifr_trainer.sim.speeds import vs1g_kt
from ifr_trainer.sim.trim import trim_lever
from ifr_trainer.sim.units import DEG

from .conftest import fly


def test_trimmed_level_flight_holds(make_sim):
    sim = make_sim()
    alt, cas = sim.alt_ft, sim.cas_kt
    fly(sim, 60)
    assert sim.alt_ft == pytest.approx(alt, abs=5)
    assert sim.cas_kt == pytest.approx(cas, abs=1)
    assert 50 < sim.engines.n1 < 65  # typical A320 N1 at 220 kt level


def test_approach_on_glidepath_is_realistic(make_sim):
    sim = make_sim(altitude_ft=1500.0, cas_kt=138.0, flaps=4, gear_down=True)
    sim.state.gamma = -3.0 * DEG
    lever = trim_lever(sim.model, sim.engines, sim.state, sim.aero(), sim.mass, gamma=-3.0 * DEG)
    sim.engines.set_steady(lever)
    sim.inputs.lever = lever
    fly(sim, 10)
    assert 50 < sim.engines.n1 < 62
    assert 2.0 < math.degrees(sim.state.theta) < 4.5
    assert sim.vs_fpm == pytest.approx(-740, abs=60)


def test_vls_conf_full(aircraft):
    assert 1.23 * vs1g_kt(aircraft, 64000, aircraft.flaps[-1].cl_max) == pytest.approx(133, abs=4)


def test_idle_in_level_flight_decelerates(make_sim):
    sim = make_sim()
    sim.inputs.lever = 0.0
    fly(sim, 20)
    assert sim.cas_kt < 210


def test_neutral_stick_holds_bank_and_flight_path_in_turn(make_sim):
    # Normal law holds the flight path (not altitude) with automatic bank compensation.
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 25 / 15)
    sim.inputs.stick_roll = 0.0
    fly(sim, 2)
    bank, vs = math.degrees(sim.state.phi), sim.vs_fpm
    fly(sim, 20)
    assert math.degrees(sim.state.phi) == pytest.approx(bank, abs=0.5)
    assert abs(vs) < 200
    assert sim.vs_fpm == pytest.approx(vs, abs=30)


def test_bank_beyond_33_returns_to_33(make_sim):
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 4)
    assert math.degrees(sim.state.phi) > 50
    sim.inputs.stick_roll = 0.0
    fly(sim, 15)
    assert math.degrees(sim.state.phi) == pytest.approx(33, abs=1)


def test_full_stick_bank_limited_to_67(make_sim):
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 15)
    assert math.degrees(sim.state.phi) == pytest.approx(67, abs=1)


def test_pitch_attitude_limited(make_sim):
    sim = make_sim(cas_kt=300.0)
    sim.inputs.stick_pitch = 1.0
    sim.inputs.lever = 1.0
    fly(sim, 15)
    assert math.degrees(sim.state.theta) < 31


def test_full_back_stick_at_idle_stalls_without_blowing_up(make_sim):
    sim = make_sim()
    sim.inputs.stick_pitch = 1.0
    sim.inputs.lever = 0.0
    fly(sim, 60)
    s = sim.state
    assert all(math.isfinite(v) for v in (s.tas, s.gamma, s.h_m, s.nz))
    assert s.lift_limited
    assert sim.cas_kt < 170
