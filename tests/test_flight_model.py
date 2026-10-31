import math

import pytest

from ifr_trainer.sim.speeds import vs1g_kt
from ifr_trainer.sim.vec import euler_from_quat, quat_from_euler, rotation_matrix, mat_vec

from .conftest import fly


def test_quaternion_euler_round_trip():
    angles = (0.3, -0.2, 4.0)
    assert euler_from_quat(quat_from_euler(*angles)) == pytest.approx(angles)


def test_rotation_matrix_points_nose_along_heading():
    north, east, down = mat_vec(rotation_matrix(quat_from_euler(0.0, 0.0, math.pi / 2)), (1.0, 0.0, 0.0))
    assert (north, east, down) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)


def test_trimmed_level_flight_holds(make_sim):
    sim = make_sim()
    alt, cas = sim.alt_ft, sim.cas_kt
    fly(sim, 60)
    assert sim.alt_ft == pytest.approx(alt, abs=5)
    assert sim.cas_kt == pytest.approx(cas, abs=1)
    assert 50 < sim.engines.n1 < 65  # typical A320 N1 at 220 kt level
    assert abs(math.degrees(sim.surfaces.elevator)) < 0.5  # THS carries the trim


def test_approach_on_glidepath_is_realistic(make_sim):
    sim = make_sim(altitude_ft=1500.0, cas_kt=138.0, flaps=4, gear_down=True, flight_path_deg=-3.0)
    fly(sim, 10)
    assert 50 < sim.engines.n1 < 62
    assert 2.0 < math.degrees(sim.state.theta) < 4.5
    assert sim.vs_fpm == pytest.approx(-740, abs=60)
    assert 0 < math.degrees(sim.surfaces.ths) < 13.5


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
    fly(sim, 3)
    bank = math.degrees(sim.state.phi)
    worst_vs = 0.0
    for _ in range(20 * 120):
        sim.step(1 / 120, sim.inputs)
        worst_vs = max(worst_vs, abs(sim.vs_fpm))
    assert math.degrees(sim.state.phi) == pytest.approx(bank, abs=1.0)
    assert worst_vs < 150
    assert abs(math.degrees(sim.state.beta)) < 0.5  # turn coordinated without pedals


@pytest.mark.parametrize("cas_kt, flaps", [(220.0, 0), (200.0, 1), (135.0, 4)])
def test_pitch_settles_after_stick_release(make_sim, cas_kt, flaps):
    # Releasing the stick after a pull must not start a pitch oscillation (at approach
    # speeds the old load factor loop limit-cycled against the elevator rate limit),
    # and the C* law keeps the nose close to where the pilot left it.
    sim = make_sim(cas_kt=cas_kt, flaps=flaps)
    sim.inputs.stick_pitch = 0.5
    fly(sim, 2)
    sim.inputs.stick_pitch = 0.0
    fly(sim, 0.7)  # pitch rate has stopped
    released_pitch = math.degrees(sim.state.theta)
    fly(sim, 3.3)
    rates = []
    for _ in range(6 * 120):
        sim.step(1 / 120, sim.inputs)
        rates.append(math.degrees(sim.state.q))
    assert max(rates) - min(rates) < 0.5
    assert released_pitch - math.degrees(sim.state.theta) < 2.0


def test_bank_beyond_33_returns_to_33(make_sim):
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 4)
    assert math.degrees(sim.state.phi) > 50
    sim.inputs.stick_roll = 0.0
    fly(sim, 15)
    assert math.degrees(sim.state.phi) == pytest.approx(33, abs=1.5)


def test_full_stick_bank_limited_to_67(make_sim):
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 15)
    assert math.degrees(sim.state.phi) == pytest.approx(67, abs=2)


def test_pitch_attitude_limited(make_sim):
    sim = make_sim(cas_kt=300.0)
    sim.inputs.stick_pitch = 1.0
    sim.inputs.lever = 1.0
    fly(sim, 15)
    assert math.degrees(sim.state.theta) < 31


def test_full_back_stick_at_idle_does_not_stall(make_sim):
    sim = make_sim()
    sim.inputs.stick_pitch = 1.0
    sim.inputs.lever = 0.0
    alpha_stall = sim.model.aero.alpha_stall(sim.aero())
    max_alpha = 0.0
    for _ in range(60 * 120):
        sim.step(1 / 120, sim.inputs)
        max_alpha = max(max_alpha, sim.state.alpha)
    s = sim.state
    assert all(math.isfinite(v) for v in (s.tas, s.alpha, s.h_m, s.q))
    assert max_alpha < alpha_stall


def test_crosswind_crabs_with_zero_sideslip(make_sim):
    sim = make_sim(wind_dir_deg=270.0, wind_kt=30.0)
    fly(sim, 10)
    assert abs(math.degrees(sim.state.beta)) < 0.5
    assert 3 < sim.track_deg - sim.heading_deg < 10
