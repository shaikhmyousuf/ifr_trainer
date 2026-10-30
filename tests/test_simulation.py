import pytest

from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.scenario import load_scenarios
from ifr_trainer.sim.simulation import Phase, wind_vector
from ifr_trainer.sim.units import DEG, FT, KT

from .conftest import fly


def test_scenarios_load_and_reference_real_runways():
    airport = Airport.load()
    for sc in load_scenarios().values():
        airport.runway(sc.runway)


def test_wind_from_west_blows_east():
    e, n = wind_vector(270.0, 20.0)
    assert e == pytest.approx(20 * KT)
    assert n == pytest.approx(0.0, abs=1e-9)


def test_crosswind_drifts_track(make_sim):
    sim = make_sim(wind_dir_deg=270.0, wind_kt=30.0)
    fly(sim, 1)
    assert sim.heading_deg == pytest.approx(0.0, abs=0.1)
    assert 3 < sim.track_deg < 10


def test_flaps_and_gear_move_over_time(make_sim, aircraft):
    sim = make_sim(cas_kt=180.0)
    sim.inputs.flap_lever = 2
    sim.inputs.gear_down = True
    fly(sim, 1)
    assert 0 < sim.flap_pos < 1 and 0 < sim.gear_pos < 1
    fly(sim, 2 * aircraft.flap_transit_s_per_step)
    assert sim.flap_pos == 2 and sim.gear_pos == 1


def _place_over_runway(sim, sink_fpm, gear=True):
    sim.state.x_m, sim.state.y_m = 0.0, 400.0
    sim.state.h_m = sim.airport.elevation_m + 0.5 * FT
    sim.state.gamma = sink_fpm * FT / 60 / sim.state.tas
    sim.state.theta = 3 * DEG
    sim.gear_pos = 1.0 if gear else 0.0
    sim.inputs.gear_down = gear


def test_gentle_touchdown_on_runway_lands(make_sim):
    sim = make_sim(cas_kt=135.0, flaps=4, gear_down=True)
    _place_over_runway(sim, -300)
    fly(sim, 1)
    assert sim.phase is Phase.LANDED


def test_gear_up_touchdown_crashes(make_sim):
    sim = make_sim(cas_kt=135.0, flaps=4)
    _place_over_runway(sim, -300, gear=False)
    fly(sim, 1)
    assert sim.phase is Phase.CRASHED
    assert "gear" in sim.message


def test_reset_restores_start(make_sim):
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 3)
    inputs = sim.reset()
    assert sim.state.phi == 0.0 and inputs.stick_roll == 0.0
