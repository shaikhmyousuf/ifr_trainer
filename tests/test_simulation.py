import math

import pytest

from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.scenario import load_scenarios
from ifr_trainer.sim.simulation import Phase, Severity, wind_vector
from ifr_trainer.sim.units import KT

from .conftest import fly, fly_until


def test_scenarios_load_and_reference_real_runways():
    airport = Airport.load()
    for sc in load_scenarios().values():
        airport.runway(sc.runway)


def test_wind_from_west_blows_east():
    e, n = wind_vector(270.0, 20.0)
    assert e == pytest.approx(20 * KT)
    assert n == pytest.approx(0.0, abs=1e-9)


def test_flaps_and_gear_move_over_time(make_sim, aircraft):
    sim = make_sim(cas_kt=180.0)
    sim.inputs.flap_lever = 2
    sim.inputs.gear_down = True
    fly(sim, 1)
    assert 0 < sim.flap_pos < 1 and 0 < sim.gear_pos < 1
    fly(sim, 2 * aircraft.flap_transit_s_per_step)
    assert sim.flap_pos == 2 and sim.gear_pos == 1


def test_reset_restores_start(make_sim):
    sim = make_sim()
    sim.inputs.stick_roll = 1.0
    fly(sim, 3)
    inputs = sim.reset()
    assert sim.state.phi == pytest.approx(0.0) and inputs.stick_roll == 0.0


# --- Ground ---------------------------------------------------------------------------


def _takeoff_sim(make_sim, **kw):
    return make_sim(distance_nm=-0.03, on_ground=True, flaps=1, **kw)


def test_parked_aircraft_sits_still_on_its_gear(make_sim, aircraft):
    sim = _takeoff_sim(make_sim)
    x, y = sim.state.x_m, sim.state.y_m
    fly(sim, 10)
    loads = sim.state.gear.loads
    total = sum(loads.values())
    assert total == pytest.approx(sim.mass * 9.80665, rel=0.02)
    assert 0.05 < loads["nose"] / total < 0.15  # nosewheel carries ~10 % of the weight
    assert math.hypot(sim.state.x_m - x, sim.state.y_m - y) < 0.5
    assert sim.phase is Phase.ON_GROUND and sim.fcc.law == "GROUND"


def test_takeoff_roll_rotation_and_liftoff(make_sim):
    sim = _takeoff_sim(make_sim)
    rwy = sim.airport.runway("36")
    sim.inputs.brakes = 0.0
    sim.inputs.lever = 1.0
    assert fly_until(sim, lambda: sim.cas_kt >= 145, 45)
    assert 25 < sim.time_s < 40  # A320 at TOGA: about half a minute to VR
    along, cross = rwy.to_runway_frame(sim.state.x_m, sim.state.y_m)
    assert along < 1300 and abs(cross) < 1
    sim.inputs.stick_pitch = 0.4
    assert fly_until(sim, lambda: sim.phase is Phase.AIRBORNE, 10)
    sim.inputs.stick_pitch = 0.0
    sim.inputs.gear_down = False
    fly(sim, 20)
    assert sim.phase is Phase.AIRBORNE
    assert sim.fcc.law == "NORMAL"
    assert sim.vs_fpm > 1000 and sim.agl_ft > 500


def test_full_pedal_at_speed_leaves_the_runway(make_sim):
    sim = _takeoff_sim(make_sim)
    sim.inputs.brakes = 0.0
    sim.inputs.lever = 1.0
    fly_until(sim, lambda: sim.cas_kt >= 50, 30)
    sim.inputs.rudder = 1.0
    fly(sim, 20)
    assert sim.phase is Phase.CRASHED and "excursion" in sim.message


def _short_final(make_sim, **kw):
    fields = dict(distance_nm=0.4, altitude_ft=660.0, cas_kt=138.0, flaps=4, gear_down=True, flight_path_deg=-3.0)
    fields.update(kw)
    return make_sim(**fields)


def _flare_and_stop(sim):
    sim.max_ground_spoiler = 0.0
    touched = False
    for _ in range(120 * 120):
        if sim.phase is Phase.CRASHED:
            return
        if not touched and sim.agl_ft < 30:
            sim.inputs.stick_pitch = max(-0.3, min(0.5, (-150 - sim.vs_fpm) / 1500))
            sim.inputs.lever = 0.0
        if sim.state.on_ground:
            touched = True
            sim.inputs.stick_pitch = 0.0
            sim.inputs.brakes = 1.0
        sim.step(1 / 120, sim.inputs)
        sim.max_ground_spoiler = max(sim.max_ground_spoiler, sim.surfaces.ground_spoiler)
        if touched and sim.groundspeed_kt < 0.5:
            return


def test_flared_landing_touches_down_gently_and_stops(make_sim):
    sim = _short_final(make_sim)
    _flare_and_stop(sim)
    assert sim.phase is Phase.ON_GROUND
    assert sim.severity is Severity.INFO
    assert sim.message.startswith("TOUCHDOWN") and "Stopped" in sim.message
    assert sim.max_ground_spoiler > 0.9


def test_landing_without_flare_is_hard(make_sim):
    sim = _short_final(make_sim)
    fly_until(sim, lambda: sim.state.on_ground, 60)
    fly(sim, 0.5)
    assert sim.severity is Severity.CAUTION and sim.message.startswith("HARD LANDING")


def test_gear_up_landing_crashes(make_sim):
    sim = _short_final(make_sim, gear_down=False)
    _flare_and_stop(sim)
    assert sim.phase is Phase.CRASHED


def test_steep_approach_crashes(make_sim):
    sim = _short_final(make_sim, flight_path_deg=-6.0)
    fly_until(sim, lambda: sim.state.on_ground, 60)
    fly(sim, 0.5)
    assert sim.phase is Phase.CRASHED


def test_mouse_drag_deflects_stick_and_release_centres(aircraft, monkeypatch):
    import pygame

    from ifr_trainer.input.controls import MOUSE_STICK_RANGE_PX, Controls, JoystickMapping

    pygame.init()
    pygame.display.set_mode((100, 100))
    controls = Controls(JoystickMapping(), aircraft.engines.detents, len(aircraft.flaps))
    controls.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(500, 300)))
    monkeypatch.setattr(pygame.mouse, "get_pos", lambda: (500 + MOUSE_STICK_RANGE_PX / 2, 300 + MOUSE_STICK_RANGE_PX))
    inputs = controls.poll(0.016)
    assert inputs.stick_roll == pytest.approx(0.5)
    assert inputs.stick_pitch == pytest.approx(1.0)  # dragged down = full back stick
    controls.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=(0, 0)))
    inputs = controls.poll(0.016)
    assert inputs.stick_roll == 0.0 and inputs.stick_pitch == 0.0
    controls.handle_event(pygame.event.Event(pygame.MOUSEWHEEL, x=0, y=3))
    assert inputs.lever == pytest.approx(0.06)
