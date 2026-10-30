import pytest

from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.scenario import load_scenarios
from ifr_trainer.sim.simulation import Phase, Severity, Simulation, wind_vector
from ifr_trainer.sim.units import KT

from .conftest import fly, fly_until


def test_every_scenario_starts_trimmed_and_flies(aircraft):
    airport = Airport.load()
    for name, sc in load_scenarios().items():
        sim = Simulation(aircraft, airport, sc)
        alt = sim.alt_ft
        fly(sim, 5)
        assert sim.phase is Phase.FLYING, name
        assert abs(sim.alt_ft - alt - 5 * sim.vs_fpm / 60) < 30, name


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


def test_reset_restores_start_and_cockpit(make_sim):
    sim = make_sim(cockpit={"minimum_ft": 700.0, "ils_crs": 360.0})
    sim.inputs.stick_roll = 1.0
    sim.cockpit.minimum_ft = 900.0
    fly(sim, 3)
    inputs = sim.reset()
    assert sim.state.phi == pytest.approx(0.0) and inputs.stick_roll == 0.0
    assert sim.cockpit.minimum_ft == 700.0


# --- Minimum, go-around, terrain ------------------------------------------------------


def _ils_final(make_sim, **kw):
    # On the localizer and glideslope at 4 NM, configured for landing.
    fields = dict(distance_nm=4.0, altitude_ft=1830.0, cas_kt=138.0, flaps=4, gear_down=True,
                  flight_path_deg=-3.0, cockpit={"minimum_ft": 700.0})
    fields.update(kw)
    return make_sim(**fields)


def test_minimum_callout_then_go_around_is_scored(make_sim):
    sim = _ils_final(make_sim)
    assert fly_until(sim, lambda: sim.minimum_reached_time is not None, 120)
    assert sim.message.startswith("MINIMUM")
    sim.inputs.lever = 1.0  # TOGA...
    sim.inputs.stick_pitch = 0.5  # ...and rotate to the go-around attitude
    assert fly_until(sim, lambda: sim.message.startswith(("GO-AROUND", "LATE")), 20)
    assert sim.severity is Severity.INFO, sim.message
    sim.inputs.stick_pitch = 0.0
    sim.inputs.gear_down = False
    fly(sim, 20)
    assert sim.phase is Phase.FLYING and sim.vs_fpm > 500


def test_slow_go_around_is_late(make_sim):
    sim = _ils_final(make_sim)
    fly_until(sim, lambda: sim.minimum_reached_time is not None, 120)
    fly(sim, 6)  # continue descending for 6 s (~75 ft at 750 fpm) before reacting
    sim.inputs.lever = 1.0
    sim.inputs.stick_pitch = 0.5
    fly_until(sim, lambda: sim.message.startswith(("GO-AROUND", "LATE")), 20)
    assert sim.severity is Severity.CAUTION and sim.message.startswith("LATE")


def test_continuing_below_minimum_flies_into_terrain(make_sim):
    sim = _ils_final(make_sim)
    fly_until(sim, lambda: sim.phase is Phase.CRASHED, 120)
    assert sim.phase is Phase.CRASHED
    assert "terrain" in sim.message and "minimum" in sim.message


def test_no_minimum_callout_when_not_set(make_sim):
    sim = _ils_final(make_sim, cockpit={})
    fly(sim, 40)
    assert sim.minimum_reached_time is None


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
