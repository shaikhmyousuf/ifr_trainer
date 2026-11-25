"""Every registered aircraft module must plug into the shared simulation.

These tests run for each aircraft in ``ifr_trainer.aircraft.AIRCRAFT``, so a new aircraft
is checked as soon as it is registered.
"""

import pytest

from ifr_trainer.aircraft import AIRCRAFT, load_aircraft
from ifr_trainer.sim.aircraft import Aircraft
from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.cockpit import Cockpit
from ifr_trainer.sim.scenario import load_scenarios, merge_scenarios
from ifr_trainer.sim.simulation import Phase, Simulation

from .conftest import fly


@pytest.fixture(scope="module", params=sorted(AIRCRAFT))
def each_aircraft(request) -> Aircraft:
    return load_aircraft(request.param)


def test_unknown_aircraft_is_rejected():
    with pytest.raises(KeyError, match="available"):
        load_aircraft("concorde")


def test_aircraft_module_provides_the_interface(each_aircraft):
    ac = each_aircraft
    assert isinstance(ac, Aircraft) and ac.id in AIRCRAFT and ac.title
    assert len(ac.airframe.flaps) >= 1
    propulsion = ac.new_propulsion()
    assert all(0.0 <= v <= 1.0 for v in propulsion.detents.values())
    assert isinstance(ac.new_cockpit({}), Cockpit)


def test_every_scenario_starts_trimmed_and_flies(each_aircraft):
    airport = Airport.load()
    scenarios = load_scenarios(each_aircraft)
    assert scenarios, "an aircraft must offer at least one scenario"
    for name, sc in scenarios.items():
        sim = Simulation(each_aircraft, airport, sc)
        alt = sim.alt_ft
        fly(sim, 5)
        assert sim.phase is Phase.FLYING, name
        assert abs(sim.alt_ft - alt - 5 * sim.vs_fpm / 60) < 30, name


def test_panel_draws_every_scenario(each_aircraft, monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    import pygame

    from ifr_trainer.displays.common import Text

    pygame.init()
    panel = each_aircraft.new_panel(Text())
    screen = pygame.display.set_mode(panel.size)
    airport = Airport.load()
    for name, sc in load_scenarios(each_aircraft).items():
        sim = Simulation(each_aircraft, airport, sc)
        if hasattr(sim.cockpit, "fpv"):
            sim.cockpit.fpv = True  # glass cockpits: draw the flight path vector too
        fly(sim, 0.5)
        panel.draw(screen, sim, throttle_needs_sync=True)
        assert not panel.click(sim, (-100, -100), False), name  # outside the window: not a panel field


def test_aircraft_settings_merge_over_shared_scenarios():
    shared = {
        "ils": {"description": "ILS", "runway": "36", "distance_nm": 10.0, "altitude_ft": 3000.0,
                "cockpit": {"ils_mhz": 110.3, "minimum_ft": 700.0}},
        "hold": {"description": "Hold", "runway": "36", "distance_nm": 0.0, "altitude_ft": 4000.0},
    }
    settings = {
        "ils": {"cas_kt": 90.0, "altitude_ft": 2000.0, "cockpit": {"minimum_ft": 650.0}},
        "own": {"description": "Own", "runway": "18", "distance_nm": 5.0, "altitude_ft": 2500.0, "cas_kt": 100.0},
    }
    merged = merge_scenarios(shared, settings)
    assert list(merged) == ["ils", "own"]  # "hold" is not offered: the aircraft does not list it
    ils = merged["ils"]
    assert ils.cas_kt == 90.0 and ils.altitude_ft == 2000.0 and ils.distance_nm == 10.0
    assert ils.cockpit == {"ils_mhz": 110.3, "minimum_ft": 650.0}
