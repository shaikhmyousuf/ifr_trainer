import pytest

from ifr_trainer.sim.aircraft import AircraftData
from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.scenario import Scenario
from ifr_trainer.sim.simulation import Simulation


@pytest.fixture(scope="session")
def aircraft() -> AircraftData:
    return AircraftData.load()


@pytest.fixture(scope="session")
def airport() -> Airport:
    return Airport.load()


@pytest.fixture
def make_sim(aircraft, airport):
    def make(**overrides) -> Simulation:
        fields = dict(name="test", description="", runway="36", distance_nm=12.0, altitude_ft=3500.0, cas_kt=220.0)
        fields.update(overrides)
        return Simulation(aircraft, airport, Scenario(**fields))

    return make


def fly(sim: Simulation, seconds: float, dt: float = 1 / 120) -> None:
    for _ in range(round(seconds / dt)):
        sim.step(dt, sim.inputs)
