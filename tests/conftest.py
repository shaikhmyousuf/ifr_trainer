import pytest

from ifr_trainer.aircraft import load_aircraft
from ifr_trainer.aircraft.a320 import A320
from ifr_trainer.sim.airport import Airport
from ifr_trainer.sim.scenario import Scenario
from ifr_trainer.sim.simulation import Phase, Simulation


@pytest.fixture(scope="session")
def aircraft() -> A320:
    """The reference aircraft for tests of the shared simulation."""
    return load_aircraft("a320")


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
        if sim.phase is Phase.CRASHED:
            return
        sim.step(dt, sim.inputs)


def fly_until(sim: Simulation, condition, max_seconds: float, dt: float = 1 / 120) -> bool:
    for _ in range(round(max_seconds / dt)):
        if condition():
            return True
        if sim.phase is Phase.CRASHED:
            return False
        sim.step(dt, sim.inputs)
    return condition()
