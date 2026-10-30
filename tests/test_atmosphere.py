import pytest

from ifr_trainer.sim.atmosphere import cas_from_tas, isa, tas_from_cas
from ifr_trainer.sim.units import KT


def test_sea_level():
    air = isa(0.0)
    assert air.temperature == pytest.approx(288.15)
    assert air.pressure == pytest.approx(101325.0)
    assert air.density == pytest.approx(1.225, abs=1e-3)
    assert air.speed_of_sound == pytest.approx(340.3, abs=0.1)


def test_tropopause():
    air = isa(11000.0)
    assert air.temperature == pytest.approx(216.65)
    assert air.pressure == pytest.approx(22632, rel=1e-3)


def test_cas_equals_tas_at_sea_level():
    assert cas_from_tas(100.0, isa(0.0)) == pytest.approx(100.0, rel=1e-6)


@pytest.mark.parametrize("h", [0.0, 3000.0, 10000.0])
def test_cas_tas_round_trip(h):
    air = isa(h)
    assert cas_from_tas(tas_from_cas(130.0, air), air) == pytest.approx(130.0, rel=1e-9)


def test_tas_exceeds_cas_at_altitude():
    # 250 kt CAS at FL100 is roughly 290 kt TAS.
    assert tas_from_cas(250.0 * KT, isa(3048.0)) / KT == pytest.approx(290.0, rel=0.02)
