"""Turbofan model, with the A320's engine data."""

import pytest

from ifr_trainer.sim.atmosphere import isa
from ifr_trainer.sim.turbofan import Turbofan


def test_lever_to_n1_monotonic(aircraft):
    e = Turbofan(aircraft.engines)
    values = [e.lever_to_n1(i / 100) for i in range(101)]
    assert values == sorted(values)
    assert values[0] == aircraft.engines.n1_idle
    assert e.lever_to_n1(aircraft.engines.detents["CL"]) == pytest.approx(89.0)


def test_spool_up_idle_to_toga_takes_several_seconds(aircraft):
    e = Turbofan(aircraft.engines)
    air = isa(0.0)
    toga = e.thrust_at(e.lever_to_n1(1.0), air, 0.0)
    t, dt = 0.0, 1 / 120
    while e.thrust(air, 0.0) < 0.9 * toga:
        e.step(dt, 1.0)
        t += dt
    assert 5.0 < t < 9.0


def test_thrust_lapses_with_altitude(aircraft):
    e = Turbofan(aircraft.engines)
    assert e.thrust_at(90.0, isa(10000.0), 0.78) < 0.5 * e.thrust_at(90.0, isa(0.0), 0.0)
