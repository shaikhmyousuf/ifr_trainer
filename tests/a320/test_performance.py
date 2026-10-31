"""A320 performance against published Airbus figures and operational rules of thumb.

Airbus does not publish aerodynamic data, so the A320 coefficients are estimates. These
tests check them where the aerodynamics show up in figures pilots know: the speeds on the
speed tape, approach attitude and thrust, idle descent gradients, deceleration, cruise
lift-to-drag ratio and the recommended maximum altitude. All at 64 t unless stated.
"""

import math

import pytest

from ifr_trainer.aircraft.a320.speeds import vs1g_kt
from ifr_trainer.sim.aero import AeroConfig, AeroModel
from ifr_trainer.sim.atmosphere import isa, tas_from_cas
from ifr_trainer.sim.trim import trim_flight
from ifr_trainer.sim.units import FPM, FT, G, KT

MASS = 64000.0
CLIMB = 0.556  # thrust lever in the CL detent


def _config(af, flaps=0, gear=0.0):
    c = af.flaps[flaps]
    return AeroConfig(af.cl0 + c.cl0_delta, c.cl_max, af.cd0 + c.cd0_delta + af.gear.cd * gear, af.aero.cm0 + c.cm0_delta)


def _trim(aircraft, alt_ft, cas_kt=None, mach=None, mass=MASS, flaps=0, gear=0.0, gamma_deg=0.0):
    af, prop = aircraft.airframe, aircraft.new_propulsion()
    air = isa(alt_ft * FT)
    tas = mach * air.speed_of_sound if mach else tas_from_cas(cas_kt * KT, air)
    trim = trim_flight(af, AeroModel(af), prop, _config(af, flaps, gear), mass, alt_ft * FT, tas, math.radians(gamma_deg))
    return trim, prop, air, tas


def _steady_gamma_deg(aircraft, lever, alt_ft, cas_kt=None, mach=None, mass=MASS):
    """Flight path angle at which this lever position holds the speed, clean."""
    lo, hi = -10.0, 15.0
    for _ in range(40):
        mid = (lo + hi) / 2
        trim, prop, air, tas = _trim(aircraft, alt_ft, cas_kt, mach, mass, gamma_deg=mid)
        lo, hi = (mid, hi) if prop.steady_thrust(lever, air, tas / air.speed_of_sound) > trim.thrust else (lo, mid)
    return lo, tas


def _vertical_speed_fpm(gamma_deg, tas):
    return tas * math.sin(math.radians(gamma_deg)) / FPM


# --- Speeds --------------------------------------------------------------------------------


def test_stall_speeds(aircraft):
    af = aircraft.airframe
    assert vs1g_kt(af, MASS, af.flaps[0].cl_max) == pytest.approx(148, abs=6)  # VS1G clean
    assert vs1g_kt(af, MASS, af.flaps[-1].cl_max) == pytest.approx(108, abs=4)  # VS1G CONF FULL (VLS 133)


@pytest.mark.parametrize("mass", [50000.0, 64000.0, 75000.0])
def test_green_dot_is_close_to_best_lift_to_drag_speed(aircraft, mass):
    # Green dot (2 kt per tonne + 85 kt below FL200) approximates the best lift-to-drag speed.
    green_dot = 2.0 * mass / 1000.0 + 85.0
    floor = round(1.3 * vs1g_kt(aircraft.airframe, mass, aircraft.airframe.flaps[0].cl_max))
    best = min(range(floor, 300), key=lambda cas: _trim(aircraft, 10000, cas, mass=mass)[0].thrust)
    assert best == pytest.approx(green_dot, rel=0.06)


# --- Approach ------------------------------------------------------------------------------


@pytest.mark.parametrize("mass", [55000.0, 64000.0, 70000.0])
def test_final_approach_attitude_and_thrust(aircraft, mass):
    # CONF FULL, gear down, VLS + 5 kt on a 3 degree glide path: about 2.5 to 3.5 degrees nose up, N1 in the mid-50s.
    vapp = 1.23 * vs1g_kt(aircraft.airframe, mass, aircraft.airframe.flaps[-1].cl_max) + 5.0
    trim, prop, _, _ = _trim(aircraft, 1500, vapp, mass=mass, flaps=4, gear=1.0, gamma_deg=-3.0)
    assert 2.0 < math.degrees(trim.alpha) - 3.0 < 4.0
    assert 50 < prop.lever_to_n1(trim.lever) < 62


# --- Descent and deceleration -------------------------------------------------------------


def test_idle_descent_at_250_kt_is_about_three_degrees(aircraft):
    # The 3:1 rule: about 3 NM per 1000 ft at idle, 250 kt below FL100.
    gamma, tas = _steady_gamma_deg(aircraft, 0.0, 10000, cas_kt=250)
    assert gamma == pytest.approx(-3.1, abs=0.5)
    assert -2000 < _vertical_speed_fpm(gamma, tas) < -1300


def test_idle_descent_at_cruise_mach(aircraft):
    gamma, tas = _steady_gamma_deg(aircraft, 0.0, 35000, mach=0.78)
    assert -3200 < _vertical_speed_fpm(gamma, tas) < -2000


def test_level_idle_deceleration_is_about_10_kt_per_nm(aircraft):
    trim, prop, air, tas = _trim(aircraft, 10000, 250)
    decel = (trim.thrust - prop.steady_thrust(0.0, air, tas / air.speed_of_sound)) / MASS  # m/s^2
    kt_per_nm = decel / KT * (1852.0 / tas)
    assert kt_per_nm == pytest.approx(10, abs=3)


# --- Cruise and ceiling --------------------------------------------------------------------


def test_cruise_lift_to_drag_and_thrust(aircraft):
    # FL350 M0.78: L/D about 16 to 17 (fuel flow about 2,400 kg/h), N1 in the low 80s.
    trim, prop, _, _ = _trim(aircraft, 35000, mach=0.78)
    assert MASS * G / trim.thrust == pytest.approx(16.5, abs=1.2)
    assert 80 < prop.lever_to_n1(trim.lever) < 87


def test_recommended_maximum_altitude_near_fl390(aircraft):
    # REC MAX: the altitude where climb thrust still gives 300 fpm; about FL390 at 64 t.
    def climb_fpm(alt_ft):
        return _vertical_speed_fpm(*_steady_gamma_deg(aircraft, CLIMB, alt_ft, mach=0.78))

    assert climb_fpm(37000) > 300
    assert 0 < climb_fpm(39000) < 500
    assert climb_fpm(41000) < 300
