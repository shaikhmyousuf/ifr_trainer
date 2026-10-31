import math

import pytest

from ifr_trainer.nav.navaids import Ils, Ndb, Vor
from ifr_trainer.nav.receivers import GS_DEG_PER_DOT, read_adf, read_ils, read_vor
from ifr_trainer.sim.units import FT, NM

ELEV = 500 * FT


@pytest.fixture
def ils36(airport) -> Ils:
    return next(n for n in airport.navaids if isinstance(n, Ils) and n.runway == "36")


VOR = Vor("TST", 113.0, 0.0, 0.0, ELEV)


def test_vor_inbound_on_course_is_centred_with_to_flag():
    r = read_vor(VOR, 360.0, 0.0, -10 * NM, 3000 * FT)
    assert r.radial == pytest.approx(180.0)
    assert r.to_from == "TO" and r.deviation_dots == pytest.approx(0.0, abs=1e-9)


def test_vor_needle_points_towards_the_course():
    # East of the inbound 360 course: the course is to the left.
    r = read_vor(VOR, 360.0, 1 * NM, -10 * NM, 3000 * FT)
    assert r.deviation_dots < 0
    # Outbound on 180 (FROM), east of course while heading south: course to the right.
    r = read_vor(VOR, 180.0, 1 * NM, -10 * NM, 3000 * FT)
    assert r.to_from == "FROM" and r.deviation_dots > 0


def test_vor_dot_is_five_degrees():
    x = 10 * NM * math.sin(math.radians(5.0))
    y = -10 * NM * math.cos(math.radians(5.0))
    assert read_vor(VOR, 360.0, x, y, 3000 * FT).deviation_dots == pytest.approx(-1.0, abs=1e-6)


def test_vor_dme_is_slant_range_and_cone_of_confusion_flags():
    r = read_vor(VOR, 360.0, 0.0, -100.0, ELEV + 3000.0)  # nearly overhead
    assert r.to_from is None and r.deviation_dots is None
    assert r.dme_nm == pytest.approx(math.hypot(100.0, 3000.0) / NM)


def test_vor_out_of_line_of_sight_is_not_received():
    assert read_vor(VOR, 360.0, 0.0, -50 * NM, ELEV + 500 * FT) is None  # 1.23 * sqrt(500) = 27 NM
    assert read_vor(VOR, 360.0, 0.0, -50 * NM, 10000 * FT) is not None


def test_localizer_full_scale_is_105_m_at_threshold(ils36):
    r = read_ils(ils36, -105.0, 0.0, ELEV + 50 * FT)  # 105 m left of the threshold
    assert r.loc_dots == pytest.approx(2.0, abs=0.01)  # localizer to the right: fly right


def test_localizer_sign_east_of_course_is_fly_left(ils36):
    assert read_ils(ils36, 300.0, -5 * NM, 2000 * FT).loc_dots < 0


def test_glideslope_centred_on_three_degrees_and_positive_when_high(ils36):
    d = 5 * NM
    gs_dist = ils36.glideslope_y_m + d
    h_on = ELEV + gs_dist * math.tan(math.radians(3.0))
    assert read_ils(ils36, 0.0, -d, h_on).gs_dots == pytest.approx(0.0, abs=0.02)
    h_high = ELEV + gs_dist * math.tan(math.radians(3.0 + GS_DEG_PER_DOT))
    assert read_ils(ils36, 0.0, -d, h_high).gs_dots == pytest.approx(1.0, abs=0.02)


def test_ils_coverage_limits(ils36):
    assert read_ils(ils36, 0.0, -20 * NM, 5000 * FT).gs_dots is None  # glideslope beyond 10 NM
    assert read_ils(ils36, 0.0, -20 * NM, 5000 * FT).loc_dots is not None  # localizer to 25 NM
    side = 15 * NM
    assert read_ils(ils36, side * math.tan(math.radians(40)), -side, 5000 * FT) is None  # outside 35 deg
    assert read_ils(ils36, 0.0, 10 * NM, 3000 * FT) is None  # behind the localizer


def test_adf_bearing_to_station():
    ndb = Ndb("TS", 380.0, 0.0, 0.0, ELEV)
    assert read_adf(ndb, -5 * NM, 0.0, 3000 * FT).bearing == pytest.approx(90.0)
    assert read_adf(ndb, 0.0, -40 * NM, 3000 * FT) is None  # beyond 25 NM


def test_receivers_follow_tuned_frequencies(make_sim):
    sim = make_sim(cockpit={"vor1_mhz": 113.40, "ils_mhz": 110.30, "adf1_khz": 380.0})
    r = sim.nav_readings()
    assert r.vor1.ident == "TRN" and r.ils.ident == "ITRS" and r.adf1.ident == "TR"
    sim.cockpit.vor1_mhz = 115.20
    sim.cockpit.ils_mhz = 109.50  # ILS 18: behind its localizer from the south
    sim.cockpit.adf1_khz = 200.0
    r = sim.nav_readings()
    assert r.vor1.ident == "WES" and r.ils is None and r.adf1 is None


def test_ils36_scenario_starts_east_of_localizer_outside_glideslope_range(make_sim, aircraft):
    from ifr_trainer.sim.scenario import load_scenarios

    sc = load_scenarios(aircraft)["ils36"]
    sim = make_sim(**{k: getattr(sc, k) for k in ("distance_nm", "offset_nm", "heading_deg", "altitude_ft", "cas_kt", "flaps")},
                   cockpit=sc.cockpit)
    r = sim.nav_readings()
    assert r.ils.loc_dots < -2 and r.ils.gs_dots is None
