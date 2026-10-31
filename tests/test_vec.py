import math

import pytest

from ifr_trainer.sim.vec import euler_from_quat, mat_vec, quat_from_euler, rotation_matrix


def test_quaternion_euler_round_trip():
    angles = (0.3, -0.2, 4.0)
    assert euler_from_quat(quat_from_euler(*angles)) == pytest.approx(angles)


def test_rotation_matrix_points_nose_along_heading():
    north, east, down = mat_vec(rotation_matrix(quat_from_euler(0.0, 0.0, math.pi / 2)), (1.0, 0.0, 0.0))
    assert (north, east, down) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
