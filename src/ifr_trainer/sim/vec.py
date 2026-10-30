"""3-vector and quaternion helpers. Pure Python is faster than numpy for vectors this small.

Quaternions are (w, x, y, z) and rotate body axes into NED (north, east, down) axes.
"""

import math

Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]
Mat = tuple[Vec, Vec, Vec]


def cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quat_from_euler(phi: float, theta: float, psi: float) -> Quat:
    cr, sr = math.cos(phi / 2), math.sin(phi / 2)
    cp, sp = math.cos(theta / 2), math.sin(theta / 2)
    cy, sy = math.cos(psi / 2), math.sin(psi / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def euler_from_quat(q: Quat) -> Vec:
    """(phi, theta, psi) in radians; psi in [0, 2 pi)."""
    w, x, y, z = q
    phi = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    theta = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    psi = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) % math.tau
    return phi, theta, psi


def rotation_matrix(q: Quat) -> Mat:
    """Body-to-NED rotation matrix."""
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def mat_vec(m: Mat, v: Vec) -> Vec:
    """m @ v (body -> NED for a rotation matrix)."""
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def mat_t_vec(m: Mat, v: Vec) -> Vec:
    """m.T @ v (NED -> body for a rotation matrix)."""
    return (
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    )


def quat_integrate(q: Quat, p: float, q_rate: float, r: float, dt: float) -> Quat:
    """Advance the attitude by body rates (p, q, r) over dt, renormalised."""
    w, x, y, z = q
    h = 0.5 * dt
    w, x, y, z = (
        w + h * (-x * p - y * q_rate - z * r),
        x + h * (w * p + y * r - z * q_rate),
        y + h * (w * q_rate - x * r + z * p),
        z + h * (w * r + x * q_rate - y * p),
    )
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)
