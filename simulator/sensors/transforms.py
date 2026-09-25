"""Small quaternion/rigid-transform toolkit with no numeric dependency."""

from __future__ import annotations

import math
from dataclasses import dataclass


Quaternion = tuple[float, float, float, float]
Vector3 = tuple[float, float, float]


def quaternion_xyzw_to_wxyz(q: Quaternion) -> Quaternion:
    """Convert the ROS/geometry ``xyzw`` order to Isaac's ``wxyz`` order."""
    x, y, z, w = q
    return (w, x, y, z)


def quaternion_wxyz_to_xyzw(q: Quaternion) -> Quaternion:
    """Convert Isaac's ``wxyz`` order to the ROS/geometry ``xyzw`` order."""
    w, x, y, z = q
    return (x, y, z, w)


def quaternion_normalize(q: Quaternion) -> Quaternion:
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError("quaternion must contain four finite values")
    # hypot scales its inputs before squaring, avoiding overflow for otherwise
    # valid finite quaternions such as (1e308, 0, 0, 0).
    norm = math.hypot(*q)
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    result = tuple(v / norm for v in q)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("normalized quaternion must be finite")
    return result  # type: ignore[return-value]


def interpolate_position(start: Vector3, end: Vector3, fraction: float) -> Vector3:
    """Linearly interpolate two finite positions for ``fraction`` in ``[0, 1]``."""

    fraction = float(fraction)
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("interpolation fraction must be finite and in [0, 1]")
    if len(start) != 3 or len(end) != 3 or not all(math.isfinite(v) for v in (*start, *end)):
        raise ValueError("positions must contain three finite values")
    try:
        result = tuple(math.fsum(((1.0 - fraction) * a, fraction * b)) for a, b in zip(start, end))
    except OverflowError as exc:
        raise ValueError("interpolated position must be finite") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError("interpolated position must be finite")
    return result  # type: ignore[return-value]


def quaternion_slerp(start: Quaternion, end: Quaternion, fraction: float) -> Quaternion:
    """Interpolate unit orientation on the shortest arc in ROS ``xyzw`` order.

    Quaternion signs describe the same rotation.  Flipping the second input
    when the dot product is negative prevents a visually discontinuous long
    rotation through that equivalent sign boundary.
    """

    fraction = float(fraction)
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("interpolation fraction must be finite and in [0, 1]")
    left = quaternion_normalize(start)
    right = quaternion_normalize(end)
    dot = sum(a * b for a, b in zip(left, right))
    if dot < 0.0:
        right = tuple(-value for value in right)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quaternion_normalize(tuple(a + fraction * (b - a) for a, b in zip(left, right)))  # type: ignore[arg-type]
    angle = math.acos(dot)
    scale = math.sin(angle)
    left_scale = math.sin((1.0 - fraction) * angle) / scale
    right_scale = math.sin(fraction * angle) / scale
    return quaternion_normalize(tuple(left_scale * a + right_scale * b for a, b in zip(left, right)))  # type: ignore[arg-type]


def quaternion_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    r, p, y = (math.radians(v) / 2.0 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return quaternion_normalize((sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy))


def rpy_deg_from_quaternion(q: Quaternion) -> tuple[float, float, float]:
    """Convert an ``xyzw`` quaternion to XYZ roll/pitch/yaw degrees.

    The conversion is the inverse of :func:`quaternion_from_rpy_deg` for the
    non-singular orientations used by the sensor trajectories.  Keeping this
    helper in the dependency-free transform module lets the Isaac runtime
    apply the complete sampled pose to USD without importing Isaac in tests.
    """
    x, y, z, w = quaternion_normalize(q)

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))  # type: ignore[return-value]


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    p = (v[0], v[1], v[2], 0.0)
    r = quaternion_multiply(quaternion_multiply(q, p), quaternion_conjugate(q))
    return (r[0], r[1], r[2])


@dataclass(frozen=True)
class Transform:
    parent: str
    child: str
    translation_m: Vector3
    rotation_xyzw: Quaternion


def interpolate_transform(start: Transform, end: Transform, fraction: float) -> Transform:
    """Interpolate compatible rigid transforms without changing frame meaning."""

    if (start.parent, start.child) != (end.parent, end.child):
        raise ValueError("transform frames must match for interpolation")
    return Transform(
        start.parent,
        start.child,
        interpolate_position(start.translation_m, end.translation_m, fraction),
        quaternion_slerp(start.rotation_xyzw, end.rotation_xyzw, fraction),
    )


def compose(parent_to_mid: Transform, mid_to_child: Transform) -> Transform:
    if parent_to_mid.child != mid_to_child.parent:
        raise ValueError("transform frames do not compose")
    offset = rotate_vector(parent_to_mid.rotation_xyzw, mid_to_child.translation_m)
    translation = tuple(a + b for a, b in zip(parent_to_mid.translation_m, offset))
    rotation = quaternion_normalize(quaternion_multiply(parent_to_mid.rotation_xyzw, mid_to_child.rotation_xyzw))
    return Transform(parent_to_mid.parent, mid_to_child.child, translation, rotation)  # type: ignore[arg-type]


def transform_point(transform: Transform, point_m: Vector3) -> Vector3:
    rotated = rotate_vector(transform.rotation_xyzw, point_m)
    return tuple(a + b for a, b in zip(transform.translation_m, rotated))  # type: ignore[return-value]


def camera_optical_quaternion() -> Quaternion:
    """Return camera_link -> ROS optical frame rotation.

    camera_link uses x-forward/y-left/z-up; the optical frame uses
    x-right/y-down/z-forward.
    """
    # Columns of this matrix are optical basis vectors expressed in link axes.
    # The equivalent quaternion is (0.5, -0.5, 0.5, -0.5), up to sign.
    return (0.5, -0.5, 0.5, -0.5)


def camera_usd_quaternion() -> Quaternion:
    """Return the USD camera-local basis expressed in ``camera_link``.

    Isaac cameras look along local ``-Z`` and use local ``+Y`` as up.  The
    camera-link convention is ``+X`` forward, ``+Y`` left, ``+Z`` up, so the
    basis is USD forward ``-Z`` -> link ``+X``; USD up ``+Y`` -> link ``+Z``;
    USD right ``+X`` -> link ``-Y``.
    """
    return (0.5, -0.5, -0.5, 0.5)
