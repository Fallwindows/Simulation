"""Small quaternion/rigid-transform toolkit with no numeric dependency."""

from __future__ import annotations

import math
from dataclasses import dataclass


Quaternion = tuple[float, float, float, float]
Vector3 = tuple[float, float, float]


def quaternion_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0:
        raise ValueError("zero quaternion")
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def quaternion_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    r, p, y = (math.radians(v) / 2.0 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return quaternion_normalize((sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy))


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
