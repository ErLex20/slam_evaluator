"""Dependency-free SE3 helpers, styled after ros2_openloris_publishers/geometry.py.

Poses/transforms are (translation, quaternion) pairs with quaternion as
(x, y, z, w), following this package's convention (see README, "Unverified
assumptions"). ``compose(t_ab, t_bc)`` and ``invert(t_ab)`` follow standard
TF semantics: ``t_xy`` is the pose of frame y expressed in frame x, i.e.
``point_in_x = t_xy.translation + rotate(t_xy.quaternion, point_in_y)``.
"""

import math


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def rotate_vector(q, vector):
    """Rotate a vector by a normalized quaternion without extra packages."""
    qx, qy, qz, qw = normalize_quaternion(q)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose(t_ab, t_bc):
    """Compose two poses: t_ab (b in a) and t_bc (c in b) -> t_ac (c in a)."""
    pos_ab, quat_ab = t_ab
    pos_bc, quat_bc = t_bc
    rotated_bc = rotate_vector(quat_ab, pos_bc)
    return (
        tuple(pos_ab[i] + rotated_bc[i] for i in range(3)),
        quaternion_multiply(quat_ab, quat_bc),
    )


def invert(t_ab):
    """Invert a pose: t_ab (b in a) -> t_ba (a in b)."""
    pos_ab, quat_ab = t_ab
    quat_ba = quaternion_conjugate(normalize_quaternion(quat_ab))
    pos_ba = rotate_vector(quat_ba, tuple(-value for value in pos_ab))
    return (pos_ba, quat_ba)


def rotation_matrix_to_quaternion(matrix):
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion."""
    m00, m01, m02 = matrix[0][0], matrix[0][1], matrix[0][2]
    m10, m11, m12 = matrix[1][0], matrix[1][1], matrix[1][2]
    m20, m21, m22 = matrix[2][0], matrix[2][1], matrix[2][2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    return normalize_quaternion(quaternion)
