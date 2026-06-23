"""Small dependency-free pose helpers used by the dataset adapters."""

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


def relative_pose(origin_position, origin_orientation, position, orientation):
    inverse_origin = quaternion_conjugate(
        normalize_quaternion(origin_orientation)
    )
    delta = tuple(position[i] - origin_position[i] for i in range(3))
    return (
        rotate_vector(inverse_origin, delta),
        quaternion_multiply(inverse_origin, orientation),
    )
