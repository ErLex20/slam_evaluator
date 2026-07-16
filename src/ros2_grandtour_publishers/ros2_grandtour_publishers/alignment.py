"""Offline alignment helpers for disconnected GrandTour reference frames."""

import numpy as np

from . import geometry


def nearest_timestamp_matches(query_timestamps, reference_timestamps,
                              max_difference):
    """Match each query timestamp to its nearest reference timestamp.

    Returns query indices, reference indices, and absolute timestamp deltas
    for matches no farther apart than ``max_difference`` seconds.
    """
    query = np.asarray(query_timestamps, dtype=np.float64)
    reference = np.asarray(reference_timestamps, dtype=np.float64)
    if query.ndim != 1 or reference.ndim != 1:
        raise ValueError('timestamps must be one-dimensional')
    if reference.size == 0:
        return (
            np.empty(0, dtype=np.int64),
            np.empty(0, dtype=np.int64),
            np.empty(0, dtype=np.float64),
        )

    right = np.searchsorted(reference, query)
    right = np.clip(right, 0, reference.size - 1)
    left = np.maximum(right - 1, 0)
    use_left = np.abs(reference[left] - query) <= np.abs(
        reference[right] - query)
    nearest = np.where(use_left, left, right)
    deltas = np.abs(reference[nearest] - query)
    valid = deltas <= float(max_difference)
    return np.flatnonzero(valid), nearest[valid], deltas[valid]


def fit_rigid_transform(source_points, target_points):
    """Fit T(target, source) with a no-scale Kabsch/Umeyama alignment.

    The returned translation and quaternion transform ``source_points`` into
    ``target_points``. The third return value is the translational RMSE.
    """
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError('source and target points must have matching Nx3 shapes')
    if source.shape[0] < 3:
        raise ValueError('at least three point pairs are required')

    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center

    u_matrix, _, vt_matrix = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_center - rotation @ source_center

    transformed = source @ rotation.T + translation
    residuals = transformed - target
    rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    quaternion = geometry.rotation_matrix_to_quaternion(rotation)
    return tuple(translation.tolist()), quaternion, rmse
